#include <control/cvt_controller.h>

// Keep includes close to where symbols are used.
// If your project already centralizes these in a precompiled header, you can
// remove duplicates here—but this keeps the file self-sufficient.
#include <Arduino.h>
#include <constants.h>
#include <macros.h>
#include <types.h>
#include <filters/iirfilter.h>
#include <filters/median_filter.h>
#include <hardware/odrive.h>
#include <hardware/actuator.h>
#include <hardware/ecenterlock.h>
#include <hardware/can_bus.h>
#include <hardware/sensors.h>
#include <logging/logger.h>
#include <control_function_state.pb.h>  // Nanopb: ControlFunctionState_* symbols
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <cstring>

// ------- Externs for existing singletons / globals (unchanged behavior) -------
extern ODrive odrive;
extern ODrive ecenterlock_odrive;
extern Actuator actuator;
extern Ecenterlock ecenterlock;

extern IIRFilter engine_rpm_rotation_filter;
extern IIRFilter engine_rpm_time_filter;
extern IIRFilter engine_rpm_derror_filter;
extern IIRFilter gear_rpm_time_filter;
extern IIRFilter throttle_filter;

extern MedianFilter engine_rpm_median_filter;

extern bool sd_initialized;
extern bool using_ecenterlock;
extern bool serial_logging;

// Logger globals / helpers (leave as-is; we’re just calling them)
extern volatile bool logging_disconnected;
extern u8 cur_buffer_num;
extern LogBuffer double_buffer[2];
extern u8 message_buffer[MESSAGE_BUFFER_SIZE];
extern size_t encode_pb_message(u8 buffer[], size_t buffer_length, u8 id,
                                const pb_msgdesc_t* fields,
                                const void* message_struct);
extern u8 write_to_double_buffer(u8 data[], size_t data_length,
                                 LogBuffer double_buffer[2], u8* buffer_num,
                                 bool split);

// Sensors (as defined in sensors.cpp)
extern Sensor engine_sensor;
extern Sensor gear_sensor;
extern Sensor lw_gear_sensor;
extern Sensor rw_gear_sensor;

// CAN helper if you used it for the dashboard
extern void send_command(uint32_t node_id, uint8_t id, uint8_t sub_id, float value);

// Buttons array (pins) lives in constants.h; the *previous* last_button_state
// was a global in main.cpp. We now keep our own copy inside CvtController.

// ------------------------- Static trampoline wiring --------------------------
CvtController* CvtController::instance_ = nullptr;

void CvtController::isrTrampoline() {
  if (instance_) instance_->tick();
}

void CvtController::tick() {
  switch (mode_) {
    case Mode::Normal:      updateNormal();      break;
    case Mode::ButtonShift: updateButtonShift(); break;
    case Mode::Debug:       updateDebug();       break;
  }
}

// Replaced globals `control_state`, `control_cycle_count`, `last_throttle`,
//     `last_engine_rpm_error`, `last_secondary_rpm`, `last_button_state` with
//     the class members: control_state_, control_cycle_count_, last_throttle_,
//     last_engine_rpm_error_, last_secondary_rpm_, last_button_state_[].


void CvtController::updateButtonShift() {
  bool button_pressed[5] = {false, false, false, false, false};

  for (size_t i = 0; i < 5; i++) {
    button_pressed[i] = !digitalRead(BUTTON_PINS[i]) && last_button_state_[i];
  }
  for (size_t i = 0; i < 5; i++) {
    last_button_state_[i] = digitalRead(BUTTON_PINS[i]);
  }

  Serial.printf("State: %d, Velocity: %f, Out: %d, Engage: %d, In: %d,\n",
                odrive.get_axis_state(), odrive.get_vel_estimate(),
                actuator.get_outbound_limit(), actuator.get_engage_limit(),
                actuator.get_inbound_limit());

  float velocity = 50.0f;
  if (button_pressed[0]) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  } else if (button_pressed[1]) {
    odrive.set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  } else if (button_pressed[2]) {
    actuator.set_velocity(-velocity);
  } else if (button_pressed[3]) {
    actuator.set_velocity(0.0f);
  } else if (button_pressed[4]) {
    actuator.set_velocity(velocity);
  }

  control_cycle_count_++;
}

void CvtController::updateDebug() {
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;
  control_cycle_count_++;

  // (Keep your LED/limit switch debug snippets identical)
  if (digitalRead(LIMIT_SWITCH_IN_PIN) == LOW) {
    digitalWrite(LED_3_PIN, HIGH);
  } else {
    digitalWrite(LED_3_PIN, LOW);
  }

  // control_state reset + timestamp
  control_state_ = ControlFunctionState_init_default;  // or _init_zero per your nanopb
  control_state_.cycle_start_us = micros();

  // Read raw sensors
  control_state_.raw_throttle = analogRead(THROTTLE_SENSOR_PIN);
  control_state_.raw_brake    = analogRead(BRAKE_SENSOR_PIN);

  // Scale and clamp
  control_state_.throttle =
      map_int_to_float(control_state_.raw_throttle, THROTTLE_MIN_VALUE,
                       THROTTLE_MAX_VALUE, 0.0, 1.0);
  control_state_.throttle = CLAMP(control_state_.throttle, 0.0, 1.0);

  control_state_.brake =
      map_int_to_float(control_state_.raw_brake, BRAKE_MIN_VALUE,
                       BRAKE_MAX_VALUE, 0.0, 1.0);
  control_state_.brake = CLAMP(control_state_.brake, 0.0, 1.0);

  control_state_.throttle_filtered = throttle_filter.update(control_state_.throttle);

  control_state_.d_throttle =
      (control_state_.throttle_filtered - last_throttle_) * (1.0f / dt_s);
  last_throttle_ = control_state_.throttle_filtered;

  // Snapshot counter/timing from ISRs (atomic section)
  noInterrupts();
  control_state_.engine_count   = engine_sensor.get_count();
  control_state_.gear_count     = gear_sensor.get_count();
  control_state_.lw_gear_count  = lw_gear_sensor.get_count();
  control_state_.rw_gear_count  = rw_gear_sensor.get_count();

  float cur_engine_time_diff_us      = engine_sensor.get_time_diff();
  float cur_filt_engine_time_diff_us = engine_sensor.get_filtered_time_diff();
  float cur_gear_time_diff_us        = gear_sensor.get_time_diff();
  float lw_cur_gear_time_diff_us     = lw_gear_sensor.get_time_diff();
  float rw_cur_gear_time_diff_us     = rw_gear_sensor.get_time_diff();
  interrupts();

  // Engine RPM (instant + filtered), same calculations as before
  control_state_.engine_rpm = 0.0f;
  if (cur_engine_time_diff_us != 0.0f) {
    control_state_.engine_rpm =
        ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
        cur_engine_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;

    control_state_.filtered_engine_rpm =
        ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
        cur_filt_engine_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;

    control_state_.filtered_engine_rpm =
        engine_rpm_median_filter.update(control_state_.filtered_engine_rpm);
    control_state_.filtered_engine_rpm =
        engine_rpm_time_filter.update(control_state_.filtered_engine_rpm);
  }

  // Gear/secondary/wheel RPM (same as original)
  float gear_rpm = 0.0f;
  float filt_gear_rpm = 0.0f;
  if (cur_gear_time_diff_us != 0.0f) {
    gear_rpm = GEAR_SAMPLE_WINDOW / GEAR_COUNTS_PER_ROT /
               cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
    filt_gear_rpm = gear_rpm_time_filter.update(gear_rpm);
  }

  float wheel_rpm = gear_rpm * GEAR_TO_WHEEL_RATIO;
  control_state_.secondary_rpm = gear_rpm / GEAR_TO_SECONDARY_RATIO;
  control_state_.filtered_secondary_rpm = filt_gear_rpm / GEAR_TO_SECONDARY_RATIO;

  // (If you were sending to dash)
  // send_command(DASH_NODE_ID, 1, 0, control_state_.filtered_secondary_rpm);

  float wheel_mph = control_state_.filtered_secondary_rpm *
                    WHEEL_TO_SECONDARY_RATIO * WHEEL_MPH_PER_RPM;

  float d_secondary_rpm =
      (control_state_.filtered_secondary_rpm - last_secondary_rpm_) * (1.0f / dt_s);
  last_secondary_rpm_ = control_state_.filtered_secondary_rpm;

  // Simple velocity-mode controller (as in original debug)
  if (WHEEL_REF_ENABLED) {
    control_state_.target_rpm =
        (wheel_mph - WHEEL_REF_BREAKPOINT_LOW_MPH) * WHEEL_REF_PIECEWISE_SLOPE +
        WHEEL_REF_LOW_RPM;
    control_state_.target_rpm =
        CLAMP(control_state_.target_rpm, WHEEL_REF_LOW_RPM, WHEEL_REF_HIGH_RPM);
  } else {
    // control_state_.target_rpm = ENGINE_TARGET_RPM;
  }

  control_state_.engine_rpm_error =
      control_state_.filtered_engine_rpm - control_state_.target_rpm;

  float filtered_engine_rpm_error =
      engine_rpm_derror_filter.update(control_state_.engine_rpm_error);

  control_state_.engine_rpm_derror =
      (filtered_engine_rpm_error - last_engine_rpm_error_) * (1.0f / dt_s);
  last_engine_rpm_error_ = filtered_engine_rpm_error;

  control_state_.velocity_mode = true;
  control_state_.velocity_command =
        (control_state_.engine_rpm_error * ACTUATOR_KP) +
        MAX(0.0f, control_state_.engine_rpm_derror * ACTUATOR_KD);

  actuator.set_velocity(control_state_.velocity_command);

  if (using_ecenterlock) {
    // If you had left-wheel sensor faulty you used right only; keep it the same
    float left_front_wheel_rpm  = 0.0f; // set if you had it previously
    float right_front_wheel_rpm = 0.0f;
    if (rw_cur_gear_time_diff_us != 0.0f) {
      right_front_wheel_rpm = R_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT /
                              rw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
    }
    ecenterlock.control_update(gear_rpm, right_front_wheel_rpm, left_front_wheel_rpm);
  }

  // Populate telemetry from ODrive
  control_state_.inbound_limit_switch  = actuator.get_inbound_limit();
  control_state_.outbound_limit_switch = actuator.get_outbound_limit();
  control_state_.engage_limit_switch   = actuator.get_engage_limit();

  control_state_.last_heartbeat_ms = odrive.get_time_since_heartbeat_ms();
  control_state_.disarm_reason     = odrive.get_disarm_reason();
  control_state_.active_errors     = odrive.get_active_errors();
  control_state_.procedure_result  = odrive.get_procedure_result();

  control_state_.bus_current    = odrive.get_bus_current();
  control_state_.bus_voltage    = odrive.get_bus_voltage();
  control_state_.iq_measured    = odrive.get_iq_measured();
  control_state_.iq_setpoint    = odrive.get_iq_setpoint();
  control_state_.velocity_estimate  = odrive.get_vel_estimate();
  control_state_.position_estimate  = odrive.get_pos_estimate();

  control_state_.p_term = ACTUATOR_KP;
  control_state_.d_term = ACTUATOR_KD;

  control_state_.total_charge_used = odrive.get_total_charge_used();
  control_state_.total_power_used  = odrive.get_total_power_used();

  // Logging (unchanged)
  if (sd_initialized && !logging_disconnected) {
    size_t message_length = encode_pb_message(
        message_buffer, MESSAGE_BUFFER_SIZE, PROTO_CONTROL_FUNCTION_MESSAGE_ID,
        ControlFunctionState_fields, &control_state_);
    u8 write_status = write_to_double_buffer(
        message_buffer, message_length, double_buffer, &cur_buffer_num, false);
    if (write_status != 0) {
      Serial.printf("Error: Failed to write to double buffer with error %d\n",
                    write_status);
    }
  } else {
    // If you drove LEDs on SD fault in original, keep it there (logger.cpp)
  }

  if (serial_logging) {
    // Example LED handling you had in debug:
    if (control_state_.outbound_limit_switch == LOW) digitalWrite(LED_1_PIN, LOW);
    if (control_state_.inbound_limit_switch  == LOW) digitalWrite(LED_3_PIN, LOW);
    if (control_state_.engage_limit_switch   == LOW) digitalWrite(LED_2_PIN, LOW);
  }

  control_state_.cycle_count++;
}

void CvtController::updateNormal() {
  odrive.request_nonstand_charge_used();
  odrive.request_nonstand_power_used();
  control_state_ = ControlFunctionState_init_default;
  control_state_.cycle_start_us = micros();
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;

  control_state_.raw_throttle = analogRead(THROTTLE_SENSOR_PIN);
  control_state_.raw_brake = analogRead(BRAKE_SENSOR_PIN);

  control_state_.throttle =
      map_int_to_float(control_state_.raw_throttle, THROTTLE_MIN_VALUE,
                       THROTTLE_MAX_VALUE, 0.0, 1.0);
  control_state_.throttle = CLAMP(control_state_.throttle, 0.0, 1.0);

  control_state_.brake = map_int_to_float(
      control_state_.raw_brake, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE, 0.0, 1.0);
  control_state_.brake = CLAMP(control_state_.brake, 0.0, 1.0);

  control_state_.throttle_filtered =
      throttle_filter.update(control_state_.throttle);

  control_state_.d_throttle =
      (control_state_.throttle_filtered - last_throttle_) / dt_s;
  last_throttle_ = control_state_.throttle_filtered;

  // Grab sensor data
  noInterrupts();
  control_state_.engine_count = engine_sensor.get_count();
  control_state_.gear_count = gear_sensor.get_count();
  control_state_.lw_gear_count = lw_gear_sensor.get_count();
  control_state_.rw_gear_count = rw_gear_sensor.get_count();

  float cur_engine_time_diff_us = engine_sensor.get_time_diff();
  float cur_filt_engine_time_diff_us = engine_sensor.get_filtered_time_diff();
  float cur_gear_time_diff_us = gear_sensor.get_time_diff();
  float lw_cur_gear_time_diff_us = lw_gear_sensor.get_time_diff(); 
  float rw_cur_gear_time_diff_us = rw_gear_sensor.get_time_diff(); 
  interrupts();

  // Calculate instantaneous RPMs
  // TODO: Fix edge case of no movement
  control_state_.engine_rpm = 0;
  if (cur_engine_time_diff_us != 0) {
    control_state_.engine_rpm = ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
                               cur_engine_time_diff_us * US_PER_SECOND *
                               SECONDS_PER_MINUTE;
    control_state_.filtered_engine_rpm =
        ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
        cur_filt_engine_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;

    // TODO: Confirm we need median filter
    control_state_.filtered_engine_rpm =
        engine_rpm_median_filter.update(control_state_.filtered_engine_rpm);
    control_state_.filtered_engine_rpm =
        engine_rpm_time_filter.update(control_state_.filtered_engine_rpm);
  }

  float gear_rpm = 0.0;
  float filt_gear_rpm = 0.0;
  if (cur_gear_time_diff_us != 0) {
    gear_rpm = GEAR_SAMPLE_WINDOW / GEAR_COUNTS_PER_ROT /
               cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
    filt_gear_rpm = gear_rpm_time_filter.update(gear_rpm);
  }

  float wheel_rpm = gear_rpm * GEAR_TO_WHEEL_RATIO;
  control_state_.secondary_rpm = gear_rpm / GEAR_TO_SECONDARY_RATIO;
  control_state_.filtered_secondary_rpm =
      filt_gear_rpm / GEAR_TO_SECONDARY_RATIO;
::send_command(static_cast<u32>(DASH_NODE_ID), 1u, false,
               control_state_.filtered_secondary_rpm);  float wheel_mph = control_state_.filtered_secondary_rpm *
                    WHEEL_TO_SECONDARY_RATIO * WHEEL_MPH_PER_RPM;

  float d_secondary_rpm = (control_state_.filtered_secondary_rpm - last_secondary_rpm_) / dt_s;
  last_secondary_rpm_ = control_state_.filtered_secondary_rpm;

  float left_front_wheel_rpm = 0.0;  
  if (lw_cur_gear_time_diff_us != 0) {
    left_front_wheel_rpm = L_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT / 
                          lw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
  }
  control_state_.left_front_wheel_rpm = left_front_wheel_rpm;

  float right_front_wheel_rpm = 0.0; 
  float filt_rfw_rpm = 0.0; 
  if (rw_cur_gear_time_diff_us != 0) {
    right_front_wheel_rpm = R_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT / 
                            rw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
  }
  control_state_.right_front_wheel_rpm = right_front_wheel_rpm;

   if(digitalRead(LIMIT_SWITCH_IN_PIN) == LOW) {
    digitalWrite(LED_3_PIN, HIGH); 
  } else {
    digitalWrite(LED_3_PIN, LOW); 
  }


  // Controller (Velocity)
  if (WHEEL_REF_ENABLED) {
    control_state_.target_rpm =
        (wheel_mph - WHEEL_REF_BREAKPOINT_LOW_MPH) * WHEEL_REF_PIECEWISE_SLOPE +
        WHEEL_REF_LOW_RPM;
    control_state_.target_rpm =
        CLAMP(control_state_.target_rpm, WHEEL_REF_LOW_RPM, WHEEL_REF_HIGH_RPM);
  } else {
    //control_state.target_rpm = ENGINE_TARGET_RPM;
  }

  control_state_.engine_rpm_error =
      control_state_.filtered_engine_rpm - control_state_.target_rpm;

  float filtered_engine_rpm_error =
      engine_rpm_derror_filter.update(control_state_.engine_rpm_error);

  control_state_.engine_rpm_derror =
      (filtered_engine_rpm_error - last_engine_rpm_error_) / dt_s;
  last_engine_rpm_error_ = filtered_engine_rpm_error;

  control_state_.velocity_mode = true;

  control_state_.velocity_command =
       (control_state_.engine_rpm_error * ACTUATOR_KP +
      MAX(0, control_state_.engine_rpm_derror * ACTUATOR_KD));

  // TODO: Move this logic to actuator ?
  /*
  if (odrive.get_pos_estimate() < ACTUATOR_SLOW_INBOUND_REGION_ROT) {
    control_state.velocity_command =
        CLAMP(control_state.velocity_command, -ODRIVE_VEL_LIMIT,
              ACTUATOR_SLOW_INBOUND_VEL);
  } else {
    control_state.velocity_command =
        CLAMP(control_state.velocity_command, -ODRIVE_VEL_LIMIT,
              ACTUATOR_FAST_INBOUND_VEL);
  }
  */
 
  actuator.set_velocity(control_state_.velocity_command);

  if (control_cycle_count_ % 20 == 0) {
   // Serial.printf("Inbound %d, Engage %d, Outbound %d \n", actuator.get_inbound_limit(), actuator.get_engage_limit(), actuator.get_outbound_limit());
  }

  // Ecenterlock Control Function 
  if (using_ecenterlock) {
    ecenterlock.control_update(gear_rpm, right_front_wheel_rpm, left_front_wheel_rpm); 
  }

  // Populate control state
  control_state_.inbound_limit_switch = actuator.get_inbound_limit();
  control_state_.outbound_limit_switch = actuator.get_outbound_limit();
  control_state_.engage_limit_switch = actuator.get_engage_limit();

  control_state_.last_heartbeat_ms = odrive.get_time_since_heartbeat_ms();
  control_state_.disarm_reason = odrive.get_disarm_reason();
  control_state_.active_errors = odrive.get_active_errors();
  control_state_.procedure_result = odrive.get_procedure_result();

  control_state_.bus_current = odrive.get_bus_current();
  control_state_.bus_voltage = odrive.get_bus_voltage();
  control_state_.iq_measured = odrive.get_iq_measured();
  control_state_.iq_setpoint = odrive.get_iq_setpoint();

  control_state_.velocity_estimate = odrive.get_vel_estimate();
  control_state_.position_estimate = odrive.get_pos_estimate();

  control_state_.p_term = ACTUATOR_KP;
  control_state_.d_term = ACTUATOR_KD;

  control_state_.total_charge_used = odrive.get_total_charge_used();
  control_state_.total_power_used = odrive.get_total_power_used();

  if (sd_initialized && !logging_disconnected) {
    // Serialize control state
    size_t message_length = encode_pb_message(
        message_buffer, MESSAGE_BUFFER_SIZE, PROTO_CONTROL_FUNCTION_MESSAGE_ID,
        &ControlFunctionState_msg, &control_state_);

    // Write to double buffer
    u8 write_status = write_to_double_buffer(
        message_buffer, message_length, double_buffer, &cur_buffer_num, false);

    if (write_status != 0) {
      Serial.printf("Error: Failed to write to double buffer with error %d\n",
                    write_status);
    }
  }

    if (serial_logging) {
    //Serial.printf("Throt Pot: %f, Secondary: %d\n", control_state.throttle, control_state.gear_count);
    if (control_state_.outbound_limit_switch == LOW) digitalWrite(LED_1_PIN, LOW);
    if (control_state_.inbound_limit_switch == LOW) digitalWrite(LED_3_PIN, LOW);
    if (control_state_.engage_limit_switch == LOW) digitalWrite(LED_2_PIN, LOW);
  }

  control_state_.cycle_count++;
}

uint32_t CvtController::cycleCount() {
  return instance_ ? instance_->control_cycle_count_ : 0u;
}
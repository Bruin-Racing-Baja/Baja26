#include <control/cvt_controller.h>
#include <hardware/actuator.h>
#include <hardware/odrive.h>
#include <hardware/ecenterlock.h>
#include <hardware/can_bus.h>
#include <hardware/sensors.h>
#include <logging/logger.h>
#include <constants.h>
#include <macros.h>
#include <iirfilter.h>
#include <median_filter.h>

// External references to global objects from main.cpp
extern ODrive odrive;
extern ODrive ecenterlock_odrive;
extern Actuator actuator;
extern Ecenterlock ecenterlock;
extern bool using_ecenterlock;
extern bool sd_initialized;
extern bool serial_logging;

extern Sensor engine_sensor;
extern Sensor gear_sensor;
extern Sensor lw_gear_sensor;
extern Sensor rw_gear_sensor;

extern IIRFilter engine_rpm_time_filter;
extern IIRFilter engine_rpm_derror_filter;
extern IIRFilter gear_rpm_time_filter;
extern IIRFilter throttle_fitler;
extern MedianFilter engine_rpm_median_filter;

extern bool last_button_state[5];

// Control state variables
ControlFunctionState control_state = ControlFunctionState_init_default;
u32 control_cycle_count = 0;

float last_throttle = 0.0;
float last_engine_rpm_error = 0;
float last_secondary_rpm = 0;

// à¶ž
void control_function() {
  odrive.request_nonstand_charge_used();
  odrive.request_nonstand_power_used();
  control_state = ControlFunctionState_init_default;
  control_state.cycle_start_us = micros();
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;

  control_state.raw_throttle = analogRead(THROTTLE_SENSOR_PIN);
  control_state.raw_brake = analogRead(BRAKE_SENSOR_PIN);

  control_state.throttle =
      map_int_to_float(control_state.raw_throttle, THROTTLE_MIN_VALUE,
                       THROTTLE_MAX_VALUE, 0.0, 1.0);
  control_state.throttle = CLAMP(control_state.throttle, 0.0, 1.0);

  control_state.brake = map_int_to_float(
      control_state.raw_brake, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE, 0.0, 1.0);
  control_state.brake = CLAMP(control_state.brake, 0.0, 1.0);

  control_state.throttle_filtered =
      throttle_fitler.update(control_state.throttle);

  control_state.d_throttle =
      (control_state.throttle_filtered - last_throttle) / dt_s;
  last_throttle = control_state.throttle_filtered;

  // Grab sensor data
  noInterrupts();
  control_state.engine_count = engine_sensor.get_count();
  control_state.gear_count = gear_sensor.get_count();
  control_state.lw_gear_count = lw_gear_sensor.get_count();
  control_state.rw_gear_count = rw_gear_sensor.get_count();

  float cur_engine_time_diff_us = engine_sensor.get_time_diff();
  float cur_filt_engine_time_diff_us = engine_sensor.get_filtered_time_diff();
  float cur_gear_time_diff_us = gear_sensor.get_time_diff();
  float lw_cur_gear_time_diff_us = lw_gear_sensor.get_time_diff(); 
  float rw_cur_gear_time_diff_us = rw_gear_sensor.get_time_diff(); 
  interrupts();

  // Calculate instantaneous RPMs
  // TODO: Fix edge case of no movement
  control_state.engine_rpm = 0;
  if (cur_engine_time_diff_us != 0) {
    control_state.engine_rpm = ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
                               cur_engine_time_diff_us * US_PER_SECOND *
                               SECONDS_PER_MINUTE;
    control_state.filtered_engine_rpm =
        ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
        cur_filt_engine_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;

    // TODO: Confirm we need median filter
    control_state.filtered_engine_rpm =
        engine_rpm_median_filter.update(control_state.filtered_engine_rpm);
    control_state.filtered_engine_rpm =
        engine_rpm_time_filter.update(control_state.filtered_engine_rpm);
  }

  float gear_rpm = 0.0;
  float filt_gear_rpm = 0.0;
  if (cur_gear_time_diff_us != 0) {
    gear_rpm = GEAR_SAMPLE_WINDOW / GEAR_COUNTS_PER_ROT /
               cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
    filt_gear_rpm = gear_rpm_time_filter.update(gear_rpm);
  }

  float wheel_rpm = gear_rpm * GEAR_TO_WHEEL_RATIO;
  control_state.secondary_rpm = gear_rpm / GEAR_TO_SECONDARY_RATIO;
  control_state.filtered_secondary_rpm =
      filt_gear_rpm / GEAR_TO_SECONDARY_RATIO;
  send_command(DASH_NODE_ID, 1, 0, control_state.filtered_secondary_rpm);
  float wheel_mph = control_state.filtered_secondary_rpm *
                    WHEEL_TO_SECONDARY_RATIO * WHEEL_MPH_PER_RPM;

  float d_secondary_rpm = (control_state.filtered_secondary_rpm - last_secondary_rpm) / dt_s;
  last_secondary_rpm = control_state.filtered_secondary_rpm;

  float left_front_wheel_rpm = 0.0;  
  if (lw_cur_gear_time_diff_us != 0) {
    left_front_wheel_rpm = L_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT / 
                          lw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
  }
  control_state.left_front_wheel_rpm = left_front_wheel_rpm;

  float right_front_wheel_rpm = 0.0; 
  float filt_rfw_rpm = 0.0; 
  if (rw_cur_gear_time_diff_us != 0) {
    right_front_wheel_rpm = R_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT / 
                            rw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
  }
  control_state.right_front_wheel_rpm = right_front_wheel_rpm;

   if(digitalRead(LIMIT_SWITCH_IN_PIN) == LOW) {
    digitalWrite(LED_3_PIN, HIGH); 
  } else {
    digitalWrite(LED_3_PIN, LOW); 
  }


  // Controller (Velocity)
  if (WHEEL_REF_ENABLED) {
    control_state.target_rpm =
        (wheel_mph - WHEEL_REF_BREAKPOINT_LOW_MPH) * WHEEL_REF_PIECEWISE_SLOPE +
        WHEEL_REF_LOW_RPM;
    control_state.target_rpm =
        CLAMP(control_state.target_rpm, WHEEL_REF_LOW_RPM, WHEEL_REF_HIGH_RPM);
  } else {
    //control_state.target_rpm = ENGINE_TARGET_RPM;
  }
  
  control_state.engine_rpm_error =
      control_state.filtered_engine_rpm - control_state.target_rpm;

  float filtered_engine_rpm_error =
      engine_rpm_derror_filter.update(control_state.engine_rpm_error);

  control_state.engine_rpm_derror =
      (filtered_engine_rpm_error - last_engine_rpm_error) / dt_s;
  last_engine_rpm_error = filtered_engine_rpm_error;

  control_state.velocity_mode = true;

  control_state.velocity_command =
       (control_state.engine_rpm_error * ACTUATOR_KP +
      MAX(0, control_state.engine_rpm_derror * ACTUATOR_KD));

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
 
  actuator.set_velocity(control_state.velocity_command);

  if (control_cycle_count % 20 == 0) {
   // Serial.printf("Inbound %d, Engage %d, Outbound %d \n", actuator.get_inbound_limit(), actuator.get_engage_limit(), actuator.get_outbound_limit());
  }

  // Ecenterlock Control Function 
  if (using_ecenterlock) {
    ecenterlock.control_update(gear_rpm, right_front_wheel_rpm, left_front_wheel_rpm); 
  }

  // Populate control state
  control_state.inbound_limit_switch = actuator.get_inbound_limit();
  control_state.outbound_limit_switch = actuator.get_outbound_limit();
  control_state.engage_limit_switch = actuator.get_engage_limit();

  control_state.last_heartbeat_ms = odrive.get_time_since_heartbeat_ms();
  control_state.disarm_reason = odrive.get_disarm_reason();
  control_state.active_errors = odrive.get_active_errors();
  control_state.procedure_result = odrive.get_procedure_result();

  control_state.bus_current = odrive.get_bus_current();
  control_state.bus_voltage = odrive.get_bus_voltage();
  control_state.iq_measured = odrive.get_iq_measured();
  control_state.iq_setpoint = odrive.get_iq_setpoint();

  control_state.velocity_estimate = odrive.get_vel_estimate();
  control_state.position_estimate = odrive.get_pos_estimate();

  control_state.p_term = ACTUATOR_KP;
  control_state.d_term = ACTUATOR_KD;

  control_state.total_charge_used = odrive.get_total_charge_used();
  control_state.total_power_used = odrive.get_total_power_used();

  if (sd_initialized && !logging_disconnected) {
    // Serialize control state
    size_t message_length = encode_pb_message(
        message_buffer, MESSAGE_BUFFER_SIZE, PROTO_CONTROL_FUNCTION_MESSAGE_ID,
        &ControlFunctionState_msg, &control_state);

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
    if (control_state.outbound_limit_switch == LOW) digitalWrite(LED_1_PIN, LOW); 
    if (control_state.inbound_limit_switch == LOW) digitalWrite(LED_3_PIN, LOW); 
    if (control_state.engage_limit_switch == LOW) digitalWrite(LED_2_PIN, LOW); 
  }

  control_state.cycle_count++;
}

void button_shift_mode() {
  bool button_pressed[5] = {false, false, false, false, false};
  for (size_t i = 0; i < 5; i++) {
    button_pressed[i] = !digitalRead(BUTTON_PINS[i]) && last_button_state[i];
  }
  for (size_t i = 0; i < 5; i++) {
    last_button_state[i] = digitalRead(BUTTON_PINS[i]);
  }

  Serial.printf("State: %d, Velocity: %f, Out: %d, Engage: %d, In: %d,\n",
                odrive.get_axis_state(), odrive.get_vel_estimate(),
                actuator.get_outbound_limit(), actuator.get_engage_limit(),
                actuator.get_inbound_limit());

  float velocity = 50.0;
  if (button_pressed[0]) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  } else if (button_pressed[1]) {
    odrive.set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  } else if (button_pressed[2]) {
    actuator.set_velocity(-velocity);
  } else if (button_pressed[3]) {
    actuator.set_velocity(0.0);
  } else if (button_pressed[4]) {
    actuator.set_velocity(velocity);
  }

  control_cycle_count++;
}

void debug_mode() {
  float dt_s = CONTROL_FUNCTION_INTERVAL_MS * SECONDS_PER_MS;
  control_cycle_count++;

  //if (control_cycle_count % 10 == 0)
  //  Serial.printf("Engage: %d, Disengage: %d\n", digitalRead(ECENTERLOCK_SWITCH_ENGAGE), digitalRead(ECENTERLOCK_SWITCH_DISENGAGE)); 

  // if(digitalRead(LIMIT_SWITCH_OUT_PIN) == LOW) {
  //   digitalWrite(LED_5_PIN, HIGH); 
  // } else {
  //   digitalWrite(LED_5_PIN, LOW); 
  // }

  // if(digitalRead(LIMIT_SWITCH_ENGAGE_PIN) == LOW) {
  //   digitalWrite(LED_4_PIN, HIGH); 
  // } else {
  //   digitalWrite(LED_4_PIN, LOW); 
  // }

  if(digitalRead(LIMIT_SWITCH_IN_PIN) == LOW) {
    digitalWrite(LED_3_PIN, HIGH); 
  } else {
    digitalWrite(LED_3_PIN, LOW); 
  }

  control_state = ControlFunctionState_init_default;
  control_state.cycle_start_us = micros();

  control_state.raw_throttle = analogRead(THROTTLE_SENSOR_PIN);
  control_state.raw_brake = analogRead(BRAKE_SENSOR_PIN);

  control_state.throttle =
      map_int_to_float(control_state.raw_throttle, THROTTLE_MIN_VALUE,
                       THROTTLE_MAX_VALUE, 0.0, 1.0);
  control_state.throttle = CLAMP(control_state.throttle, 0.0, 1.0);

  control_state.brake = map_int_to_float(
      control_state.raw_brake, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE, 0.0, 1.0);
  control_state.brake = CLAMP(control_state.brake, 0.0, 1.0);

  control_state.throttle_filtered =
      throttle_fitler.update(control_state.throttle);

  control_state.d_throttle =
      (control_state.throttle_filtered - last_throttle) / dt_s;
  last_throttle = control_state.throttle_filtered;

  // Grab sensor data
  noInterrupts();
  control_state.engine_count = engine_sensor.get_count();
  control_state.gear_count = gear_sensor.get_count();
  control_state.lw_gear_count = lw_gear_sensor.get_count();
  control_state.rw_gear_count = rw_gear_sensor.get_count();

  float cur_engine_time_diff_us = engine_sensor.get_time_diff();
  float cur_filt_engine_time_diff_us = engine_sensor.get_filtered_time_diff();
  float cur_gear_time_diff_us = gear_sensor.get_time_diff();
  float lw_cur_gear_time_diff_us = lw_gear_sensor.get_time_diff(); 
  float rw_cur_gear_time_diff_us = rw_gear_sensor.get_time_diff(); 
  interrupts();

    // Populate control state
  control_state.inbound_limit_switch = actuator.get_inbound_limit();
  control_state.outbound_limit_switch = actuator.get_outbound_limit();
  control_state.engage_limit_switch = actuator.get_engage_limit();

  control_state.last_heartbeat_ms = odrive.get_time_since_heartbeat_ms();
  control_state.disarm_reason = odrive.get_disarm_reason();
  control_state.active_errors = odrive.get_active_errors();
  control_state.procedure_result = odrive.get_procedure_result();

  control_state.bus_current = odrive.get_bus_current();
  control_state.bus_voltage = odrive.get_bus_voltage();
  control_state.iq_measured = odrive.get_iq_measured();
  control_state.iq_setpoint = odrive.get_iq_setpoint();

  control_state.velocity_estimate = odrive.get_vel_estimate();
  control_state.position_estimate = odrive.get_pos_estimate();

  control_state.p_term = ACTUATOR_KP;
   control_state.d_term = ACTUATOR_KD;

  if (sd_initialized && !logging_disconnected) {
    // Serialize control state
    size_t message_length = encode_pb_message(
        message_buffer, MESSAGE_BUFFER_SIZE, PROTO_CONTROL_FUNCTION_MESSAGE_ID,
        &ControlFunctionState_msg, &control_state);

    // Write to double buffer
    u8 write_status = write_to_double_buffer(
        message_buffer, message_length, double_buffer, &cur_buffer_num, false);

    if (write_status != 0) {
      Serial.printf("Error: Failed to write to double buffer with error %d\n",
                    write_status);
    }
  }

  // Grab sensor data
  noInterrupts();
  control_state.engine_count = engine_sensor.get_count();
  control_state.gear_count = gear_sensor.get_count();
  control_state.lw_gear_count = lw_gear_sensor.get_count();
  control_state.rw_gear_count = rw_gear_sensor.get_count();
  interrupts();

  control_state.engine_rpm = 0;
  if (cur_engine_time_diff_us != 0) {
    control_state.engine_rpm = ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
                               cur_engine_time_diff_us * US_PER_SECOND *
                               SECONDS_PER_MINUTE;
    control_state.filtered_engine_rpm =
        ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
        cur_filt_engine_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;

    // TODO: Confirm we need median filter
    control_state.filtered_engine_rpm =
        engine_rpm_median_filter.update(control_state.filtered_engine_rpm);
    control_state.filtered_engine_rpm =
        engine_rpm_time_filter.update(control_state.filtered_engine_rpm);
  }

    control_state.engine_rpm = 0;
  if (cur_engine_time_diff_us != 0) {
    control_state.engine_rpm = ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
                               cur_engine_time_diff_us * US_PER_SECOND *
                               SECONDS_PER_MINUTE;
    control_state.filtered_engine_rpm =
        ENGINE_SAMPLE_WINDOW / ENGINE_COUNTS_PER_ROT /
        cur_filt_engine_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;

    // TODO: Confirm we need median filter
    control_state.filtered_engine_rpm =
        engine_rpm_median_filter.update(control_state.filtered_engine_rpm);
    control_state.filtered_engine_rpm =
        engine_rpm_time_filter.update(control_state.filtered_engine_rpm);
  }

  float gear_rpm = 0.0;
  float filt_gear_rpm = 0.0;
  if (cur_gear_time_diff_us != 0) {
    gear_rpm = GEAR_SAMPLE_WINDOW / GEAR_COUNTS_PER_ROT /
               cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
    filt_gear_rpm = gear_rpm_time_filter.update(gear_rpm);
  }

  float wheel_rpm = gear_rpm * GEAR_TO_WHEEL_RATIO;
  control_state.secondary_rpm = gear_rpm / GEAR_TO_SECONDARY_RATIO;
  control_state.filtered_secondary_rpm =
      filt_gear_rpm / GEAR_TO_SECONDARY_RATIO;

  float wheel_mph = control_state.filtered_secondary_rpm *
                    WHEEL_TO_SECONDARY_RATIO * WHEEL_MPH_PER_RPM;

  if (control_cycle_count % 20 == 0) {
    Serial.printf("Inbound %d, Engage %d, Outbound %d \n", actuator.get_inbound_limit(), actuator.get_engage_limit(), actuator.get_outbound_limit());
    //Serial.printf("Wheel MPH, RPM: %f, %f\n", wheel_mph, wheel_rpm);
  }
  //Serial.printf("Engine Count %d\n", engine_count);
  // Serial.printf("Gear Count %d\n", gear_count);

}
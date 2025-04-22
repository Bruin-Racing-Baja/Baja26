#include "core_pins.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <actuator.h>
#include <ecenterlock.h>
#include <constants.h>
#include <iirfilter.h>
// clang-format off
#include <SPI.h>
// clang-format on
#include <HardwareSerial.h>
#include <SD.h>
#include <TimeLib.h>
#include <control_function_state.pb.h>
#include <cstring>
#include <macros.h>
#include <median_filter.h>
#include <odrive.h>
#include <operation_header.pb.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <stddef.h>
#include <stdio.h>
#include <types.h>

// Acknowledgements to Tyler, Drew, Getty, et al. :)

enum class OperatingMode {
  NORMAL,
  BUTTON_SHIFT,
  DEBUG,
  NONE,
};

/**** Operation Flags ****/
constexpr OperatingMode operating_mode = OperatingMode::NORMAL; 
constexpr bool wait_for_serial = false;
constexpr bool wait_for_can = true;
constexpr bool using_ecenterlock = true; 

/**** Global Objects ****/
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;

ODrive odrive(&flexcan_bus, ODRIVE_NODE_ID);
ODrive ecenterlock_odrive(&flexcan_bus, ECENTERLOCK_ODRIVE_NODE_ID);

Actuator actuator(&odrive);
Ecenterlock ecenterlock(&ecenterlock_odrive);

File log_file;
IIRFilter engine_rpm_rotation_filter(ENGINE_RPM_ROTATION_FILTER_B,
                                     ENGINE_RPM_ROTATION_FILTER_A,
                                     ENGINE_RPM_ROTATION_FILTER_M,
                                     ENGINE_RPM_ROTATION_FILTER_N);

IIRFilter engine_rpm_time_filter(ENGINE_RPM_TIME_FILTER_B,
                                 ENGINE_RPM_TIME_FILTER_A,
                                 ENGINE_RPM_TIME_FILTER_M,
                                 ENGINE_RPM_TIME_FILTER_N);

IIRFilter engine_rpm_derror_filter(ENGINE_RPM_DERROR_FILTER_B,
                                   ENGINE_RPM_DERROR_FILTER_A,
                                   ENGINE_RPM_DERROR_FILTER_M,
                                   ENGINE_RPM_DERROR_FILTER_N);
IIRFilter gear_rpm_time_filter(GEAR_RPM_TIME_FILTER_B, GEAR_RPM_TIME_FILTER_A,
                               GEAR_RPM_TIME_FILTER_M, GEAR_RPM_TIME_FILTER_N);
IIRFilter throttle_fitler(THROTTLE_FILTER_B, THROTTLE_FILTER_A,
                          THROTTLE_FILTER_M, THROTTLE_FILTER_N);

MedianFilter engine_rpm_median_filter(ENGINE_RPM_MEDIAN_FILTER_WINDOW);

/**** Status Variables ****/
bool sd_initialized = false;

/**** ECVT State Variables ****/
// TODO: Confirm variables only accessed in timer ISR dont need to be volatile
u32 control_cycle_count = 0;

volatile u32 engine_count = 0;
volatile u32 engine_time_diff_us = 0;
volatile float filt_engine_time_diff_us = 0;
u32 last_engine_time_us = 0;
u32 last_sample_engine_time_us = 0;

volatile u32 gear_count = 0;
volatile u32 gear_time_diff_us = 0;
volatile float last_gear_time_diff_us = 0;
u32 last_gear_time_us = 0;
u32 last_sample_gear_time_us = 0;

volatile u32 lw_gear_count = 0;
volatile u32 lw_gear_time_diff_us = 0;
volatile float lw_last_gear_time_diff_us = 0;
u32 lw_last_gear_time_us = 0;
u32 lw_last_sample_gear_time_us = 0;

volatile u32 rw_gear_count = 0;
volatile u32 rw_gear_time_diff_us = 0;
volatile float rw_last_gear_time_diff_us = 0;
u32 rw_last_gear_time_us = 0;
u32 rw_last_sample_gear_time_us = 0;

float last_throttle = 0.0;

float last_engine_rpm_error = 0;

ControlFunctionState control_state = ControlFunctionState_init_default;

/**** System State Variables ****/
bool last_button_state[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};

/**** Logging Variables ****/
volatile bool logging_disconnected = false;
struct LogBuffer {
  char buffer[LOG_BUFFER_SIZE];
  size_t idx;
  bool full;
};

// TODO: Can these be non-volatile assuming no overlap between
// ISR and loop (should be no overlap if buffer is implemented correctly)
u8 cur_buffer_num = 0;
LogBuffer double_buffer[2];

u8 message_buffer[MESSAGE_BUFFER_SIZE];

/**** Global Functions ****/
time_t get_teensy3_time() { return Teensy3Clock.get(); }

void can_parse(const CAN_message_t &msg) { odrive.parse_message(msg); }

inline void write_all_leds(u8 state) {
  digitalWrite(LED_1_PIN, state);
  digitalWrite(LED_2_PIN, state);
  digitalWrite(LED_3_PIN, state);
  digitalWrite(LED_4_PIN, state);
  digitalWrite(LED_5_PIN, state);
}

size_t encode_pb_message(u8 buffer[], size_t buffer_length, u8 id,
                         const pb_msgdesc_t *fields,
                         const void *message_struct) {
  // Serialize message
  pb_ostream_t ostream = pb_ostream_from_buffer(
      buffer + PROTO_DELIMITER_LENGTH, buffer_length - PROTO_DELIMITER_LENGTH);
  pb_encode(&ostream, fields, message_struct);

  size_t message_length = ostream.bytes_written;

  // Create message delimiter
  char delimiter[PROTO_DELIMITER_LENGTH + 1];
  snprintf(delimiter, PROTO_DELIMITER_LENGTH + 1, "%01X%04X", id,
           message_length);
  memcpy(buffer, delimiter, PROTO_DELIMITER_LENGTH);
  message_length += PROTO_DELIMITER_LENGTH;

  return message_length;
}

constexpr u8 DOUBLE_BUFFER_SUCCESS = 0;
constexpr u8 DOUBLE_BUFFER_FULL_ERROR = 1;
constexpr u8 DOUBLE_BUFFER_INDEX_ERROR = 2;

u8 write_to_double_buffer(u8 data[], size_t data_length,
                          LogBuffer double_buffer[2], u8 *buffer_num,
                          bool split) {
  LogBuffer *cur_buffer = &double_buffer[*buffer_num];

  if (cur_buffer->full) {
    // If current buffer is full then something is wrong
    return DOUBLE_BUFFER_FULL_ERROR;
  } else if (cur_buffer->idx + data_length > LOG_BUFFER_SIZE) {
    // If data_length exceeds remaining space in buffer

    size_t remaining_space = 0;
    if (split) {
      // Split data across the two buffers
      remaining_space = LOG_BUFFER_SIZE - cur_buffer->idx;
      memcpy(cur_buffer->buffer + cur_buffer->idx, data, remaining_space);
      cur_buffer->idx = LOG_BUFFER_SIZE;
    }

    // Switch to the other buffer
    cur_buffer->full = true;

    *buffer_num = !(*buffer_num);
    cur_buffer = &double_buffer[*buffer_num];

    if (cur_buffer->idx != 0) {
      // If new buffer doesn't start at the beginning then something is wrong
      return DOUBLE_BUFFER_INDEX_ERROR;
    } else {
      // Write data to new buffer
      memcpy(cur_buffer->buffer, data + remaining_space,
             data_length - remaining_space);
      cur_buffer->idx += data_length;
    }
  } else {
    // If data fits in current buffer then write it
    memcpy(cur_buffer->buffer + cur_buffer->idx, data, data_length);
    cur_buffer->idx += data_length;
  }

  return DOUBLE_BUFFER_SUCCESS;
}

// TODO: Fix filtered engine spikes
void on_engine_sensor() {
  u32 cur_time_us = micros();
  if (cur_time_us - last_engine_time_us > ENGINE_COUNT_MINIMUM_TIME_US) {
    if (engine_count % ENGINE_SAMPLE_WINDOW == 0) {
      engine_time_diff_us = cur_time_us - last_sample_engine_time_us;
      if (engine_time_diff_us > 12000) {
        filt_engine_time_diff_us = engine_time_diff_us;
      } else {
        filt_engine_time_diff_us =
            engine_rpm_rotation_filter.update(engine_time_diff_us);
      }

      last_sample_engine_time_us = cur_time_us;
    }
    ++engine_count;
  }
  last_engine_time_us = cur_time_us;
}

void on_geartooth_sensor() {
  u32 cur_time_us = micros();
  if (cur_time_us - last_gear_time_us > GEAR_COUNT_MINIMUM_TIME_US) {
    if (gear_count % GEAR_SAMPLE_WINDOW == 0) {
      gear_time_diff_us = cur_time_us - last_sample_gear_time_us;

      last_sample_gear_time_us = cur_time_us;
    }
    ++gear_count;
  }
  last_gear_time_diff_us = gear_time_diff_us;
  last_gear_time_us = cur_time_us;
}

void on_lw_geartooth_sensor() {
  u32 lw_cur_time_us = micros();
  if (lw_cur_time_us - lw_last_gear_time_us > WHEEL_GEAR_COUNT_MINIMUM_TIME_US) {
    if (gear_count % L_WHEEL_GEAR_SAMPLE_WINDOW == 0) {
      lw_gear_time_diff_us = lw_cur_time_us - lw_last_sample_gear_time_us;

      lw_last_sample_gear_time_us = lw_cur_time_us;
    }
    ++lw_gear_count;
  }
  lw_last_gear_time_diff_us = lw_gear_time_diff_us;
  lw_last_gear_time_us = lw_cur_time_us;
}

void on_rw_geartooth_sensor() {
  u32 rw_cur_time_us = micros();
  if (rw_cur_time_us - rw_last_gear_time_us > WHEEL_GEAR_COUNT_MINIMUM_TIME_US) {
    if (rw_gear_count % R_WHEEL_GEAR_SAMPLE_WINDOW == 0) {
      rw_gear_time_diff_us = rw_cur_time_us - rw_last_sample_gear_time_us;

      rw_last_sample_gear_time_us = rw_cur_time_us;
    }
    ++rw_gear_count;
  }
  rw_last_gear_time_diff_us = rw_gear_time_diff_us;
  rw_last_gear_time_us = rw_cur_time_us;
}

void on_outbound_limit_switch() {
  odrive.set_absolute_position(0.0);
  odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
}

void on_engage_limit_switch() {
  // TODO: Implement better slowdown
  float vel_estimate = odrive.get_vel_estimate();
  if (vel_estimate < -10) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  }
}

void on_inbound_limit_switch() {
  odrive.set_absolute_position(15.0);
  odrive.set_axis_state(ODrive::AXIS_STATE_IDLE); 
}

// Engaged/engaging when sensor is HIGH
void on_ecenterlock_sensor() {
  if (digitalRead(ECENTERLOCK_SENSOR_PIN) == LOW) {
    Serial.printf("Sensor LOW\n");
    //TODO: Add state variable to handle this 
  } else {
    Serial.printf("Sensor HIGH\n");
  }
    //TODO: Add state variable to handle this 
}

void on_ecenterlock_switch_engage() {
  if(ecenterlock.get_state() == Ecenterlock::DISENGAGED_2WD) {
    ecenterlock.set_engage(true);
    ecenterlock.change_state(Ecenterlock::WANT_ENGAGE); 
  }
}

//TODO: Implement this, need to allocate another pin for this :) 
void on_ecenterlock_switch_disengage() { 
  if(ecenterlock.get_state() == Ecenterlock::ENGAGED_4WD) {
    ecenterlock.set_disengage(true); 
    ecenterlock.change_state(Ecenterlock::WANT_DISENGAGE);
  }
}

inline void ecenterlock_control_function(float gear_rpm, float left_wheel_rpm, float right_wheel_rpm) {
  int cycles_to_wait_for_vel = 10; 

  float avg_front_rpm = ((left_wheel_rpm + right_wheel_rpm) / 2);

  ecenterlock_odrive.request_nonstand_pos_rel(); 
  ecenterlock.set_prev_position(ecenterlock.get_position()); 

  // TODO: Check if should add of subtract offset 
  float ecenterlock_position = ecenterlock_odrive.get_pos_rel() - ecenterlock.get_offset(); 
  ecenterlock.set_position(ecenterlock_position); 


  // State Machine for ECenterlock
  switch(ecenterlock.get_state()) {
    case Ecenterlock::UNHOMED:
      Serial.printf("Ecenterlock State: Unhomed\n");
      noInterrupts(); 
      break;
  
    case Ecenterlock::DISENGAGED_2WD: 
      break; 
    
    case Ecenterlock::WANT_ENGAGE: 

      // Pre-Engage Safety Checks! 
       
      //TODO: what's a good threshold here 
      if (gear_rpm == 0) {
        //Case 1: Car is Stopped
        ecenterlock.set_num_tries(3); 
      } else if (gear_rpm - avg_front_rpm > ECENTERLOCK_ALLOWABLE_SHIFTING_DIFFERENCE) {
        // Case 2: FW and BW Speed Difference 
        ecenterlock.set_num_tries(0); 
        Serial.printf("Ecenterlock will NOT shift: unsafe wheel speed difference"); 
        ecenterlock.change_state(Ecenterlock::DISENGAGED_2WD); 
      } else {
        //Case 3: Car is moving normally
        ecenterlock.set_num_tries(5); 
        // TODO: This isn't giving failsafe defaults 
      }

      if (ecenterlock.get_num_tries() > 0) {
        ecenterlock.set_velocity(ECENTERLOCK_VELOCITY);
        cycles_to_wait_for_vel = 10; 
        // TODO: Change to State
        ecenterlock.change_state(Ecenterlock::WAITING_FOR_VEL);
      }
      break; 

    case Ecenterlock::ENGAGED_4WD: 
      break; 

    case Ecenterlock::WANT_DISENGAGE: 
      ecenterlock.set_velocity(-ECENTERLOCK_VELOCITY);
      ecenterlock.change_state(Ecenterlock::DISENGAGING);
      break;
    
    case Ecenterlock::WAITING_FOR_VEL: 
      if (cycles_to_wait_for_vel == 0) {
        ecenterlock.change_state(Ecenterlock::ENGAGING); 
      } else {
        cycles_to_wait_for_vel--; 
      }
      break; 
    
    case Ecenterlock::ENGAGING:

        Serial.printf("Ecenterlock State: Engaging!, %f, %f\n", ecenterlock.get_position(), ecenterlock.get_prev_position()); 
        // TODO: Should this be checking position
        if (ecenterlock.get_position() == ecenterlock.get_prev_position()) {
          
          // Stopped b/c Engaged 
          // TODO: Do we want to have that clearance difference there just in case some slipping happens? 
          // TODO: Add sensor here? 
          if (ecenterlock.get_position() >= ECENTERLOCK_ENGAGED_POSITION - 0.05) {
            Serial.printf("Ecenterlock Engaged %f\n", ecenterlock.get_position()); 
            ecenterlock.set_velocity(0); 
            ecenterlock.change_state(Ecenterlock::ENGAGED_4WD); 
          } else { //Stopped in Edge Case 
            ecenterlock.set_velocity(-ECENTERLOCK_VELOCITY); 
            ecenterlock.change_state(Ecenterlock::DISENGAGING); 
            
            u8 tries_left = ecenterlock.get_num_tries() - 1; 
            if (tries_left) {
              ecenterlock.set_engage(true); 
              ecenterlock.set_num_tries(tries_left); 
            }    
          }
        }

    case Ecenterlock::DISENGAGING: 
      // Serial.printf("Ecenterlock State: Disengaging\n"); 
      // TODO: Idk if we need all these checks, a bit redundant 
      if (ecenterlock.get_position() == ecenterlock.get_prev_position() && ecenterlock.get_position() <= 0.05 && !digitalRead(ECENTERLOCK_SENSOR_PIN)) {
        Serial.printf("Ecenterlock Disengaged! %f n", ecenterlock.get_position()); 
        ecenterlock.set_velocity(0); 
        ecenterlock.change_state(Ecenterlock::DISENGAGED_2WD); 
      }
  }
     
  control_cycle_count++;
}

// ඞ
void control_function() {
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
  control_state.engine_count = engine_count;
  control_state.gear_count = gear_count;
  control_state.lw_gear_count = lw_gear_count;
  control_state.rw_gear_count = rw_gear_count;

  float cur_engine_time_diff_us = engine_time_diff_us;
  float cur_filt_engine_time_diff_us = filt_engine_time_diff_us;
  float cur_gear_time_diff_us = gear_time_diff_us;
  float lw_cur_gear_time_diff_us = lw_gear_time_diff_us; 
  float rw_cur_gear_time_diff_us = rw_gear_time_diff_us; 
  interrupts();

  // Calculate instantaneous RPMs
  // TODO: Fix edge case of no movement
  control_state.engine_rpm = 0;
  if (engine_time_diff_us != 0) {
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
  if (gear_time_diff_us != 0) {
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

  float left_front_wheel_rpm = 0.0;  
  if (lw_gear_time_diff_us != 0) {
    left_front_wheel_rpm = L_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT / 
                          lw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
  }
  control_state.left_front_wheel_rpm = left_front_wheel_rpm;

  float right_front_wheel_rpm = 0.0; 
  float filt_rfw_rpm = 0.0; 
  if (rw_gear_time_diff_us != 0) {
    right_front_wheel_rpm = R_WHEEL_GEAR_SAMPLE_WINDOW / WHEEL_GEAR_COUNTS_PER_ROT / 
                          rw_cur_gear_time_diff_us * US_PER_SECOND * SECONDS_PER_MINUTE;
  }
  control_state.right_front_wheel_rpm = right_front_wheel_rpm;

  // Controller
  control_state.target_rpm =
      (wheel_mph - WHEEL_REF_BREAKPOINT_LOW_MPH) * WHEEL_REF_PIECEWISE_SLOPE +
      WHEEL_REF_LOW_RPM;
  control_state.target_rpm =
      CLAMP(control_state.target_rpm, WHEEL_REF_LOW_RPM, WHEEL_REF_HIGH_RPM);


  // **** NOTE **** negative because of polarity of ODrive (shifting in is negative, shifting out is positive)
  // TODO: Make sign a global variable
  control_state.engine_rpm_error =
      (control_state.target_rpm - control_state.filtered_engine_rpm);

  float filtered_engine_rpm_error =
      engine_rpm_derror_filter.update(control_state.engine_rpm_error);

  control_state.engine_rpm_derror =
      (filtered_engine_rpm_error - last_engine_rpm_error) / dt_s;
  last_engine_rpm_error = filtered_engine_rpm_error;


  control_state.actuator_offset = ((wheel_mph - WHEEL_REF_BREAKPOINT_LOW_MPH) * ACTUATOR_OFFSET_SLOPE) + ACTUATOR_OFFSET_LOW;
  control_state.actuator_offset = CLAMP(control_state.actuator_offset, ACTUATOR_OFFSET_HIGH, ACTUATOR_OFFSET_LOW);

  control_state.engine_rpm_error_integral += control_state.engine_rpm_error * dt_s;


  // TODO: Handle integral-windup; tune and remove magic numbers
  // TODO: Make sign a global variable
  control_state.engine_rpm_error_integral = CLAMP(control_state.engine_rpm_error_integral, -ERROR_INTEGRAL_LIMIT_VALUE, ERROR_INTEGRAL_LIMIT_VALUE);
  if(fabs(control_state.engine_rpm_error) > 500 || wheel_mph > 3){
    control_state.engine_rpm_error_integral = 0;
  }

  // **** NOTE **** Actuator offset is negative because of polarity of ODrive (shifting in is negative, shifting out is positive)
  //
  control_state.pi_position_command = control_state.engine_rpm_error * ACTUATOR_KP + control_state.engine_rpm_error_integral * ACTUATOR_KI;
  control_state.position_command = control_state.actuator_offset  + control_state.pi_position_command;
  control_state.position_command = CLAMP(control_state.position_command, ACTUATOR_MIN_POS, ACTUATOR_MAX_POS);
  /*
  // to not interfere with starting the car 
  if (control_state.engine_rpm < 500) {
    control_state.position_command = 0; 
  }
  */
  actuator.set_position(control_state.position_command);
  if (control_state.cycle_count % 200 == 0) {
    Serial.printf("Offset %f, Ref %f, PID %f, Pos Com %f \n", actuator_offset, wheel_mph, pid_value, control_state.engine_rpm_error); 
  }
  
  // Ecenterlock Control Function 
  if (using_ecenterlock) {
    ecenterlock_control_function(gear_rpm, right_front_wheel_rpm, left_front_wheel_rpm); 
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
  //control_state.d_term = ACTUATOR_KD;

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

  float velocity = 10.0;
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

  // if(digitalRead(LIMIT_SWITCH_IN_PIN) == LOW) {
  //   digitalWrite(LED_3_PIN, HIGH); 
  // } else {
  //   digitalWrite(LED_3_PIN, LOW); 
  // }

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
  control_state.engine_count = engine_count;
  control_state.gear_count = gear_count;
  control_state.lw_gear_count = lw_gear_count;
  control_state.rw_gear_count = rw_gear_count;

  float cur_engine_time_diff_us = engine_time_diff_us;
  float cur_filt_engine_time_diff_us = filt_engine_time_diff_us;
  float cur_gear_time_diff_us = gear_time_diff_us;
  float lw_cur_gear_time_diff_us = lw_gear_time_diff_us; 
  float rw_cur_gear_time_diff_us = rw_gear_time_diff_us; 
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
  // control_state.d_term = ACTUATOR_KD;

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
  control_state.engine_count = engine_count;
  control_state.gear_count = gear_count;
  interrupts();

  control_state.engine_rpm = 0;
  if (engine_time_diff_us != 0) {
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
  if (engine_time_diff_us != 0) {
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
  if (gear_time_diff_us != 0) {
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
    Serial.printf("Engine RPM: %f\n", control_state.engine_rpm); 
    Serial.printf("Wheel MPH, RPM: %f, %f\n", wheel_mph, wheel_rpm);
  }
  //Serial.printf("Engine Count %d\n", engine_count);
  // Serial.printf("Gear Count %d\n", gear_count);

}

void setup() {
  // Pin setup
  for (u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
    pinMode(pin, OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_4_PIN, OUTPUT);
  pinMode(LED_5_PIN, OUTPUT);

  for (size_t i = 0; i < sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]); i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  pinMode(ENGINE_SENSOR_PIN, INPUT);
  pinMode(GEARTOOTH_SENSOR_PIN, INPUT);

  pinMode(THROTTLE_SENSOR_PIN, INPUT);
  pinMode(BRAKE_SENSOR_PIN, INPUT);

  pinMode(LIMIT_SWITCH_IN_PIN, INPUT);
  pinMode(LIMIT_SWITCH_OUT_PIN, INPUT);
  pinMode(LIMIT_SWITCH_ENGAGE_PIN, INPUT);

  // Status LED
  digitalWrite(LED_BUILTIN, HIGH);

  // Wait for serial if enabled
  if (wait_for_serial) {
    u32 led_flash_time_ms = 500;
    while (!Serial) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
    }
  }
  write_all_leds(LOW);

  // Setup RTC
  setSyncProvider(get_teensy3_time);

  bool rtc_set = timeStatus() == timeSet && year() > 2021;
  if (!rtc_set) {
    Serial.println("Warning: Failed to sync time with RTC");
  }

  // SD initialization
  sd_initialized = SD.sdfs.begin(SdioConfig(DMA_SDIO));
  if (!sd_initialized) {
    Serial.println("Warning: SD failed to initialize");
  } else {
    char log_name[64];
    u16 log_name_length = 0;

    if (rtc_set) {
      log_name_length = snprintf(
          log_name, sizeof(log_name), "log_%04d-%02d-%02d_%02d-%02d-%02d.bin",
          year(), month(), day(), hour(), minute(), second());
    } else {
      strncpy(log_name, "log_unknown_time.bin", sizeof(log_name));
      log_name_length = 20;
    }

    if (SD.exists(log_name)) {
      char log_name_duplicate[64];
      for (int log_num = 0; log_num < 1000; log_num++) {
        snprintf(log_name_duplicate, sizeof(log_name_duplicate),
                 "%.*s_%03d.bin", log_name_length - 4, log_name, log_num);
        if (!SD.exists(log_name_duplicate)) {
          break;
        }
      }
      strncpy(log_name, log_name_duplicate, sizeof(log_name));
      log_name[sizeof(log_name) - 1] = '\0';
    }
    Serial.printf("Info: Logging to %s\n", log_name);
    log_file = SD.open(log_name, FILE_WRITE);
    if (!log_file) {
      Serial.println("Warning: Log file was not opened! (Sarah)");
    }
  }

  // Attach sensor interrupts
  attachInterrupt(ENGINE_SENSOR_PIN, on_engine_sensor, FALLING);
  attachInterrupt(GEARTOOTH_SENSOR_PIN, on_geartooth_sensor, FALLING);

  // Attach limit switch interrupts
  attachInterrupt(LIMIT_SWITCH_OUT_PIN, on_outbound_limit_switch, FALLING);
  attachInterrupt(LIMIT_SWITCH_ENGAGE_PIN, on_engage_limit_switch, FALLING);
  attachInterrupt(LIMIT_SWITCH_IN_PIN, on_inbound_limit_switch, FALLING);

  // Initialize CAN bus
  flexcan_bus.begin();
  flexcan_bus.setBaudRate(FLEXCAN_BAUD_RATE);
  flexcan_bus.setMaxMB(FLEXCAN_MAX_MAILBOX);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableFIFOInterrupt();
  flexcan_bus.onReceive(can_parse);

  // Wait for ODrive can connection if enabled
  if (wait_for_can) {
    u32 led_flash_time_ms = 100;
    while (odrive.get_time_since_heartbeat_ms() > 100) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
      delay(100);
    }
  }
  write_all_leds(LOW);

  // Initialize subsystems
  u8 odrive_status_code = odrive.init();
  if (odrive_status_code != 0) {
    Serial.printf("Error: ODrive failed to initialize with error %d\n",
                  odrive_status_code);
  }

  u8 actuator_status_code = actuator.init();
  if (actuator_status_code != 0) {
    Serial.printf("Error: Actuator failed to initialize with error %d\n",
                  actuator_status_code);
  }

  // TODO: Why do we need delay?
  delay(3000);

  // Run actuator homing sequence
  digitalWrite(LED_2_PIN, HIGH); 
  u8 actuator_home_status = actuator.home_encoder(ACTUATOR_HOME_TIMEOUT_MS);
  if (actuator_home_status != 0) {
    Serial.printf("Error: Actuator failed to home with error %d\n", actuator_home_status);
  } else {
    digitalWrite(LED_2_PIN, LOW);
  }

  // Run ecenterlock homing sequence
  if (using_ecenterlock) {
    digitalWrite(LED_3_PIN, HIGH);
    u8 ecenterlock_home_status = ecenterlock.home(ACTUATOR_HOME_TIMEOUT_MS);
    if (ecenterlock_home_status != 0) {
      Serial.printf("Error: Ecenterlock failed to home with error %d\n", ecenterlock_home_status); 
    } else {
      digitalWrite(LED_3_PIN, LOW); 
    }
  }
  
  // Set interrupt priorities
  // TODO: Figure out proper ISR priority levels
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
  timer.priority(255);

  OperationHeader operation_header;

  operation_header.timestamp = now();
  operation_header.clock_us = micros();
  operation_header.controller_kp = ACTUATOR_KP;
  operation_header.wheel_ref_low_rpm = WHEEL_REF_LOW_RPM;
  operation_header.wheel_ref_high_rpm = WHEEL_REF_HIGH_RPM;
  operation_header.wheel_ref_breakpoint_low_mph = WHEEL_REF_BREAKPOINT_LOW_MPH;
  operation_header.wheel_ref_breakpoint_high_mph =
      WHEEL_REF_BREAKPOINT_HIGH_MPH;

  size_t message_length = encode_pb_message(
      message_buffer, MESSAGE_BUFFER_SIZE, PROTO_HEADER_MESSAGE_ID,
      &OperationHeader_msg, &operation_header);
  size_t num_bytes_written = log_file.write(message_buffer, message_length);
  log_file.flush();

  // Attach timer interrupt
  switch (operating_mode) {
  case OperatingMode::NORMAL:
    timer.begin(control_function, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::BUTTON_SHIFT:
    timer.begin(button_shift_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::DEBUG:
    timer.begin(debug_mode, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
    break;
  case OperatingMode::NONE:
    break;
  }
}

void loop() {
  // LED indicators
  // digitalWrite(LED_4_PIN, actuator.get_outbound_limit());
  // digitalWrite(LED_5_PIN, actuator.get_inbound_limit());

  // Flush SD card if buffer full
  if (sd_initialized && !logging_disconnected) {
    for (size_t buffer_num = 0; buffer_num < 2; buffer_num++) {
      if (double_buffer[buffer_num].full) {
        Serial.printf("Info: Writing buffer %d to SD\n", buffer_num);
        size_t num_bytes_written = log_file.write(
            double_buffer[buffer_num].buffer, double_buffer[buffer_num].idx);
        if (num_bytes_written == 0) {
          logging_disconnected = true;
          digitalWrite(LED_1_PIN, HIGH);
        } else {
          log_file.flush();
          double_buffer[buffer_num].full = false;
          double_buffer[buffer_num].idx = 0;
        }
      }
    }
  } else {
    digitalWrite(LED_1_PIN, HIGH);
  }
}

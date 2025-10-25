#include <hardware/ecenterlock.h>
#include <constants.h>
#include <macros.h>
#include <hardware/odrive.h>
#include <types.h>

// External reference for cycle count
extern u32 control_cycle_count;

/**
 * Constructor for the ecenterlock
 * @param odrive Pointer to ODrive object
 */
Ecenterlock::Ecenterlock(ODrive *odrive) 
    : odrive(odrive), 
      current_state(UNHOMED), 
      engage(false), 
      disengage(false), 
      num_tries(0), 
      cycles_since_stopped(0),
      cycles_to_wait_for_vel(20) {}

/**
 * Instructs ODrive to attempt ECenterlock homing
 * @return 0 if successful
 */
u8 Ecenterlock::home(u32 timeout_ms) {
  float start_time = millis(); 
  Serial.printf("Entering Homing Sequence\n"); 

  if (odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL) != 0) {
    return HOME_CAN_ERROR;
  }

  set_velocity(ECENTERLOCK_HOME_VEL);
  float cur_pos = 0; 

  while ((millis() - start_time) < timeout_ms) {
    odrive->request_nonstand_pos_rel(); 
    set_velocity(ECENTERLOCK_HOME_VEL); 
    delay(100); 

    cur_pos = odrive->get_pos_rel();
    Serial.printf("%f\n", cur_pos);
  }

  set_velocity(0); 
  position = 0; 
  pos_rel_offset = cur_pos; 
  odrive->set_absolute_position(0); 

  // if (odrive->set_axis_state(ODrive::AXIS_STATE_IDLE) != 0) {
  //   return HOME_CAN_ERROR;
  // }

  change_state(DISENGAGED_2WD); 
  Serial.printf("ECenterlock Homed with Offset %f\n", pos_rel_offset); 
  odrive->set_axis_state(ODrive::AXIS_STATE_IDLE);
  return HOME_SUCCESS; 
}

u8 Ecenterlock::set_velocity(float velocity) {
  if (odrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_VELOCITY_CONTROL,
                                  ODrive::INPUT_MODE_VEL_RAMP) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  velocity = CLAMP(velocity, -ODRIVE_VEL_LIMIT, ODRIVE_VEL_LIMIT);
  if (odrive->set_input_vel(velocity, 0) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  velocity_mode = true;

  return SET_VELOCITY_SUCCCESS;
}

u8 Ecenterlock::set_torque(float torque) {
  if (odrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_TORQUE_CONTROL, ODrive::INPUT_MODE_TORQUE_RAMP) != 0) {
    return SET_TORQUE_CAN_ERROR; 
  }

  torque = CLAMP(torque, -ODRIVE_TORQUE_LIMIT, ODRIVE_TORQUE_LIMIT); 
  if (odrive->set_input_torque(torque) != 0) {
    return SET_TORQUE_CAN_ERROR; 
  }

  torque_mode = true; 
  return SET_TORQUE_SUCCESS; 
}

bool Ecenterlock::get_outbound_limit() {
  return !digitalRead(ECENTERLOCK_SENSOR_PIN);
}

/**
 * Control update function - implements the state machine for 2WD/4WD transitions
 * Called from the main control loop
 * @param gear_rpm Current gear RPM
 * @param left_wheel_rpm Left front wheel RPM
 * @param right_wheel_rpm Right front wheel RPM
 */
void Ecenterlock::control_update(float gear_rpm, float left_wheel_rpm, float right_wheel_rpm) {
  float avg_front_rpm = right_wheel_rpm; //TODO: if left wheel speed sensor is not working
  // GRANT: Check again
  // odrive->request_nonstand_pos_rel(); 
  set_prev_position(get_position()); 

  float ecenterlock_position = odrive->get_pos_estimate();  
  set_position(ecenterlock_position); 

  // State Machine for Centerlock
  switch(get_state()) {
    case UNHOMED:
      Serial.printf("State: UNHOMED\n");
      break;
  
    case DISENGAGED_2WD: 
      break; 

    case ENGAGED_4WD: 
      break;

    case WANT_ENGAGE: 
      // Pre-Engage Safety Checks!
      // GRANT: Fix units maybe
      if (gear_rpm < 50) { // TODO: what's a good threshold here 
        // Case 1: Car is Stopped
        set_num_tries(3);
      } //else if (gear_rpm/GEAR_TO_WHEEL_RATIO - avg_front_rpm > ECENTERLOCK_ALLOWABLE_SHIFTING_DIFFERENCE) {
        // Case 2: FW and BW Speed Difference 
        //set_num_tries(0);  
      //} 
      else {
        // Case 3: Car moving normally
        set_num_tries(5); 
      }
      
      Serial.printf("State: WANT_ENGAGE, Num Tries = %d\n", get_num_tries());

      // If we are allowed engage, start engaging
      if (get_num_tries() > 0) {
        cycles_to_wait_for_vel = ECENTERLOCK_WAIT_CYCLES; 
        change_state(PRE_ENGAGING);
      } else {
        change_state(WANT_DISENGAGE);
      }
      break; 

    case WANT_DISENGAGE: 
      Serial.printf("State: WANT_DISENGAGE\n");
      cycles_to_wait_for_vel = ECENTERLOCK_WAIT_CYCLES; 
      set_velocity(-ECENTERLOCK_VELOCITY);
      change_state(PRE_DISENGAGING);
      break;
    
    case PRE_ENGAGING:
      Serial.printf("State: PRE_ENGAGING, %f\n", get_position());
      set_velocity(ECENTERLOCK_VELOCITY);
      if (cycles_to_wait_for_vel <= 0) {
        change_state(ENGAGING);  
      } else {
        cycles_to_wait_for_vel--; 
      }
      break; 
    
    case PRE_DISENGAGING: 
      Serial.printf("State: PRE_DISENGAGING, %f\n", get_position());
      set_velocity(-ECENTERLOCK_VELOCITY);
      if (cycles_to_wait_for_vel <= 0) {
        change_state(DISENGAGING); 
      } else {
        cycles_to_wait_for_vel--;
      }
      break; 
    
    case ENGAGING:
      Serial.printf("State: ENGAGING %f\n", get_position());

      // GRANT: Change to velocity check
      // If we are engaging and the centerlock is stopped:
      if (get_position() == get_prev_position()) {
        cycles_since_stopped++; 
      } else {
        cycles_since_stopped = 0; 
      }

      if (cycles_since_stopped > 10) { 
        cycles_since_stopped = 0; 
        // TODO: Do we want to have that clearance difference there just in case some slipping happens? 
        if (get_position() <= ECENTERLOCK_ENGAGED_POSITION) { 
          // Case 1: Successfully Engaged!
          set_velocity(0); 
          odrive->set_axis_state(ODrive::AXIS_STATE_IDLE);
          Serial.printf("State: ENGAGED_4WD\n");
          change_state(ENGAGED_4WD); 
          digitalWrite(ECENTERLOCK_SWITCH_LIGHT, HIGH);
        } else {  
          // Case 2: Centerlock stopped but not fully engaged :(

          // Backup and try again
          set_velocity(-ECENTERLOCK_VELOCITY);
          u8 tries_left = get_num_tries() - 1; 
          Serial.printf("In Edge Case, %d Tries Left\n", tries_left);
          // GRANT: Change num tries directly
          if (tries_left > 0) {
            set_num_tries(tries_left); 
            cycles_to_wait_for_vel = ECENTERLOCK_WAIT_CYCLES;
            change_state(ENGAGE_STEPBACK); 
          } else {
            cycles_to_wait_for_vel = ECENTERLOCK_WAIT_CYCLES;
            change_state(PRE_DISENGAGING); 
          }  
        }
      }
      break; 
    
    case ENGAGE_STEPBACK: 
      Serial.printf("State: ENGAGE_STEPBACK, %f\n", get_position());
      if (cycles_to_wait_for_vel <= 0) {
        cycles_to_wait_for_vel = ECENTERLOCK_WAIT_CYCLES; 
        set_velocity(ECENTERLOCK_VELOCITY); 
        change_state(PRE_ENGAGING); 
      } else {
        cycles_to_wait_for_vel--; 
      }
      break;
 
    case DISENGAGING: 
      Serial.printf("State: DISENGAGING, %f\n", get_position());
      
      // GRANT: Use velocity
      if (get_position() == get_prev_position()) {
        cycles_since_stopped++; 
      } else {
        cycles_since_stopped = 0; 
      }

      if (cycles_since_stopped > 10 && get_position() > -0.5) {
        cycles_since_stopped = 0; 
        set_velocity(0); 
        odrive->set_axis_state(ODrive::AXIS_STATE_IDLE); 
        change_state(DISENGAGED_2WD); 
        Serial.printf("State: DISENGAGED_2WD\n"); 
        digitalWrite(ECENTERLOCK_SWITCH_LIGHT, LOW); 
      }
      break; 
  }
  control_cycle_count++;
}
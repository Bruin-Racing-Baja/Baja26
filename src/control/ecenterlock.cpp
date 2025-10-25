#include <ecenterlock.h>
#include <constants.h>
#include <macros.h>
#include <hardware/odrive.h>
#include <types.h>

/**
 * Constructor for the actuator
 * @param odrive Pointer to ODrive object
 */
Ecenterlock::Ecenterlock(ODrive *odrive) : odrive(odrive), current_state(UNHOMED), engage(false), disengage(false), num_tries(0), cycles_since_stopped(0) {}

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
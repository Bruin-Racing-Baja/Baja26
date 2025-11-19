#include "constants.h"
#include <hardware/odrive.h>

ODrive::ODrive(u32 node_id)
    : node_id(node_id) {
  last_heartbeat_ms = 0;
}

/**
 * @brief Initialize the ODrive
 * @param parse Pointer to function that parses received messages
 * @return 0 if successful
 */
u8 ODrive::init(float current_softmax) {
  if (set_limits(ODRIVE_VEL_LIMIT, current_softmax) != 0) {
    return INIT_CAN_ERROR;
  }
  return INIT_SUCCESS;
}

/**
 * @brief Send a command
 * @param cmd_id ODrive CAN command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for
 * requesters)
 * @param buf 8-wide array of command data bytes
 * @return Status code of command send
 */
bool ODrive::send_command(u32 cmd_id, bool remote, u8 buf[8]) {
  // TODO: Fix error messages
  CanFrame msg = {0};

  if (cmd_id < 0x00 || 0x1f < cmd_id) {
    return 0;
  }

  msg.identifier= (node_id << 5) | cmd_id;
  msg.data_length_code = 8;
  memcpy(&msg.data, buf, 8);
  msg.rtr = remote;

  return ESP32Can.writeFrame(msg);
}

/**
 * @brief Send a command with no particular data (typically used for
 * requesters)
 * @param cmd_id Command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for
 * requesters)
 * @return Status code of command send
 */
bool ODrive::send_empty_command(u32 cmd_id, bool remote) {
  u8 buf[8] = {0};
  return ODrive::send_command(cmd_id, remote, buf);
}

/**
 * @brief Parse a CAN message and update ODrive members
 * @param msg CAN message to parse
 */
void ODrive::parse_message(const CanFrame &msg) {
  u32 parsed_node_id = (msg.identifier >> 5) & 0x3F;

  if (parsed_node_id != node_id) {
    return;
  }

  u32 cmd_id = msg.identifier & 0x1F;

  switch (cmd_id) {
  case CAN_HEARTBEAT:
    // Cyclic message; sent every 100ms
    last_heartbeat_ms = millis();
    memcpy(&axis_error, msg.data, 4);
    memcpy(&axis_state, msg.data + 4, 1);
    memcpy(&procedure_result, msg.data + 5, 1);
    memcpy(&trajectory_done_flag, msg.data + 6, 1);
    break;
  case CAN_GET_ERRORS:
    memcpy(&active_errors, msg.data, 4);
    memcpy(&disarm_reason, msg.data + 4, 4);
    break;
  case CAN_GET_ENCODER_ESTIMATES:
    memcpy(&pos_estimate, msg.data, 4);
    memcpy(&vel_estimate, msg.data + 4, 4);
    break;
  case CAN_GET_IQ:
    memcpy(&iq_setpoint, msg.data, 4);
    memcpy(&iq_measured, msg.data + 4, 4);
    break;
  case CAN_GET_BUS_VOLTAGE_CURRENT:
    memcpy(&bus_voltage, msg.data, 4);
    memcpy(&bus_current, msg.data + 4, 4);
    break;
  case CAN_TXSDO: 
    u16 endpoint_id; 
    memcpy(&endpoint_id, msg.data + 1, 2);
    if(endpoint_id == TOTAL_CHARGE_USED_ID)
      memcpy(&total_charge_used, msg.data + 4, 4); 
    else if(endpoint_id == TOTAL_POWER_USED_ID)
      memcpy(&total_power_used, msg.data + 4, 4); 
  }
}

/**
 * Requests ODrive data over the CAN bus. This occurs asynchronously through an
 * interrupt that calls ODrive.parse_message when the message is recieved,
 * meaning that class members won't be updated immediately.
 */
bool ODrive::request_nonstand_pos_rel() {
  u8 buf[8] = {0, 0xCA, 0x01, 0, 0, 0, 0, 0};
  return send_command(CAN_RXSDO, false, buf); 
}

bool ODrive::request_nonstand_charge_used(){
  u8 buf[8] = {0};
  memcpy(buf + 1, &TOTAL_CHARGE_USED_ID, 2);
  return send_command(CAN_RXSDO, false, buf); 
}
bool ODrive::request_nonstand_power_used(){
  u8 buf[8] = {0};
  memcpy(buf + 1, &TOTAL_POWER_USED_ID, 2);
  return send_command(CAN_RXSDO, false, buf); 
}

bool ODrive::request_errors() { return send_empty_command(CAN_GET_ERRORS, true); }

bool  ODrive::request_iq() { return send_empty_command(CAN_GET_IQ, true); }

bool ODrive::request_temperature() {
  return send_empty_command(CAN_GET_TEMPERATURE, true);
}

bool ODrive::request_bus_voltage_current() {
  return send_empty_command(CAN_GET_BUS_VOLTAGE_CURRENT, true);
}

/**
 * Getters for the various class ODrive class members that are updated by CAN
 * messages.
 */

u32 ODrive::get_time_since_heartbeat_ms() {
  return millis() - last_heartbeat_ms;
}

u32 ODrive::get_axis_error() { return axis_error; }

u32 ODrive::get_axis_state() { return axis_state; }

u32 ODrive::get_active_errors() { return active_errors; }

u32 ODrive::get_disarm_reason() { return disarm_reason; }

u8 ODrive::get_procedure_result() { return procedure_result; }

float ODrive::get_vel_estimate() { return vel_estimate; }

float ODrive::get_pos_estimate() { return pos_estimate; }

float ODrive::get_iq_setpoint() { return iq_setpoint; }

float ODrive::get_iq_measured() { return iq_measured; }

float ODrive::get_bus_voltage() { return bus_voltage; }

float ODrive::get_bus_current() { return bus_current; }

float ODrive::get_pos_rel() { return pos_rel; }

float ODrive::get_total_charge_used() {return total_charge_used; }

float ODrive::get_total_power_used() {return total_power_used; }

/**
 * Commands for the ODrive.
 */

bool ODrive::reboot() { return send_empty_command(CAN_REBOOT, false); }

bool ODrive::clear_errors() {
  return send_empty_command(CAN_CLEAR_ERRORS, false);
}

/**
 * Setters for the various class ODrive class members that are updated by CAN
 * messages.
 */
bool ODrive::set_axis_state(u32 requested_state) {
  u8 buf[8] = {0};
  memcpy(buf, &requested_state, 4);
  return send_command(CAN_SET_AXIS_STATE, false, buf);
}

bool ODrive::set_controller_mode(u32 control_mode, u32 input_mode) {
  u8 buf[8] = {0};
  memcpy(buf, &control_mode, 4);
  memcpy(buf + 4, &input_mode, 4);
  return send_command(CAN_SET_CONTROLLER_MODE, false, buf);
}

bool ODrive::set_input_pos(float input_pos, i16 vel_ff, i16 torque_ff) {
  u8 buf[8] = {0};
  memcpy(buf, &input_pos, 4);
  memcpy(buf + 4, &vel_ff, 2);
  memcpy(buf + 6, &torque_ff, 2);
  return send_command(CAN_SET_INPUT_POS, false, buf);
}

bool ODrive::set_input_vel(float input_vel, float torque_ff) {
  u8 buf[8] = {0};
  memcpy(buf, &input_vel, 4);
  memcpy(buf + 4, &torque_ff, 4);
  return send_command(CAN_SET_INPUT_VEL, false, buf);
}

bool ODrive::set_input_torque(float input_torque) {
  u8 buf[8] = {0};
  memcpy(buf, &input_torque, 4);
  return send_command(CAN_SET_INPUT_TORQUE, false, buf);
}

bool ODrive::set_limits(float vel_limit, float current_soft_max) {
  u8 buf[8] = {0};
  memcpy(buf, &vel_limit, 4);
  memcpy(buf + 4, &current_soft_max, 4);
  return send_command(CAN_SET_LIMITS, false, buf);
}

bool ODrive::set_traj_vel_limit(float traj_vel_limit) {
  u8 buf[8] = {0};
  memcpy(buf, &traj_vel_limit, 4);
  return send_command(CAN_SET_TRAJ_VEL_LIMIT, false, buf);
}

bool ODrive::set_traj_accel_limits(float traj_accel_limit,
                                 float traj_decel_limit) {
  u8 buf[8] = {0};
  memcpy(buf, &traj_accel_limit, 4);
  memcpy(buf + 4, &traj_decel_limit, 4);
  return send_command(CAN_SET_TRAJ_ACCEL_LIMITS, false, buf);
}

bool ODrive::set_traj_intertia(float traj_inertia) {
  u8 buf[8] = {0};
  memcpy(buf, &traj_inertia, 4);
  return send_command(CAN_SET_TRAJ_INERTIA, false, buf);
}

bool ODrive::set_absolute_position(float position) {
  u8 buf[8] = {0};
  memcpy(buf, &position, 4);
  return send_command(CAN_SET_ABSOLUTE_POSITION, false, buf);
}

bool ODrive::set_pos_gain(float pos_gain) {
  u8 buf[8] = {0};
  memcpy(buf, &pos_gain, 4);
  
  return send_command(CAN_SET_POS_GAIN, false, buf);
}

bool ODrive::set_vel_gains(float vel_gain, float vel_integrator_gain) {
  u8 buf[8] = {0};
  memcpy(buf, &vel_gain, 4);
  memcpy(buf + 4, &vel_integrator_gain, 4);
  return send_command(CAN_SET_VEL_GAINS, false, buf);
}
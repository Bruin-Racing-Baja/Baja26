#ifndef ODRIVE_H
#define ODRIVE_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <types.h>

/**
 * @brief This class provides methods to communicate with an ODrive over the CAN
 * bus.
 */
class ODrive {

public:
  static const u32 CONTROL_MODE_VOLTAGE_CONTROL = 0x0;
  static const u32 CONTROL_MODE_TORQUE_CONTROL = 0x1;
  static const u32 CONTROL_MODE_VELOCITY_CONTROL = 0x2;
  static const u32 CONTROL_MODE_POSITION_CONTROL = 0x3;

  static const u32 INPUT_MODE_INACTIVE = 0x0;
  static const u32 INPUT_MODE_PASSTHROUGH = 0x1;
  static const u32 INPUT_MODE_VEL_RAMP = 0x2;
  static const u32 INPUT_MODE_POS_FILTER = 0x3;
  static const u32 INPUT_MODE_MIX_CHANNELS = 0x4;
  static const u32 INPUT_MODE_TRAP_TRAJ = 0x5;
  static const u32 INPUT_MODE_TORQUE_RAMP = 0x6;
  static const u32 INPUT_MODE_MIRROR = 0x7;
  static const u32 INPUT_MODE_TUNING = 0x8;

  static const u32 AXIS_STATE_UNDEFINED = 0x0;
  static const u32 AXIS_STATE_IDLE = 0x1;
  static const u32 AXIS_STATE_STARTUP_SEQUENCE = 0x2;
  static const u32 AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 0x3;
  static const u32 AXIS_STATE_MOTOR_CALIBRATION = 0x4;
  static const u32 AXIS_STATE_ENCODER_INDEX_SEARCH = 0x6;
  static const u32 AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 0x7;
  static const u32 AXIS_STATE_CLOSED_LOOP_CONTROL = 0x8;
  static const u32 AXIS_STATE_LOCKIN_SPIN = 0x9;
  static const u32 AXIS_STATE_ENCODER_DIR_FIND = 0xA;
  static const u32 AXIS_STATE_HOMING = 0xB;
  static const u32 AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 0xC;
  static const u32 AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 0xD;
  static const u32 AXIS_STATE_ANTICOGGING_CALIBRATION = 0xE;

  static const u32 CAN_GET_VERSION = 0x000;
  static const u32 CAN_HEARTBEAT = 0x001;
  static const u32 CAN_ESTOP = 0x002;
  static const u32 CAN_GET_ERRORS = 0x003;
  static const u32 CAN_RXSDO = 0x004;
  static const u32 CAN_TXSDO = 0x005;
  static const u32 CAN_ADDRESS = 0x006;
  static const u32 CAN_SET_AXIS_STATE = 0x007;
  static const u32 CAN_GET_ENCODER_ESTIMATES = 0x009;
  static const u32 CAN_SET_CONTROLLER_MODE = 0x00b;
  static const u32 CAN_SET_INPUT_POS = 0x00c;
  static const u32 CAN_SET_INPUT_VEL = 0x00d;
  static const u32 CAN_SET_INPUT_TORQUE = 0x00e;
  static const u32 CAN_SET_LIMITS = 0x00f;
  static const u32 CAN_SET_TRAJ_VEL_LIMIT = 0x011;
  static const u32 CAN_SET_TRAJ_ACCEL_LIMITS = 0x012;
  static const u32 CAN_SET_TRAJ_INERTIA = 0x013;
  static const u32 CAN_GET_IQ = 0x014;
  static const u32 CAN_GET_TEMPERATURE = 0x015;
  static const u32 CAN_REBOOT = 0x016;
  static const u32 CAN_GET_BUS_VOLTAGE_CURRENT = 0x017;
  static const u32 CAN_CLEAR_ERRORS = 0x018;
  static const u32 CAN_SET_ABSOLUTE_POSITION = 0x019;
  static const u32 CAN_SET_POS_GAIN = 0x01a;
  static const u32 CAN_SET_VEL_GAINS = 0x01b;
  static const u32 CAN_GET_TORQUES = 0x01c;
  static const u32 CAN_GET_POWERS = 0x01d;
  static const u32 CAN_ENTER_DFU_MODE = 0x01f;

  static const u8 INIT_SUCCESS = 0;
  static const u8 INIT_CAN_ERROR = 1;

  static const u8 CMD_SUCCESS = 0;
  static const u8 CMD_ERROR_INVALID_AXIS = 1;
  static const u8 CMD_ERROR_INVALID_COMMAND = 2;
  static const u8 CMD_ERROR_WRITE_FAILED = 3;

  ODrive(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus, u32 node_id);

  u8 init();

  void parse_message(const CAN_message_t &msg);

  // Requesters
  u8 request_errors();
  u8 request_iq();
  u8 request_temperature();
  u8 request_bus_voltage_current();
  u8 request_nonstand_pos_rel();

  // Getters
  u32 get_time_since_heartbeat_ms();
  u32 get_axis_error();
  u32 get_axis_state();
  u32 get_active_errors();
  u32 get_disarm_reason();
  u8 get_procedure_result();
  float get_vel_estimate();
  float get_pos_estimate();
  float get_iq_setpoint();
  float get_iq_measured();
  float get_bus_voltage();
  float get_bus_current();
  float get_pos_rel(); 

  // Commands
  u8 reboot();
  u8 clear_errors();

  // Setters
  u8 set_axis_state(u32 axis_state);
  u8 set_controller_mode(u32 control_mode, u32 input_mode);
  u8 set_input_pos(float input_pos, i16 vel_ff, i16 torque_ff);
  u8 set_input_vel(float input_vel, float torque_ff);
  u8 set_input_torque(float input_torque);
  u8 set_limits(float vel_limit, float current_soft_max);
  u8 set_traj_vel_limit(float traj_vel_limit);
  u8 set_traj_accel_limits(float traj_accel_limit, float traj_decel_limit);
  u8 set_traj_intertia(float traj_inertia);
  u8 set_absolute_position(float pos_estimate);
  u8 set_pos_gain(float pos_gain);
  u8 set_vel_gains(float vel_gain, float vel_integrator_gain);

private:
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *flexcan_bus;

  u32 node_id;

  u32 last_heartbeat_ms;

  u32 axis_error;
  u32 axis_state;
  u8 procedure_result;
  u8 trajectory_done_flag;
  u32 active_errors, disarm_reason;
  float vel_estimate, pos_estimate;
  float iq_setpoint, iq_measured;
  float bus_voltage, bus_current;
  float pos_rel; 

  u8 send_command(u32 cmd_id, bool remote, u8 buf[8]);
  u8 send_empty_command(u32 cmd_id, bool remote);
};
#endif

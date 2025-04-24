#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

class Actuator {

public:
  static const u8 SET_VELOCITY_SUCCCESS = 0;
  static const u8 SET_VELOCITY_IN_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_VELOCITY_CAN_ERROR = 2;

  static const u8 SET_POSITION_SUCCCESS = 0;
  static const u8 SET_POSITION_LIMIT_SWITCH_ERROR = 1;
  static const u8 SET_POSITION_CAN_ERROR = 2;

  static const u8 HOME_SUCCCESS = 0;
  static const u8 HOME_CAN_ERROR = 1;
  static const u8 HOME_TIMEOUT_ERROR = 2;

  Actuator(ODrive *odrive);

  u8 init();
  u8 home_encoder(u32 timeout_ms);

  u8 set_velocity(float velocity);
  u8 set_position(float position, float position_estimate);

  bool get_inbound_limit();
  bool get_outbound_limit();
  bool get_engage_limit();

private:
  bool velocity_mode = true;
  ODrive *odrive;
};

#endif

#ifndef CAN_IDS_H
#define CAN_IDS_H
#include <macros.h>
#include <math.h>
#include <stddef.h>
#include <types.h>

const u32 CAN_CYCLE_COUNT = 0;
const u32 CAN_TIMESTAMP = 1;
const u32 can_cycle_start_us = 2;
const u32 CAN_ENGINE_COUNT = 3;
const u32 CAN_GEAR_COUNT = 4;
const u32 CAN_ENGINE_RPM = 5;
const u32 CAN_SECONDARY_RPM = 6;
const u32 CAN_FILTERED_ENGINE_RPM = 7;
const u32 CAN_FILTERED_SECONDARY_RPM = 8;
const u32 CAN_TARGET_RPM = 9;
const u32 CAN_ENGINE_RPM_ERROR = 10;
const u32 CAN_ECVT_VELOCITY_COMMAND = 11;
const u32 CAN_ECVT_VELOCITY_ESTIMATE = 12;
const u32 CAN_ECVT_POSITION_ESTIMATE = 13;
const u32 CAN_ECVT_LAST_HEARTBEAT_MS = 14;
const u32 CAN_ECVT_ACTIVE_ERRORS = 15;
const u32 CAN_ECVT_DISARM_REASON = 16;
const u32 CAN_ECVT_PROCEDURE_RESULT = 17;
const u32 CAN_ECVT_BUS_VOLTAGE = 18;
const u32 CAN_ECVT_BUS_CURRENT = 19;
const u32 CAN_ECVT_IQ_MEASURED = 20;
const u32 CAN_ECVT_IQ_SETPOINT = 21;
const u32 CAN_ECVT_INBOUND_LIMIT_SWITCH = 22;
const u32 CAN_ECVT_OUTBOUND_LIMIT_SWITCH = 23;
const u32 CAN_ECVT_ENGAGE_LIMIT_SWITCH = 24;
const u32 CAN_VELOCITY_MODE = 25;
const u32 can_position_command = 26;
const u32 CAN_ENGINE_RPM_DERROR = 27;
const u32 CAN_THROTTLE = 28;
const u32 CAN_THROTTLE_FILTERED = 29;
const u32 CAN_BRAKE = 30;
const u32 can_brake_filtered = 31;
const u32 CAN_ECVT_ACTUATOR_KP = 32;
const u32 CAN_ECVT_ACTUATOR_KD = 33;
const u32 can_wheel_ref_low_rpm = 34;
const u32 can_wheel_ref_high_rpm = 35;
const u32 can_wheel_ref_breakpoint_low_mph = 36;
const u32 can_wheel_ref_breakpoint_high_mph = 37;
const u32 CAN_D_THROTTLE = 38;
const u32 CAN_RAW_THROTTLE = 39;
const u32 CAN_RAW_BRAKE = 40;
const u32 CAN_LW_GEAR_COUNT = 41;
const u32 CAN_RW_GEAR_COUNT = 42;
const u32 CAN_LEFT_FRONT_WHEEL_RPM = 43;
const u32 CAN_RIGHT_FRONT_WHEEL_RPM = 44;

#endif
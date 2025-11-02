#include <hardware/can_bus.h>
#include <hardware/odrive.h>
#include <constants.h>
#include <cstring>

// External references to global objects from main.cpp
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
extern ODrive odrive;
extern ODrive ecenterlock_odrive;

void can_parse(const CAN_message_t &msg) { 
  u32 parsed_node_id = (msg.id >> 5) & 0x3F;
  if (parsed_node_id == ODRIVE_NODE_ID) {
    odrive.parse_message(msg);
  } else if (parsed_node_id == ECENTERLOCK_ODRIVE_NODE_ID) {
    ecenterlock_odrive.parse_message(msg);
  }
}

void send_command(u32 node_id, u32 cmd_id, bool remote, float val) {
  // TODO: Fix error messages
  CAN_message_t msg;
  u8 buf[8] = {0};
  if (cmd_id < 0x00 || 0x1f < cmd_id) {
    return;
  }

  msg.id = (node_id << 5) | cmd_id;
  msg.len = 4;
  memcpy(msg.buf, &val, 4);
  msg.flags.remote = remote;

  int write_code = flexcan_bus.write(msg);
}
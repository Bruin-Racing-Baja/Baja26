#include <hardware/can_bus.h>
#include <hardware/odrive.h>
#include <constants.h>
#include <cstring>

// External references to global objects from main.cpp (unchanged)
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
extern ODrive odrive;
extern ODrive ecenterlock_odrive;

void CanBus::onReceive(const CAN_message_t &msg) {
  u32 parsed_node_id = (msg.id >> 5) & 0x3F;
  if (parsed_node_id == ODRIVE_NODE_ID) {
    odrive_.parse_message(msg);
  } else if (parsed_node_id == ECENTERLOCK_ODRIVE_NODE_ID) {
    ecenterlock_odrive_.parse_message(msg);
  }
  // else ignore
}

void CanBus::sendCommand(u32 node_id, u32 cmd_id, bool remote, float val) {
  if (cmd_id > 0x1f) return; // clamp like your original

  CAN_message_t msg;
  std::memset(&msg, 0, sizeof(msg));
  msg.id = (node_id << 5) | cmd_id;
  msg.len = 4;
  std::memcpy(msg.buf, &val, 4);
  msg.flags.remote = remote;

  (void)flexcan_bus.write(msg);
}

// ---------------- Backward-compatible wrappers ----------------

void can_parse(const CAN_message_t &msg) {
  if (auto* b = CanBus::instance()) {
    b->onReceive(msg);
  } else {
    // Fallback to legacy behavior if not bound (route to globals)
    u32 parsed_node_id = (msg.id >> 5) & 0x3F;
    if (parsed_node_id == ODRIVE_NODE_ID) {
      odrive.parse_message(msg);
    } else if (parsed_node_id == ECENTERLOCK_ODRIVE_NODE_ID) {
      ecenterlock_odrive.parse_message(msg);
    }
  }
}

void send_command(u32 node_id, u32 cmd_id, bool remote, float val) {
  if (auto* b = CanBus::instance()) {
    b->sendCommand(node_id, cmd_id, remote, val);
  } else {
    // Fallback to legacy behavior
    if (cmd_id > 0x1f) return;
    CAN_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.id = (node_id << 5) | cmd_id;
    msg.len = 4;
    std::memcpy(msg.buf, &val, 4);
    msg.flags.remote = remote;
    (void)flexcan_bus.write(msg);
  }
}
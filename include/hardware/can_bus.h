#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <types.h>
#include <constants.h>

// Forward decls (your project already has these classes)
class ODrive;

// Encapsulated CAN bus router
class CanBus {
public:
  // Construct with references to things we route to / use
  CanBus(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& bus,
         ODrive& cvt, ODrive& ecenter)
    : bus_(bus), odrive_(cvt), ecenterlock_odrive_(ecenter) {}

  // Bind a global instance so the static callback can forward
  static void bind(CanBus* self) { instance_ = self; }

  // Begin + register static onReceive
  void begin(uint32_t baud = FLEXCAN_BAUD_RATE, uint8_t max_mb = FLEXCAN_MAX_MAILBOX) {
    bus_.begin();
    bus_.setBaudRate(baud);
    bus_.setMaxMB(max_mb);
    bus_.enableFIFO();
    bus_.enableFIFOInterrupt();
    bus_.onReceive(&CanBus::onReceiveStatic);
  }

  // Static trampoline for FlexCAN callback
  static void onReceiveStatic(const CAN_message_t &msg) {
    if (instance_) instance_->onReceive(msg);
  }

  // Route message to the right ODrive (same logic as before)
  void onReceive(const CAN_message_t &msg);

  // Send a float command (preserves your free function semantics)
  void sendCommand(u32 node_id, u32 cmd_id, bool remote, float val);

  // Access to underlying bus if needed
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& raw() { return bus_; }

  // Singleton-like accessor (for legacy free-function wrappers)
  static CanBus* instance() { return instance_; }

private:
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& bus_;
  ODrive& odrive_;
  ODrive& ecenterlock_odrive_;
  static inline CanBus* instance_ = nullptr;
};

// ---- Backward-compatible free functions (kept for callers) ----
void can_parse(const CAN_message_t &msg);
void send_command(u32 node_id, u32 cmd_id, bool remote, float val);

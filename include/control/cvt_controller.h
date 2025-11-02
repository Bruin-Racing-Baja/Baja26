#pragma once
#include <Arduino.h>
#include <control_function_state.pb.h>

class CvtController {
public:
  enum class Mode {
    Normal,
    ButtonShift,
    Debug
  };

  CvtController() = default;

  // Bind the global trampoline to this instance (used by IntervalTimer)
  static void bind(CvtController* self) { instance_ = self; }
  static void isrTrampoline();

  static uint32_t cycleCount(); // New: expose cycle count if needed

  // Mode control
  void setMode(Mode m) { mode_ = m; }
  Mode mode() const { return mode_; }

  // Called by isrTrampoline each control interval
  void tick();

  // Former free functions -> member methods
  void updateNormal();      // was control_function()
  void updateButtonShift(); // was button_shift_mode()
  void updateDebug();       // was debug_mode()

  // Expose state if callers need to read it (same semantics as before)
  const ControlFunctionState& state() const { return control_state_; }

private:
  // Class-scoped state that used to be global
  ControlFunctionState* control_state_ptr_ = nullptr; // optional; see .cpp notes
  ControlFunctionState* last_sent_state_ptr_ = nullptr; // optional; keep if you logged it
  ControlFunctionState control_state_{};

  uint32_t control_cycle_count_ = 0;
  float last_throttle_ = 0.0f;
  float last_engine_rpm_error_ = 0.0f;
  float last_secondary_rpm_ = 0.0f;
  bool last_button_state_[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};

  static CvtController* instance_;
  Mode mode_ = Mode::Normal;
};

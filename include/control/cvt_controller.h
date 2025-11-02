#pragma once
#include <Arduino.h>

class CvtController {
public:
  enum class Mode {
    Normal,
    ButtonShift,
    Debug
  };

  CvtController() = default;

  // Bind the global trampoline to this instance
  static void bind(CvtController* self) { instance_ = self; }

  // ISR-safe trampoline (hook this to IntervalTimer)
  static void isrTrampoline();

  // Select and read the active mode
  void setMode(Mode m) { mode_ = m; }
  Mode mode() const { return mode_; }

  // One control tick (called by isrTrampoline)
  void tick();

private:
  static CvtController* instance_;
  Mode mode_ = Mode::Normal;
};

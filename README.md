# Baja25


# use this command when cloning repo to ensure that the submodules are also added 
git clone --recurse-submodules <code link here>

using CvtController:

#include <control/cvt_controller.h>

static CvtController controller;

void setup() {
  // ...
  CvtController::bind(&controller);
  controller.setMode(CvtController::Mode::Normal);
  // timer.begin(CvtController::isrTrampoline, CONTROL_FUNCTION_INTERVAL_US);
}

When you want to switch behavior without changing the underlying functions:

controller.setMode(CvtController::Mode::ButtonShift);
// or:
controller.setMode(CvtController::Mode::Debug);
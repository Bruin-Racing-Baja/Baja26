#ifndef CVT_CONTROLLER_H
#define CVT_CONTROLLER_H

#include <control_function_state.pb.h>
#include <types.h>

// External reference for cycle count
extern ControlFunctionState control_state;
extern u32 control_cycle_count;

// Control mode functions
void control_function();
void button_shift_mode();
void debug_mode();

#endif
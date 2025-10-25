#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <types.h>
#include <iirfilter.h>
#include <median_filter.h>

// Sensor state - these will be extern, defined in sensors.cpp
extern volatile u32 engine_count;
extern volatile u32 engine_time_diff_us;
extern volatile float filt_engine_time_diff_us;
extern u32 last_engine_time_us;
extern u32 last_sample_engine_time_us;

extern volatile u32 gear_count;
extern volatile u32 gear_time_diff_us;
extern volatile float last_gear_time_diff_us;
extern u32 last_gear_time_us;
extern u32 last_sample_gear_time_us;

extern volatile u32 lw_gear_count;
extern volatile u32 lw_gear_time_diff_us;
extern volatile float lw_last_gear_time_diff_us;
extern u32 lw_last_gear_time_us;
extern u32 lw_last_sample_gear_time_us;

extern volatile u32 rw_gear_count;
extern volatile u32 rw_gear_time_diff_us;
extern volatile float rw_last_gear_time_diff_us;
extern u32 rw_last_gear_time_us;
extern u32 rw_last_sample_gear_time_us;

// Filter references - will be set by sensors_init()
extern IIRFilter* engine_rpm_rotation_filter_ptr;

// Sensor ISR functions
void on_engine_sensor();
void on_geartooth_sensor();
void on_lw_geartooth_sensor();
void on_rw_geartooth_sensor();
void on_outbound_limit_switch();
void on_engage_limit_switch();
void on_inbound_limit_switch();
void on_ecenterlock_switch_engage();
void on_ecenterlock_switch_disengage();

// Initialization function to set filter pointers
void sensors_init(IIRFilter* engine_rotation_filter);

#endif
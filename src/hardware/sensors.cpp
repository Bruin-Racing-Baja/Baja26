#include <hardware/sensors.h>
#include <hardware/odrive.h>
#include <hardware/ecenterlock.h>
#include <constants.h>

// These are the global objects from main.cpp - will be extern
extern ODrive odrive;
extern Ecenterlock ecenterlock;

// Sensor state variables - defined here, declared extern in sensors.h
volatile u32 engine_count = 0;
volatile u32 engine_time_diff_us = 0;
volatile float filt_engine_time_diff_us = 0;
u32 last_engine_time_us = 0;
u32 last_sample_engine_time_us = 0;

volatile u32 gear_count = 0;
volatile u32 gear_time_diff_us = 0;
volatile float last_gear_time_diff_us = 0;
u32 last_gear_time_us = 0;
u32 last_sample_gear_time_us = 0;

volatile u32 lw_gear_count = 0;
volatile u32 lw_gear_time_diff_us = 0;
volatile float lw_last_gear_time_diff_us = 0;
u32 lw_last_gear_time_us = 0;
u32 lw_last_sample_gear_time_us = 0;

volatile u32 rw_gear_count = 0;
volatile u32 rw_gear_time_diff_us = 0;
volatile float rw_last_gear_time_diff_us = 0;
u32 rw_last_gear_time_us = 0;
u32 rw_last_sample_gear_time_us = 0;

// Filter pointer - set by sensors_init()
IIRFilter* engine_rpm_rotation_filter_ptr = nullptr;

void sensors_init(IIRFilter* engine_rotation_filter) {
  engine_rpm_rotation_filter_ptr = engine_rotation_filter;
}

// TODO: Fix filtered engine spikes
void on_engine_sensor() {
  u32 cur_time_us = micros();
  if (cur_time_us - last_engine_time_us > ENGINE_COUNT_MINIMUM_TIME_US) {
    if (engine_count % ENGINE_SAMPLE_WINDOW == 0) {
      engine_time_diff_us = cur_time_us - last_sample_engine_time_us;
      if (engine_time_diff_us > 12000) {
        filt_engine_time_diff_us = engine_time_diff_us;
      } else {
        filt_engine_time_diff_us =
            engine_rpm_rotation_filter_ptr->update(engine_time_diff_us);
      }

      last_sample_engine_time_us = cur_time_us;
    }
    ++engine_count;
  }
  last_engine_time_us = cur_time_us;
}

void on_geartooth_sensor() {
  u32 cur_time_us = micros();
  if (cur_time_us - last_gear_time_us > GEAR_COUNT_MINIMUM_TIME_US) {
    if (gear_count % GEAR_SAMPLE_WINDOW == 0) {
      gear_time_diff_us = cur_time_us - last_sample_gear_time_us;

      last_sample_gear_time_us = cur_time_us;
    }
    ++gear_count;
  }
  last_gear_time_diff_us = gear_time_diff_us;
  last_gear_time_us = cur_time_us;
}

void on_lw_geartooth_sensor() {
  u32 lw_cur_time_us = micros();
  if (lw_cur_time_us - lw_last_gear_time_us > WHEEL_GEAR_COUNT_MINIMUM_TIME_US) {
    if (gear_count % L_WHEEL_GEAR_SAMPLE_WINDOW == 0) {
      lw_gear_time_diff_us = lw_cur_time_us - lw_last_sample_gear_time_us;

      lw_last_sample_gear_time_us = lw_cur_time_us;
    }
    ++lw_gear_count;
  }
  lw_last_gear_time_diff_us = lw_gear_time_diff_us;
  lw_last_gear_time_us = lw_cur_time_us;
}

void on_rw_geartooth_sensor() {
  u32 rw_cur_time_us = micros();
  if (rw_cur_time_us - rw_last_gear_time_us > WHEEL_GEAR_COUNT_MINIMUM_TIME_US) {
    if (rw_gear_count % R_WHEEL_GEAR_SAMPLE_WINDOW == 0) {
      rw_gear_time_diff_us = rw_cur_time_us - rw_last_sample_gear_time_us;

      rw_last_sample_gear_time_us = rw_cur_time_us;
    }
    ++rw_gear_count;
  }
  rw_last_gear_time_diff_us = rw_gear_time_diff_us;
  rw_last_gear_time_us = rw_cur_time_us;
}

void on_outbound_limit_switch() {
  odrive.set_absolute_position(0.0);
  if (odrive.get_vel_estimate() < 0) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  }
}

void on_engage_limit_switch() {
  if (odrive.get_vel_estimate() < 0) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  }
}

void on_inbound_limit_switch() {
  if (odrive.get_vel_estimate() > 0) {
    odrive.set_axis_state(ODrive::AXIS_STATE_IDLE);
  }
}

void on_ecenterlock_switch_engage() {
  Serial.print("Engage Interrupt\n");
  if(ecenterlock.get_state() == Ecenterlock::DISENGAGED_2WD) {
    ecenterlock.change_state(Ecenterlock::WANT_ENGAGE); 
  }
}

void on_ecenterlock_switch_disengage() {
  Serial.print("Engage Interrupt\n");
  if(ecenterlock.get_state() == Ecenterlock::ENGAGED_4WD) {
    ecenterlock.change_state(Ecenterlock::WANT_DISENGAGE); 
  }
}

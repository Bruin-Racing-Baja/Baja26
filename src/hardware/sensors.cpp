#include <hardware/sensors.h>
#include <hardware/odrive.h>
#include <hardware/ecenterlock.h>
#include <constants.h>

// External references to global objects from main.cpp
extern ODrive odrive;
extern Ecenterlock ecenterlock;

// ============================================================================
// Sensor Class Implementation
// ============================================================================

Sensor::Sensor(u32 min_time, u32 window, IIRFilter* filter)
    : minimum_time_us(min_time), 
      sample_window(window), 
      filter_ptr(filter),
      use_filter(filter != nullptr) {}

void Sensor::on_sensor_trigger() {
    u32 cur_time_us = micros();
    
    // Debounce check
    if (cur_time_us - last_time_us > minimum_time_us) {
        // Sample window check
        if (count % sample_window == 0) {
            time_diff_us = cur_time_us - last_sample_time_us;
            
            // Apply filter if available (engine sensor only)
            if (use_filter && filter_ptr != nullptr) {
                // For engine sensor: skip filtering if time is too large
                if (time_diff_us > 12000) {
                    filtered_time_diff_us = time_diff_us;
                } else {
                    filtered_time_diff_us = filter_ptr->update(time_diff_us);
                }
            } else {
                // No filter - just copy the value
                filtered_time_diff_us = time_diff_us;
            }
            
            last_sample_time_us = cur_time_us;
        }
        ++count;
    }
    last_time_us = cur_time_us;
}

// ============================================================================
// Global Sensor Objects - Instantiated here
// ============================================================================

// Initialize with appropriate constants from constants.h
Sensor engine_sensor(ENGINE_COUNT_MINIMUM_TIME_US, ENGINE_SAMPLE_WINDOW);
Sensor gear_sensor(GEAR_COUNT_MINIMUM_TIME_US, GEAR_SAMPLE_WINDOW);
Sensor lw_gear_sensor(WHEEL_GEAR_COUNT_MINIMUM_TIME_US, L_WHEEL_GEAR_SAMPLE_WINDOW);
Sensor rw_gear_sensor(WHEEL_GEAR_COUNT_MINIMUM_TIME_US, R_WHEEL_GEAR_SAMPLE_WINDOW);

// ============================================================================
// Sensor ISR Handlers - Simple wrappers that call the sensor objects
// ============================================================================

void on_engine_sensor() {
    engine_sensor.on_sensor_trigger();
}

void on_geartooth_sensor() {
    gear_sensor.on_sensor_trigger();
}

void on_lw_geartooth_sensor() {
    lw_gear_sensor.on_sensor_trigger();
}

void on_rw_geartooth_sensor() {
    rw_gear_sensor.on_sensor_trigger();
}

// ============================================================================
// Limit Switch ISRs - No changes from original
// ============================================================================

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

// ============================================================================
// Ecenterlock Switch ISRs 
// ============================================================================

void on_ecenterlock_switch_engage() {
    Serial.print("Engage Interrupt\n");
    if(ecenterlock.get_state() == Ecenterlock::DISENGAGED_2WD) {
        ecenterlock.change_state(Ecenterlock::WANT_ENGAGE); 
    }
}

void on_ecenterlock_switch_disengage() {
    Serial.print("Disengage Interrupt\n");
    if(ecenterlock.get_state() == Ecenterlock::ENGAGED_4WD) {
        ecenterlock.change_state(Ecenterlock::WANT_DISENGAGE); 
    }
}

// ============================================================================
// Initialization - Sets the filter for engine sensor only
// ============================================================================

void sensors_init(IIRFilter* engine_rotation_filter) {
    // Only the engine sensor uses filtering
    engine_sensor.set_filter(engine_rotation_filter);
}
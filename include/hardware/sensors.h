#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <types.h>
#include <iirfilter.h>

// Sensor class - encapsulates common sensor timing/counting logic
class Sensor {
private:
    volatile u32 count = 0;
    volatile u32 time_diff_us = 0;
    volatile float filtered_time_diff_us = 0;
    u32 last_time_us = 0;
    u32 last_sample_time_us = 0;
    
    // Configuration
    u32 minimum_time_us;
    u32 sample_window;
    IIRFilter* filter_ptr;
    bool use_filter;
    
public:
    // Constructor
    Sensor(u32 min_time, u32 window, IIRFilter* filter = nullptr);
    
    // Main ISR function - call this from interrupt handler
    void on_sensor_trigger();
    
    // Getters for sensor data
    u32 get_count() const { return count; }
    u32 get_time_diff() const { return time_diff_us; }
    float get_filtered_time_diff() const { return filtered_time_diff_us; }
    
    // Configuration
    void set_filter(IIRFilter* filter) { filter_ptr = filter; use_filter = (filter != nullptr); }
};

// Global sensor objects
extern Sensor engine_sensor;
extern Sensor gear_sensor;
extern Sensor lw_gear_sensor;
extern Sensor rw_gear_sensor;

// Sensor ISR functions - these call the sensor objects
void on_engine_sensor();
void on_geartooth_sensor();
void on_lw_geartooth_sensor();
void on_rw_geartooth_sensor();

// Limit switch ISRs
void on_outbound_limit_switch();
void on_engage_limit_switch();
void on_inbound_limit_switch();

// Ecenterlock switch ISRs
void on_ecenterlock_switch_engage();
void on_ecenterlock_switch_disengage();

// Initialization function
void sensors_init(IIRFilter* engine_rotation_filter);

#endif
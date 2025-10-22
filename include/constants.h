#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <macros.h>
#include <math.h>
#include <stddef.h>
#include <types.h>

#define dancing 13

// Units
constexpr float MINUTES_PER_HOUR = 60.0;
constexpr float SECONDS_PER_MINUTE = 60.0; // s / min
constexpr float MS_PER_SECOND = 1.0e3;     // ms / s
constexpr float US_PER_SECOND = 1.0e6;     // us / s
constexpr float SECONDS_PER_MS = 1.0e-3;   // s / ms
constexpr float SECONDS_PER_US = 1.0e-6;   // s / us

constexpr float MM_PER_INCH = 25.4;              // mm / inch
constexpr float INCHES_PER_MM = 1 / MM_PER_INCH; // inch / mm

constexpr float FEET_PER_MILE = 5280.0; // feet / mile
constexpr float INCH_PER_FEET = 12.0;   // inch / feet

constexpr float ENGINE_COUNTS_PER_ROT = 32; // count / rot
constexpr float GEAR_COUNTS_PER_ROT = 50;   // count / rot

constexpr float WHEEL_GEAR_COUNTS_PER_ROT = 12; // count / rot

// NOTE: x_to_y ratio is how many rotations of x for 1 rotation of y
constexpr float GEAR_TO_WHEEL_RATIO = 58.0 / 19.0;                
constexpr float GEAR_TO_SECONDARY_RATIO = 17.0 / 50.0;             
 
constexpr float SECONDARY_TO_WHEEL_RATIO =                    
    ((50.0 / 17.0) * (58.0 / 19.0)); // ~8.978

    // 1: 17, 2:50, 3: 19, 4: 58
constexpr float WHEEL_TO_SECONDARY_RATIO =
    (1.0 / SECONDARY_TO_WHEEL_RATIO); // ~0.111                  

constexpr float WHEEL_DIAMETER_INCH = 23.0; // inch                     
constexpr float WHEEL_MPH_PER_RPM = (WHEEL_DIAMETER_INCH * M_PI) /       
                                    (FEET_PER_MILE * INCH_PER_FEET) *    
                                    MINUTES_PER_HOUR; // mph / rpm       

constexpr u32 ENGINE_SAMPLE_WINDOW = 4;
constexpr u32 GEAR_SAMPLE_WINDOW = 10;
constexpr u32 L_WHEEL_GEAR_SAMPLE_WINDOW = 1; 
constexpr u32 R_WHEEL_GEAR_SAMPLE_WINDOW = 1;
 
// ODrive
constexpr u8 ODRIVE_NODE_ID = 0x3;         
constexpr u8 DASH_NODE_ID = 0x4;                
constexpr float ODRIVE_VEL_LIMIT = 80.0;        // rot / s    //TODO: CHange this to reasonable value
constexpr float ODRIVE_ECVT_CURRENT_SOFT_MAX = 30.0;
constexpr float ODRIVE_ECENT_CURRENT_SOFT_MAX = 20.0; // A          // TODO: Check if this is the right value 
constexpr float ODRIVE_TORQUE_LIMIT = 0.1; // Nm 

//Ecenterlock 
constexpr u8 ECENTERLOCK_ODRIVE_NODE_ID = 0x7; 

constexpr float ECENTERLOCK_PITCH_MM = 2.54; // mm / rot
constexpr float ECENTERLOCK_PITCH_CM = ECENTERLOCK_PITCH_MM / 10.0; // cm / rot

constexpr float ECENTERLOCK_ENGAGE_POS_ROT = 4.0;   // ROT? 
constexpr float ECENTERLOCK_ENGAGE_POS_CM =
    ECENTERLOCK_ENGAGE_POS_ROT * ECENTERLOCK_PITCH_CM; // cm
 
constexpr float ECENTERLOCK_ENGAGED_POSITION = -4.0; // TODO: potentially change this 
constexpr float ECENTERLOCK_VELOCITY = -6.0;     // rot / s 

constexpr float ECENTERLOCK_HOME_VEL = 6;
constexpr float ECENTERLOCK_HOME_TIMEOUT_MS = 4000;  // ms

constexpr float ECENTERLOCK_ALLOWABLE_SHIFTING_DIFFERENCE = 100; 

constexpr float ECENTERLOCK_TORQUE = 0.0899; // Nm 
constexpr u8 NUM_TRIES_ECENTERLOCK = 5;
constexpr u8 ECENTERLOCK_WAIT_CYCLES = 50;

// Driver Interface
constexpr u32 BRAKE_MIN_VALUE = 405;                // TODO: UPDATE THIS
constexpr u32 BRAKE_MAX_VALUE = 290;                // TODO: UPDATE THIS
constexpr u32 THROTTLE_MIN_VALUE = 1000;            // TODO: UPDATE THIS
constexpr u32 THROTTLE_MAX_VALUE = 80;            // TODO: UPDATE THIS

// Actuator
// NOTE: Pitch is distance / rotation
constexpr float ACTUATOR_PITCH_MM = 4.0;                      // mm / rot
constexpr float ACTUATOR_PITCH_CM = ACTUATOR_PITCH_MM / 10.0; // cm / rot

constexpr float ACTUATOR_ENGAGE_POS_ROT = 3.5;   // rot
constexpr float ACTUATOR_INBOUND_POS_ROT = 16.2; // rot
constexpr float ACTUATOR_ENGAGE_POS_CM =
    ACTUATOR_ENGAGE_POS_ROT * ACTUATOR_PITCH_CM; // cm
constexpr float ACTUATOR_INBOUND_POS_CM =
    ACTUATOR_INBOUND_POS_ROT * ACTUATOR_PITCH_CM; // cm
constexpr float ACTUATOR_HOME_VELOCITY = 4.0;     // rot / s
constexpr float ACTUATOR_HOME_TIMEOUT_MS = 4000;  // ms

constexpr float ACTUATOR_SLOW_INBOUND_REGION_ROT = 5.0;
constexpr float ACTUATOR_SLOW_INBOUND_VEL = 30.0;
constexpr float ACTUATOR_FAST_INBOUND_VEL = 60.0;

// Control Function
constexpr u32 CONTROL_FUNCTION_INTERVAL_MS = 10; // ms

// 1: Accel
// 2: Arizona Endurance

#define MODE 2

#if MODE == 1
constexpr float ACTUATOR_MAX_HARDSTOP = 0;  // hard stop from odrive tool
constexpr float ACTUATOR_MAX_POS = 0; 
constexpr float ACTUATOR_MIN_POS = -14; 
constexpr float ACTUATOR_OFFSET_LOW = 0; 
constexpr float ACTUATOR_OFFSET_HIGH = -10;
constexpr float ACTUATOR_OFFSET_BREAKPOINT_LOW_MPH = 0;
constexpr float ACTUATOR_OFFSET_BREAKPOINT_HIGH_MPH = 20.0;
constexpr float ACTUATOR_OFFSET_SLOPE = -((ACTUATOR_OFFSET_HIGH - ACTUATOR_OFFSET_LOW) / (ACTUATOR_OFFSET_BREAKPOINT_LOW_MPH - ACTUATOR_OFFSET_BREAKPOINT_HIGH_MPH));
constexpr float ACTUATOR_KP = 0.03;   
constexpr float ACTUATOR_KI = 0.000;   
constexpr float ACTUATOR_KD = 0.000;
constexpr bool WHEEL_REF_ENABLED = true;
constexpr float WHEEL_REF_LOW_RPM = 2200;
constexpr float WHEEL_REF_HIGH_RPM = 3600; 
constexpr float WHEEL_REF_BREAKPOINT_LOW_MPH = 5;
constexpr float WHEEL_REF_BREAKPOINT_HIGH_MPH = 15;
constexpr float WHEEL_REF_PIECEWISE_SLOPE =
    (WHEEL_REF_HIGH_RPM - WHEEL_REF_LOW_RPM) /
    (WHEEL_REF_BREAKPOINT_HIGH_MPH - WHEEL_REF_BREAKPOINT_LOW_MPH);
constexpr float ERROR_INTEGRAL_LIMIT_VALUE = 500.0; 
constexpr float ACTUATOR_ENGAGE_BREAKPOINT_RPM = 1000.0; 
constexpr float ACTUATOR_ENGAGE_SLOPE = ACTUATOR_MAX_POS/ACTUATOR_ENGAGE_BREAKPOINT_RPM; 
#elif MODE == 2
constexpr float ACTUATOR_MAX_HARDSTOP = 0;  // hard stop from odrive tool
constexpr float ACTUATOR_MAX_POS = 0; 
constexpr float ACTUATOR_MIN_POS = -14; 
constexpr float ACTUATOR_OFFSET_LOW = 0; 
constexpr float ACTUATOR_OFFSET_HIGH = -10;
constexpr float ACTUATOR_OFFSET_BREAKPOINT_LOW_MPH = 0;
constexpr float ACTUATOR_OFFSET_BREAKPOINT_HIGH_MPH = 20.0;
constexpr float ACTUATOR_OFFSET_SLOPE = -((ACTUATOR_OFFSET_HIGH - ACTUATOR_OFFSET_LOW) / (ACTUATOR_OFFSET_BREAKPOINT_LOW_MPH - ACTUATOR_OFFSET_BREAKPOINT_HIGH_MPH));
constexpr float ACTUATOR_KP = 0.01;
constexpr float ACTUATOR_KI = 0.000;
constexpr float ACTUATOR_KD = 0.004;
constexpr bool WHEEL_REF_ENABLED = true;
constexpr float WHEEL_REF_LOW_RPM = 2100;
constexpr float WHEEL_REF_HIGH_RPM = 3100; 
constexpr float WHEEL_REF_BREAKPOINT_LOW_MPH = 5;
constexpr float WHEEL_REF_BREAKPOINT_HIGH_MPH = 10;
constexpr float WHEEL_REF_PIECEWISE_SLOPE =
    (WHEEL_REF_HIGH_RPM - WHEEL_REF_LOW_RPM) /
    (WHEEL_REF_BREAKPOINT_HIGH_MPH - WHEEL_REF_BREAKPOINT_LOW_MPH);
constexpr float ERROR_INTEGRAL_LIMIT_VALUE = 500.0; 
constexpr float ACTUATOR_ENGAGE_BREAKPOINT_RPM = 1000.0; 
constexpr float ACTUATOR_ENGAGE_SLOPE = ACTUATOR_MAX_POS/ACTUATOR_ENGAGE_BREAKPOINT_RPM; 
#endif

constexpr u32 ENGINE_COUNT_MINIMUM_TIME_US = 100;
constexpr u32 GEAR_COUNT_MINIMUM_TIME_US = 300;

constexpr u32 WHEEL_GEAR_COUNT_MINIMUM_TIME_US = 300; //reasonable variable?

constexpr u32 ENGINE_RPM_MEDIAN_FILTER_WINDOW = 3;

constexpr float ENGINE_RPM_ROTATION_FILTER_B[] = {
    0.8677114646, -3.305398989, 4.8804516238, -3.305398989, 0.8677114646};
constexpr float ENGINE_RPM_ROTATION_FILTER_A[] = {
    1.0, -3.5518051128, 4.8720546544, -3.0589928651, 0.7438198987};
constexpr size_t ENGINE_RPM_ROTATION_FILTER_M =
    COUNT_OF(ENGINE_RPM_ROTATION_FILTER_B);
constexpr size_t ENGINE_RPM_ROTATION_FILTER_N =
    COUNT_OF(ENGINE_RPM_ROTATION_FILTER_A);

constexpr float ENGINE_RPM_TIME_FILTER_B[] = {0.24523727525278557,
                                              0.24523727525278557};
constexpr float ENGINE_RPM_TIME_FILTER_A[] = {1.0, -0.5095254494944288};
constexpr size_t ENGINE_RPM_TIME_FILTER_M = COUNT_OF(ENGINE_RPM_TIME_FILTER_B);
constexpr size_t ENGINE_RPM_TIME_FILTER_N = COUNT_OF(ENGINE_RPM_TIME_FILTER_A);

constexpr float ENGINE_RPM_DERROR_FILTER_B[] = {0.07295965726826667,
                                                0.0729596572682667};
constexpr float ENGINE_RPM_DERROR_FILTER_A[] = {1.0, -0.8540806854634666};
constexpr size_t ENGINE_RPM_DERROR_FILTER_M =
    COUNT_OF(ENGINE_RPM_DERROR_FILTER_B);
constexpr size_t ENGINE_RPM_DERROR_FILTER_N =
    COUNT_OF(ENGINE_RPM_DERROR_FILTER_A);

constexpr float GEAR_RPM_TIME_FILTER_B[] = {
    0.007820208033497193, 0.015640416066994386, 0.007820208033497193};
constexpr float GEAR_RPM_TIME_FILTER_A[] = {1.0, -1.734725768809275,
                                            0.7660066009432638};
constexpr size_t GEAR_RPM_TIME_FILTER_M = COUNT_OF(GEAR_RPM_TIME_FILTER_B);
constexpr size_t GEAR_RPM_TIME_FILTER_N = COUNT_OF(GEAR_RPM_TIME_FILTER_A);

constexpr float THROTTLE_FILTER_B[] = {0.0591907, 0.0591907};
constexpr float THROTTLE_FILTER_A[] = {1., -0.88161859};
constexpr size_t THROTTLE_FILTER_M = COUNT_OF(THROTTLE_FILTER_B);
constexpr size_t THROTTLE_FILTER_N = COUNT_OF(THROTTLE_FILTER_A);

// Teensy Pins
constexpr u8 LED_1_PIN = 7; // D0 on PCB
constexpr u8 LED_2_PIN = 8; // D1 on PCB
constexpr u8 LED_3_PIN = 9; // D2 on PCB
constexpr u8 LED_4_PIN = 28; // D3 on PCB
constexpr u8 LED_5_PIN = 29; // D4 on PCB

constexpr u8 LIMIT_SWITCH_IN_PIN = 12;
constexpr u8 LIMIT_SWITCH_OUT_PIN = 10;
constexpr u8 LIMIT_SWITCH_ENGAGE_PIN = 11;

constexpr u8 BUTTON_PINS[] = {2, 3, 4, 5, 6};

constexpr u8 ENGINE_SENSOR_PIN = 16; 
constexpr u8 GEARTOOTH_SENSOR_PIN = 15; 
constexpr u8 LEFT_WHEEL_SENSOR_PIN = 13;
constexpr u8 RIGHT_WHEEL_SENSOR_PIN = 14; 
constexpr u8 ECENTERLOCK_SENSOR_PIN = 27; 

constexpr u8 THROTTLE_SENSOR_PIN = 40;
constexpr u8 BRAKE_SENSOR_PIN = 39;

constexpr u8 ECENTERLOCK_SWITCH_ENGAGE = 38; //38
constexpr u8 ECENTERLOCK_SWITCH_DISENGAGE = 24; // 24
constexpr u8 ECENTERLOCK_SWITCH_LIGHT = 25; // TODO: Change to actual value

constexpr u32 ECENTERLOCK_HOME_TIMEOUT = 5000; 

// Flexcan
constexpr u32 FLEXCAN_BAUD_RATE = 250000;  
constexpr u32 FLEXCAN_MAX_MAILBOX = 16;     
// Logging
// bytes_per_cycle * cycle_freq * time_to_flush_sd * safety_factor
// 100 * 100 * 0.4 * 2 = 8000
constexpr size_t LOG_BUFFER_SIZE = 65536;

constexpr u8 PROTO_HEADER_MESSAGE_ID = 0x00;
constexpr u8 PROTO_CONTROL_FUNCTION_MESSAGE_ID = 0x01;

constexpr size_t MESSAGE_BUFFER_SIZE = 512;
constexpr size_t PROTO_DELIMITER_LENGTH = 5;

#endif

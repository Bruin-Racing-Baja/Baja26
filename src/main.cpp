#include "core_pins.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

#include <hardware/actuator.h>
#include <hardware/can_bus.h>
#include <hardware/ecenterlock.h>
#include <hardware/odrive.h>
#include <hardware/sensors.h>

#include <control/cvt_controller.h>
#include <logging/logger.h>
#include <constants.h>
#include <filters/iirfilter.h>
#include <control_function_state.pb.h>

// clang-format off
#include <SPI.h>
// clang-format on
#include <HardwareSerial.h>
#include <SD.h>
#include <TimeLib.h>
#include <control_function_state.pb.h>
#include <cstring>
#include <macros.h>
#include <filters/median_filter.h>
#include <operation_header.pb.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <stddef.h>
#include <stdio.h>
#include <types.h>

// Acknowledgements to Tyler, Drew, Getty, et al. :)

/*==============================================================================
|                             Operating Mode / Flags
==============================================================================*/
enum class OperatingMode {
  NORMAL,
  BUTTON_SHIFT,
  DEBUG,
  NONE,
};

constexpr OperatingMode operating_mode   = OperatingMode::NORMAL;
constexpr bool          wait_for_serial  = false;
constexpr bool          wait_for_can_ecvt = true;

bool using_ecenterlock = true;
bool serial_logging    = true;

/*==============================================================================
|                                 Globals
==============================================================================*/
// Timers / Buses
IntervalTimer timer;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;

// Devices
ODrive odrive(&flexcan_bus, ODRIVE_NODE_ID);
ODrive ecenterlock_odrive(&flexcan_bus, ECENTERLOCK_ODRIVE_NODE_ID);
Actuator actuator(&odrive);
Ecenterlock ecenterlock(&ecenterlock_odrive);
CvtController controller;

// Filters
IIRFilter engine_rpm_rotation_filter(ENGINE_RPM_ROTATION_FILTER_B,
                                     ENGINE_RPM_ROTATION_FILTER_A,
                                     ENGINE_RPM_ROTATION_FILTER_M,
                                     ENGINE_RPM_ROTATION_FILTER_N);

IIRFilter engine_rpm_time_filter(ENGINE_RPM_TIME_FILTER_B,
                                 ENGINE_RPM_TIME_FILTER_A,
                                 ENGINE_RPM_TIME_FILTER_M,
                                 ENGINE_RPM_TIME_FILTER_N);

IIRFilter engine_rpm_derror_filter(ENGINE_RPM_DERROR_FILTER_B,
                                   ENGINE_RPM_DERROR_FILTER_A,
                                   ENGINE_RPM_DERROR_FILTER_M,
                                   ENGINE_RPM_DERROR_FILTER_N);

IIRFilter gear_rpm_time_filter(GEAR_RPM_TIME_FILTER_B, GEAR_RPM_TIME_FILTER_A,
                               GEAR_RPM_TIME_FILTER_M, GEAR_RPM_TIME_FILTER_N);

IIRFilter throttle_filter(THROTTLE_FILTER_B, THROTTLE_FILTER_A,
                          THROTTLE_FILTER_M, THROTTLE_FILTER_N);

MedianFilter engine_rpm_median_filter(ENGINE_RPM_MEDIAN_FILTER_WINDOW);

// System status
bool sd_initialized = false;

// UI state
bool last_button_state[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};

// CAN owner
static CanBus CAN(flexcan_bus, odrive, ecenterlock_odrive);

// Logger singleton
static Logger& logger = Logger::instance();

/*==============================================================================
|                         Helpers / Free Functions
==============================================================================*/
time_t get_teensy3_time() { return Teensy3Clock.get(); }

inline void write_all_leds(u8 state) {
  digitalWrite(LED_1_PIN, state);
  digitalWrite(LED_2_PIN, state);
  digitalWrite(LED_3_PIN, state);
  digitalWrite(LED_4_PIN, state);
  digitalWrite(LED_5_PIN, state);
}

/*==============================================================================
|                                   setup()
==============================================================================*/
void setup() {
  /* --------------------------- Pin configuration --------------------------- */
  for (u8 pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
    pinMode(pin, OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_4_PIN, OUTPUT);
  pinMode(LED_5_PIN, OUTPUT);

  pinMode(ECENTERLOCK_SWITCH_LIGHT, OUTPUT);

  for (size_t i = 0; i < sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]); i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  pinMode(ECENTERLOCK_SWITCH_ENGAGE, INPUT_PULLUP);
  pinMode(ECENTERLOCK_SWITCH_DISENGAGE, INPUT_PULLUP);

  pinMode(ENGINE_SENSOR_PIN, INPUT);
  pinMode(GEARTOOTH_SENSOR_PIN, INPUT);

  pinMode(THROTTLE_SENSOR_PIN, INPUT);
  pinMode(BRAKE_SENSOR_PIN, INPUT);

  pinMode(LIMIT_SWITCH_IN_PIN, INPUT);
  pinMode(LIMIT_SWITCH_OUT_PIN, INPUT);
  pinMode(LIMIT_SWITCH_ENGAGE_PIN, INPUT);

  // Status LED
  digitalWrite(LED_BUILTIN, HIGH);

  /* ------------------------------- Serial wait ----------------------------- */
  if (wait_for_serial) {
    u32 led_flash_time_ms = 500;
    while (!Serial) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
    }
  }
  write_all_leds(LOW);

  /* --------------------------------- RTC ---------------------------------- */
  setSyncProvider(get_teensy3_time);
  bool rtc_set = (timeStatus() == timeSet) && (year() > 2021);
  if (!rtc_set) {
    Serial.println("Warning: Failed to sync time with RTC");
  }

  /* ---------------------------------- SD ---------------------------------- */
  sd_initialized = SD.sdfs.begin(SdioConfig(DMA_SDIO));
  if (!sd_initialized) {
    Serial.println("Warning: SD failed to initialize");
  } else {
    char log_name[64];
    u16  log_name_length = 0;

    if (rtc_set) {
      log_name_length = snprintf(
          log_name, sizeof(log_name), "log_%04d-%02d-%02d_%02d-%02d-%02d.bin",
          year(), month(), day(), hour(), minute(), second());
    } else {
      strncpy(log_name, "log_unknown_time.bin", sizeof(log_name));
      log_name_length = 20;
    }

    if (SD.exists(log_name)) {
      char log_name_duplicate[64];
      for (int log_num = 0; log_num < 1000; log_num++) {
        snprintf(log_name_duplicate, sizeof(log_name_duplicate),
                 "%.*s_%03d.bin", log_name_length - 4, log_name, log_num);
        if (!SD.exists(log_name_duplicate)) {
          break;
        }
      }
      strncpy(log_name, log_name_duplicate, sizeof(log_name));
      log_name[sizeof(log_name) - 1] = '\0';
    }

    Serial.printf("Info: Logging to %s\n", log_name);
    logger.logger_init(log_name);
  }

  /* ---------------------------- Sensors + ISRs ----------------------------- */
  sensors_init(&engine_rpm_rotation_filter);

  attachInterrupt(ENGINE_SENSOR_PIN,        on_engine_sensor,         FALLING);
  attachInterrupt(GEARTOOTH_SENSOR_PIN,     on_geartooth_sensor,      FALLING);
  attachInterrupt(LEFT_WHEEL_SENSOR_PIN,    on_lw_geartooth_sensor,   FALLING);
  attachInterrupt(RIGHT_WHEEL_SENSOR_PIN,   on_rw_geartooth_sensor,   FALLING);

  attachInterrupt(ECENTERLOCK_SWITCH_ENGAGE,    on_ecenterlock_switch_engage,    FALLING);
  attachInterrupt(ECENTERLOCK_SWITCH_DISENGAGE, on_ecenterlock_switch_disengage, FALLING);

  attachInterrupt(LIMIT_SWITCH_OUT_PIN,    on_outbound_limit_switch,  FALLING);
  attachInterrupt(LIMIT_SWITCH_ENGAGE_PIN, on_engage_limit_switch,    FALLING);
  attachInterrupt(LIMIT_SWITCH_IN_PIN,     on_inbound_limit_switch,   FALLING);

  /* ---------------------------------- CAN --------------------------------- */
  CanBus::bind(&CAN);
  CAN.begin();

  if (wait_for_can_ecvt) {
    u32 led_flash_time_ms = 100;
    while (odrive.get_time_since_heartbeat_ms() > 100) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
      delay(100);
    }
  }
  write_all_leds(LOW);

  if (using_ecenterlock) {
    u32 led_flash_time_ms = 300;
    while (ecenterlock_odrive.get_time_since_heartbeat_ms() > 100) {
      write_all_leds(millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
      digitalWrite(ECENTERLOCK_SWITCH_LIGHT, millis() % (led_flash_time_ms * 2) < led_flash_time_ms);
      delay(100);
    }
  }
  write_all_leds(LOW);

  /* -------------------------- Subsystem initialization --------------------- */
  u8 odrive_status_code = odrive.init(ODRIVE_ECVT_CURRENT_SOFT_MAX);
  if (odrive_status_code != 0) {
    Serial.printf("Error: ODrive failed to initialize with error %d\n", odrive_status_code);
  }

  odrive_status_code = ecenterlock_odrive.init(ODRIVE_ECENT_CURRENT_SOFT_MAX);
  if (odrive_status_code != 0) {
    Serial.printf("Error: Ecent ODrive failed to initialize with error %d\n", odrive_status_code);
  }

  u8 actuator_status_code = actuator.init();
  if (actuator_status_code != 0) {
    Serial.printf("Error: Actuator failed to initialize with error %d\n", actuator_status_code);
  }

  // TODO: Why do we need delay?
  delay(3000);

  /* ------------------------------ Homing seqs ------------------------------ */
  digitalWrite(LED_2_PIN, HIGH);
  u8 actuator_home_status = actuator.home_encoder(ACTUATOR_HOME_TIMEOUT_MS);
  if (actuator_home_status != 0) {
    Serial.printf("Error: Actuator failed to home with error %d\n", actuator_home_status);
  } else {
    digitalWrite(LED_2_PIN, LOW);
  }

  if (using_ecenterlock) {
    digitalWrite(LED_3_PIN, HIGH);
    u8 ecenterlock_home_status = ecenterlock.home(ECENTERLOCK_HOME_TIMEOUT);
    if (ecenterlock_home_status != 0) {
      Serial.printf("Error: Ecenterlock failed to home with error %d\n", ecenterlock_home_status);
      ecenterlock.change_state(Ecenterlock::UNHOMED);
    } else {
      digitalWrite(LED_3_PIN, LOW);
    }
  }

  /* ------------------------------ Interrupt prio --------------------------- */
  // TODO: Figure out proper ISR priority levels
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
  timer.priority(255);

  /* ------------------------------ Header log ------------------------------- */
  OperationHeader operation_header;

  operation_header.timestamp = now();
  operation_header.clock_us  = micros();
  operation_header.controller_kp = ACTUATOR_KP;
  operation_header.controller_kd = ACTUATOR_KD;
  operation_header.wheel_ref_low_rpm  = WHEEL_REF_LOW_RPM;
  operation_header.wheel_ref_high_rpm = WHEEL_REF_HIGH_RPM;
  operation_header.wheel_ref_breakpoint_low_mph  = WHEEL_REF_BREAKPOINT_LOW_MPH;
  operation_header.wheel_ref_breakpoint_high_mph = WHEEL_REF_BREAKPOINT_HIGH_MPH;

  size_t message_length = logger.encode_pb_message(
      logger.message_buffer(), MESSAGE_BUFFER_SIZE, PROTO_HEADER_MESSAGE_ID,
      OperationHeader_fields, &operation_header);
  logger.write_to_double_buffer(logger.message_buffer(), message_length, false);
  logger.logger_flush();

  /* --------------------------- Controller / Timer -------------------------- */
  CvtController::bind(&controller);

  switch (operating_mode) {
    case OperatingMode::NORMAL:       controller.setMode(CvtController::Mode::Normal);      break;
    case OperatingMode::BUTTON_SHIFT: controller.setMode(CvtController::Mode::ButtonShift); break;
    case OperatingMode::DEBUG:        controller.setMode(CvtController::Mode::Debug);       break;
    default:                          controller.setMode(CvtController::Mode::Normal);      break;
  }

  timer.begin(CvtController::isrTrampoline, CONTROL_FUNCTION_INTERVAL_MS * 1e3);
}

/*==============================================================================
|                                   loop()
==============================================================================*/
void loop() {
  // LED indicators
  // digitalWrite(LED_4_PIN, actuator.get_outbound_limit());
  // digitalWrite(LED_5_PIN, actuator.get_inbound_limit());

  // Flush SD card if buffer full
  logger.logger_flush();
}
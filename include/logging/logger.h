#pragma once
#include <Arduino.h>
#include <SD.h>
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <types.h>
#include <constants.h>

// -----------------------------------------------------------------------------
// Buffer data structure
// -----------------------------------------------------------------------------
struct LogBuffer {
  u8     buffer[LOG_BUFFER_SIZE];
  size_t idx = 0;
  bool   full = false;
};

// -----------------------------------------------------------------------------
// Return codes
// -----------------------------------------------------------------------------
static constexpr u8 DOUBLE_BUFFER_SUCCESS     = 0;
static constexpr u8 DOUBLE_BUFFER_FULL_ERROR  = 1;
static constexpr u8 DOUBLE_BUFFER_INDEX_ERROR = 2;

// -----------------------------------------------------------------------------
// Logger class (fully encapsulated, no globals)
// -----------------------------------------------------------------------------
class Logger {
public:
  // Singleton access (Meyers pattern)
  static Logger& instance() {
    static Logger s;
    return s;
  }

  // Main methods (same names as your procedural API)
  bool   logger_init(const char* log_name);
  void   logger_flush();

  size_t encode_pb_message(u8 buffer[], size_t buffer_length, u8 id,
                           const pb_msgdesc_t* fields, const void* message_struct);

  u8     write_to_double_buffer(u8 data[], size_t data_length, bool split);

  bool& disconnected() { return logging_disconnected_; }           // mutable ref
  bool  is_disconnected() const { return logging_disconnected_; }

  // Optional accessor for message buffer (so other modules can reuse it)
  u8* message_buffer() { return message_buffer_; }

private:
  Logger() = default;
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  // ---------------------------------------------------------------------------
  // Private state — these fix your "identifier undefined" errors
  // ---------------------------------------------------------------------------
  File log_file_;                        // replaces global log_file
  bool logging_disconnected_ = false;    // replaces global logging_disconnected
  u8   cur_buffer_num_ = 0;              // replaces global cur_buffer_num
  LogBuffer double_buffer_[2];           // replaces global double_buffer[2]
  u8   message_buffer_[MESSAGE_BUFFER_SIZE]; // replaces global message_buffer
};


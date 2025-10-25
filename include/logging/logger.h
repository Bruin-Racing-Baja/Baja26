#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <types.h>
#include <pb.h>
#include <constants.h>

// Logging state
extern volatile bool logging_disconnected;

struct LogBuffer {
  char buffer[LOG_BUFFER_SIZE];
  size_t idx;
  bool full;
};

extern u8 cur_buffer_num;
extern LogBuffer double_buffer[2];
extern u8 message_buffer[MESSAGE_BUFFER_SIZE];
extern File log_file;

// Logging functions
size_t encode_pb_message(u8 buffer[], size_t buffer_length, u8 id,
                         const pb_msgdesc_t *fields,
                         const void *message_struct);

constexpr u8 DOUBLE_BUFFER_SUCCESS = 0;
constexpr u8 DOUBLE_BUFFER_FULL_ERROR = 1;
constexpr u8 DOUBLE_BUFFER_INDEX_ERROR = 2;

u8 write_to_double_buffer(u8 data[], size_t data_length,
                          LogBuffer double_buffer[2], u8 *buffer_num,
                          bool split);

// Initialize logging system
bool logger_init(const char* log_name);

// Call in main loop to flush buffers
void logger_flush();

#endif
#include <logging/logger.h>
#include <pb_encode.h>
#include <cstring>

// Logging state variables
volatile bool logging_disconnected = false;
u8 cur_buffer_num = 0;
LogBuffer double_buffer[2];
u8 message_buffer[MESSAGE_BUFFER_SIZE];
File log_file;

size_t encode_pb_message(u8 buffer[], size_t buffer_length, u8 id,
                         const pb_msgdesc_t *fields,
                         const void *message_struct) {
  // Serialize message
  pb_ostream_t ostream = pb_ostream_from_buffer(
      buffer + PROTO_DELIMITER_LENGTH, buffer_length - PROTO_DELIMITER_LENGTH);
  pb_encode(&ostream, fields, message_struct);

  size_t message_length = ostream.bytes_written;

  // Create message delimiter
  char delimiter[PROTO_DELIMITER_LENGTH + 1];
  snprintf(delimiter, PROTO_DELIMITER_LENGTH + 1, "%01X%04X", id,
           message_length);
  memcpy(buffer, delimiter, PROTO_DELIMITER_LENGTH);
  message_length += PROTO_DELIMITER_LENGTH;

  return message_length;
}

u8 write_to_double_buffer(u8 data[], size_t data_length,
                          LogBuffer double_buffer[2], u8 *buffer_num,
                          bool split) {
  LogBuffer *cur_buffer = &double_buffer[*buffer_num];

  if (cur_buffer->full) {
    // If current buffer is full then something is wrong
    return DOUBLE_BUFFER_FULL_ERROR;
  } else if (cur_buffer->idx + data_length > LOG_BUFFER_SIZE) {
    // If data_length exceeds remaining space in buffer

    size_t remaining_space = 0;
    if (split) {
      // Split data across the two buffers
      remaining_space = LOG_BUFFER_SIZE - cur_buffer->idx;
      memcpy(cur_buffer->buffer + cur_buffer->idx, data, remaining_space);
      cur_buffer->idx = LOG_BUFFER_SIZE;
    }

    // Switch to the other buffer
    cur_buffer->full = true;

    *buffer_num = !(*buffer_num);
    cur_buffer = &double_buffer[*buffer_num];

    if (cur_buffer->idx != 0) {
      // If new buffer doesn't start at the beginning then something is wrong
      return DOUBLE_BUFFER_INDEX_ERROR;
    } else {
      // Write data to new buffer
      memcpy(cur_buffer->buffer, data + remaining_space,
             data_length - remaining_space);
      cur_buffer->idx += data_length;
    }
  } else {
    // If data fits in current buffer then write it
    memcpy(cur_buffer->buffer + cur_buffer->idx, data, data_length);
    cur_buffer->idx += data_length;
  }

  return DOUBLE_BUFFER_SUCCESS;
}

bool logger_init(const char* log_name) {
  log_file = SD.open(log_name, FILE_WRITE);
  if (!log_file) {
    Serial.println("Warning: Log file was not opened! (Sarah)");
    return false;
  }
  return true;
}

void logger_flush() {
  extern bool sd_initialized;
  
  if (sd_initialized && !logging_disconnected) {
    for (size_t buffer_num = 0; buffer_num < 2; buffer_num++) {
      if (double_buffer[buffer_num].full) {
        // Serial.printf("Info: Writing buffer %d to SD\n", buffer_num);
        size_t num_bytes_written = log_file.write(
            double_buffer[buffer_num].buffer, double_buffer[buffer_num].idx);
        if (num_bytes_written == 0) {
          logging_disconnected = true;
          digitalWrite(LED_1_PIN, HIGH);
        } else {
          log_file.flush();
          double_buffer[buffer_num].full = false;
          double_buffer[buffer_num].idx = 0;
        }
      }
    }
  } else {
    digitalWrite(LED_1_PIN, HIGH);
  }
}
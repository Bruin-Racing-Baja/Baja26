#include <logging/logger.h>
#include <pb_encode.h>
#include <cstring>

// Initialize SD logging system
bool Logger::logger_init(const char* log_name) {
  log_file_ = SD.open(log_name, FILE_WRITE);
  if (!log_file_) {
    Serial.println("Warning: Log file was not opened!");
    return false;
  }
  return true;
}

// Flush double buffers to SD
void Logger::logger_flush() {
  extern bool sd_initialized;

  if (sd_initialized && !logging_disconnected_) {
    for (size_t i = 0; i < 2; ++i) {
      if (double_buffer_[i].full) {
        size_t written =
            log_file_.write(double_buffer_[i].buffer, double_buffer_[i].idx);
        if (written == 0) {
          logging_disconnected_ = true;
          digitalWrite(LED_1_PIN, HIGH);
        } else {
          log_file_.flush();
          double_buffer_[i].full = false;
          double_buffer_[i].idx = 0;
        }
      }
    }
  } else {
    digitalWrite(LED_1_PIN, HIGH);
  }
}

// Encode protobuf message and prepend delimiter
size_t Logger::encode_pb_message(uint8_t buffer[], size_t buffer_length, uint8_t id,
                                 const pb_msgdesc_t* fields,
                                 const void* message_struct) {
  pb_ostream_t ostream = pb_ostream_from_buffer(
      buffer + PROTO_DELIMITER_LENGTH, buffer_length - PROTO_DELIMITER_LENGTH);
  pb_encode(&ostream, fields, message_struct);

  size_t message_length = ostream.bytes_written;
  char delimiter[PROTO_DELIMITER_LENGTH + 1];
  snprintf(delimiter, sizeof(delimiter), "%01X%04X", id, (unsigned)message_length);
  memcpy(buffer, delimiter, PROTO_DELIMITER_LENGTH);
  message_length += PROTO_DELIMITER_LENGTH;

  return message_length;
}

// Write data to active double buffer, swap when full
uint8_t Logger::write_to_double_buffer(uint8_t data[], size_t data_length, bool split) {
  LogBuffer* cur = &double_buffer_[cur_buffer_num_];

  if (cur->full) {
    return DOUBLE_BUFFER_FULL_ERROR;
  } else if (cur->idx + data_length > LOG_BUFFER_SIZE) {
    size_t remaining_space = 0;
    if (split) {
      remaining_space = LOG_BUFFER_SIZE - cur->idx;
      memcpy(cur->buffer + cur->idx, data, remaining_space);
      cur->idx = LOG_BUFFER_SIZE;
    }

    cur->full = true;
    cur_buffer_num_ = !cur_buffer_num_;
    cur = &double_buffer_[cur_buffer_num_];

    if (cur->idx != 0) {
      return DOUBLE_BUFFER_INDEX_ERROR;
    } else {
      memcpy(cur->buffer, data + remaining_space, data_length - remaining_space);
      cur->idx += (data_length - remaining_space);
    }
  } else {
    memcpy(cur->buffer + cur->idx, data, data_length);
    cur->idx += data_length;
  }

  return DOUBLE_BUFFER_SUCCESS;
}

#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <Arduino.h>
#include <SdFat.h>

class LogBuffer : public Print {
public:
  static const size_t capacity = 384;

  /**
   * @brief Write a single byte to the log buffer.
   * @param value The byte to write.
   * @return The number of bytes written (1 or 0).
   */
  size_t write(uint8_t value) override {
    if (length == capacity && !drain()) {
      return 0;
    }
    buffer[length++] = value;
    return 1;
  }

  /**
   * @brief Write multiple bytes to the log buffer.
   * @param data Pointer to the data to write.
   * @param size Number of bytes to write.
   * @return The number of bytes written.
   */
  size_t write(const uint8_t *data, size_t size) override {
    size_t written = 0;
    while (written < size) {
      size_t available = capacity - length;
      if (available == 0) {
        if (!drain()) {
          break;
        }
        available = capacity;
      }

      size_t chunk = min(available, size - written);
      memcpy(buffer + length, data + written, chunk);
      length += chunk;
      written += chunk;
    }
    return written;
  }

  /**
   * @brief Attach a File32 object to the log buffer for writing.
   * @param file The File32 object to attach.
   */
  void attach(File32 &file) {
    destination = &file;
  }

  /**
   * @brief Drain the log buffer to the attached File32 object.
   * @param file The File32 object to write to.
   * @return True if the drain was successful, false otherwise.
   */
  bool drain(File32 &file) {
    if (length == 0) {
      return true;
    }

    if (file.write(buffer, length) != length) {
      return false;
    }
    length = 0;
    return true;
  }

private:
  bool drain() {
    return destination != nullptr && drain(*destination);
  }

  uint8_t buffer[capacity];
  size_t length = 0;
  File32 *destination = nullptr;
};

#endif

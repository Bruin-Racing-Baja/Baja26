#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <Arduino.h>

class MedianFilter {
public:
  MedianFilter(size_t window_size_);
  ~MedianFilter();

  float update(float new_value);
  float get();

private:
  size_t window_size;
  float *window;
  float *sorted_window;
  size_t head;
};

#endif

#ifndef IIR_FILTER_H
#define IIR_FILTER_H
#include <Arduino.h>
#include <types.h>

class IIRFilter {
public:
  IIRFilter(const float b_[], const float a_[], size_t M_, size_t N_);
  ~IIRFilter();

  float update(float new_x);

  float get();

private:
  float *b;
  float *a;
  float *y;
  float *x;
  size_t M;
  size_t N;
};

#define iirfilter_h
#endif

/*
  Object to create a number that wraps around when it is incremented/decremented past it's max values.
  Used to manage array indicies.
*/

#include "Arduino.h"
#include "wrapCounter.h"

wrapCounter::wrapCounter(void) {}

wrapCounter::wrapCounter(int num_values) {
  _num_values = num_values;
  value = 0;
}

void wrapCounter::setCount(int count) {
  value = count;
}

void wrapCounter::setNumValues(int num_values) {
  _num_values = num_values;
}

boolean wrapCounter::increment(void) {
  value = value + 1;
  if (value > _num_values - 1) {
    value = 0;
    return true;
  }
  return false;
}

boolean wrapCounter::decrement(void) {
  value = value - 1;
  if (value < 0) {
    value = _num_values - 1;
    return true;
  }
  return false;
}

void wrapCounter::reset(void) {
  value = 0;
}

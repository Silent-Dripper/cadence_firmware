/*
  Object to create a number that wraps around when it is incremented/decremented
  past it's max values. Used to manage array indicies.
*/

#include "wrapCounter.h"
#include "Arduino.h"

wrapCounter::wrapCounter(void) {}

/**
 * @brief Construct a new wrap Counter::wrap Counter object.
 *
 * @param num_values The number of positions of the counter.
 */
wrapCounter::wrapCounter(int num_values) {
  _num_values = num_values;
  value = 0;
}

/**
 * @brief Manually change the current value of the counter. No protections
 * around setting it to too large/small of a value.
 *
 * @param count The new value to switch the counter to.
 */
void wrapCounter::setCount(int count) { value = count; }

/**
 * @brief Set the "wrap" point, where after this value the counter's value will
 * be zero.
 *
 * @param num_values The new max position.
 */
void wrapCounter::setNumValues(int num_values) { _num_values = num_values; }

/**
 * @brief Increment the counter to the next position. If we're over the number
 * of values for the counter, wrap back to zero. Return true if a wrap occurs,
 * false otherwise.
 *
 * @return boolean
 */
boolean wrapCounter::increment(void) {
  value = value + 1;
  if (value > _num_values - 1) {
    value = 0;
    return true;
  }
  return false;
}

/**
 * @brief Opposite of increment, decrease the current count. If we decrement at
 * zero, wrap back to the max value. If this wrap occurs, return true.
 *
 * @return boolean
 */
boolean wrapCounter::decrement(void) {
  value = value - 1;
  if (value < 0) {
    value = _num_values - 1;
    return true;
  }
  return false;
}

/**
 * @brief Reset the counter's count back to zero.
 *
 */
void wrapCounter::reset(void) { value = 0; }

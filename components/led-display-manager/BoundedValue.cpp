
#include "BoundedValue.hpp"
#include <algorithm>

BoundedValue::BoundedValue(uint8_t min, uint8_t max, uint8_t step, uint8_t value) :
  _min(min), _max(max), _step(step)
{
  _value = std::max(_min, value);
}

bool BoundedValue::Set(uint8_t value)
{
  if (value < _min || value > _max) return false;

  _value = value;
  return true;
}

#pragma once

#include <stdint.h>

class BoundedValue
{
	private:
	uint8_t _value;
	uint8_t _min;
	uint8_t _max;
	uint8_t _step;

	public:
	BoundedValue(uint8_t min, uint8_t max, uint8_t step = 1, uint8_t value = 0);
	bool Set(uint8_t value);
	uint8_t Max() const { return _max; }
	uint8_t Min() const { return _min; }
	uint8_t Value() const { return _value; }
};

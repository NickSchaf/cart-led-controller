#pragma once

#include <stdint.h>

class SaveableSetting
{
	private:
	bool _changed;
	uint32_t _changedTime;

	public:
	SaveableSetting();
	void SetChanged();
	void ClearChanged();
	bool NeedsSaving();
};

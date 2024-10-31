
#include "SaveableSetting.hpp"
#include "esp_system.h"
#include "esp_timer.h"

#define SAVE_SETTINGS_TIMEOUT      5000

#ifndef millis
unsigned long millis() { return (unsigned long) (esp_timer_get_time() / 1000ULL); }
#endif

SaveableSetting::SaveableSetting()
{
  _changed = false;
  _changedTime = 0;
}

void SaveableSetting::SetChanged()
{
  _changed = true;
  _changedTime = millis();
}

void SaveableSetting::ClearChanged()
{
  _changed = false;
}

bool SaveableSetting::NeedsSaving()
{
  return (_changed && ((millis() - _changedTime) >= SAVE_SETTINGS_TIMEOUT));
}


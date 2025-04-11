#pragma once
#include <Arduino.h>

#include "Globals.h"
#include "esp_log.h"

bool ClockEnable(int pin, int hz);
void ClockDisable();

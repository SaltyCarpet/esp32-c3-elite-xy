#pragma once

#include <Adafruit_NeoPixel.h>

enum SystemStatus {
  STATUS_OK,
  STATUS_WARNING,
  STATUS_ERROR,
  STATUS_BUSY
};

void setStatus(SystemStatus status);
void errorHandlerSetup();

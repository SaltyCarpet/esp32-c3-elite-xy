#include <Adafruit_NeoPixel.h>
#include "ErrorHandler.h"

#define NUMPIXELS   1

Adafruit_NeoPixel pixels;

void setStatus(SystemStatus status) {
  uint32_t color;

  switch (status) {
    case STATUS_OK:
      color = pixels.Color(0, 255, 0);   // Green
      break;
    case STATUS_WARNING:
      color = pixels.Color(255, 165, 0); // Orange
      break;
    case STATUS_ERROR:
      color = pixels.Color(255, 0, 0);   // Red
      break;
    case STATUS_BUSY:
      color = pixels.Color(0, 0, 255);   // Blue
      break;
    default:
      color = pixels.Color(0, 0, 0);     // Off
      break;
  }

  pixels.setPixelColor(0, color);
  pixels.show();

  while(status == STATUS_ERROR) {}
}

void errorHandlerSetup(int led_pin) {
  pixels = Adafruit_NeoPixel(NUMPIXELS, led_pin, NEO_GRB + NEO_KHZ800);
  pixels.begin();
  pixels.setBrightness(50); // dim to save power
  setStatus(STATUS_OK);     // start as OK
}

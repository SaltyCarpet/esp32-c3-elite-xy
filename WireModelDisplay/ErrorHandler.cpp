#include "ErrorHandler.h"
#include <Adafruit_NeoPixel.h>

namespace {

constexpr uint16_t NUMPIXELS = 1;
constexpr int DEVKITC1_V10_RGB_PIN = 48;
constexpr int DEVKITC1_V11_RGB_PIN = 38;

Adafruit_NeoPixel pixels_v10(NUMPIXELS, DEVKITC1_V10_RGB_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_v11(NUMPIXELS, DEVKITC1_V11_RGB_PIN, NEO_GRB + NEO_KHZ800);

void setStripColor(Adafruit_NeoPixel& strip, uint32_t color) {
  strip.setPixelColor(0, color);
  strip.show();
}

uint32_t statusColor(SystemStatus status) {
  switch (status) {
    case STATUS_OK:
      return pixels_v11.Color(0, 255, 0);   // Green
    case STATUS_WARNING:
      return pixels_v11.Color(255, 165, 0); // Orange
    case STATUS_ERROR:
      return pixels_v11.Color(255, 0, 0);   // Red
    case STATUS_BUSY:
      return pixels_v11.Color(0, 0, 255);   // Blue
    default:
      return pixels_v11.Color(0, 0, 0);     // Off
  }
}

}  // namespace

void setStatus(SystemStatus status) {
  const uint32_t color = statusColor(status);
  setStripColor(pixels_v10, color);
  setStripColor(pixels_v11, color);

  // Intentional fatal stop on error.
  while(status == STATUS_ERROR) {}
}

void errorHandlerSetup() {
  pixels_v10.begin();
  pixels_v11.begin();
  pixels_v10.setBrightness(50);
  pixels_v11.setBrightness(50);
  setStatus(STATUS_OK);     // start as OK
}
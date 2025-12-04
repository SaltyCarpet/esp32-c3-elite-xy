#include <Arduino.h>
#include <Preferences.h>
#include <LittleFS.h>
#include "ErrorHandler.h"
#include "esp_system.h"
#include "Game.h"

// -------- Pins --------
#define LED_PIN 38

#define PIN_MOSI 11
#define PIN_CLK 12
#define PIN_CS_XY 10
#define PIN_LDAC 13
#define PIN_Z_1 18
#define PIN_Z_2 8

Preferences prefs;
uint64_t c_millis = 0;
float frameTime = 10;


// ---------- Arduino setup/loop ----------
void reboot_after_flash()
{
  prefs.begin("bootflag", false); // false = read/write
  bool rebootedAfterFlash = prefs.getBool("rebooted", false);
  if (!rebootedAfterFlash) {
    Serial.println("First boot after flashing. Rebooting...");
    prefs.putBool("rebooted", true); // Set flag so it won't reboot again
    prefs.end();
    delay(1000); // Optional: give time for Serial output
    esp_restart(); // Trigger one-time reboot
  } else {
    Serial.println("Normal boot.");
    prefs.end();
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  //reboot_after_flash();
  errorHandlerSetup(LED_PIN);
  setStatus(STATUS_BUSY);
  Serial.println("Ready");
  Serial.printf("Free heap: %lu bytes\n", ESP.getFreeHeap());

  // Initialize drawing pipeline (LUT, DMA, DAC task)
  Drawing_Setup(PIN_MOSI, PIN_CLK, PIN_CS_XY, PIN_Z_1, PIN_Z_2, PIN_LDAC, 80 * 1000000, 32); // max 80
  addPoint(1<<11, 1<<11, 0);
  
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS mounted.");
  //listLittleFS();
  if (!initGame()) {
    Serial.println("Starting Game Failed");
    return;
  }
  Serial.println("Started Game.");

  setStatus(STATUS_OK);
  Serial.println("Setup finished");
}

void loop() {

  serialInterface();
  //Serial.printf("cmd=%d dx=%d dy=%d kd: ax=%d ay=%d az=%d tx=%d ty=%d tz=%d\n",
  //            cmd, dx, dy, kd.max, kd.may, kd.maz, kd.mtx, kd.mty, kd.mtz);


  unsigned long now = millis();
  if((now - c_millis) > frameTime)
  {
    float dt = (now - c_millis) / 1000.0f; // convert ms → seconds
    if (dt > 0.1f) dt = 0.1f;              // clamp to avoid spikes
    c_millis = now;

    updateGame(dt);
    drawModels();
  }
}
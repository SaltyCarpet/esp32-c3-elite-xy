#include <Arduino.h>
//#include <time.h>
#include <Preferences.h>
#include <LittleFS.h>
#include "ErrorHandler.h"
#include "Drawing.h"
#include "esp_system.h"
#include "Wireframe.h"
#include "Models.h"

// -------- Pins --------
#define LED_PIN 38

#define PIN_MOSI 11
#define PIN_CLK 12
#define PIN_CS_XY 10
#define PIN_LDAC 13
#define PIN_Z_1 18
#define PIN_Z_2 8

// --- Protocol constants ---
#define HEADER 0xAA
#define KEY_OFFSET 150
#define CMD_LEN  7

Preferences prefs;
uint64_t c_millis = 0;
WireframeModel SHIPModel;
WireframeModel ROCKModel;
ModelBuf shipbuffer;
ModelBuf rockbuffer;
ModelBuf coordbuffer;
MoveBuf shipmover;
MoveBuf rockmover[50];
MaxMove maxmov;
MaxMove rocklim;
KeyDir kd;
Vec3 kdDir;
float frameTime = 10;

// Map from int key to handler
std::unordered_map<int, std::function<void(float)>> keyMap = {
  { 'w', [](float v){ kd.trans.z = v; } },
  { 's', [](float v){ kd.trans.z = -v; } },
  { 'a', [](float v){ kd.angle.z = v; } },
  { 'd', [](float v){ kd.angle.z = -v; } },
  { 'q', [](float v){ kd.angle.y = -v; } },
  { 'e', [](float v){ kd.angle.y = v; } },
  { 'i', [](float v){ kd.angle.x = v; } },
  { 'k', [](float v){ kd.angle.x = -v; } },
  { 'j', [](float v){ kd.trans.y = -v; } },
  { 'l', [](float v){ kd.trans.y = v; } },
  { 'r', [](float v){ if (v > 0.5f) clearMovBufs(shipmover, kd); } },

  { 0, [](float v){ if (v > 0.5f) clearMovBufs(shipmover, kd); } },
  { KEY_OFFSET, [](float v){ kdDir.x = v;} },
  { KEY_OFFSET + 1, [](float v){ kdDir.y = -v;} },
  { KEY_OFFSET + 4, [](float v){ kd.trans.z = -(1+v)/2; } },
  { KEY_OFFSET + 5, [](float v){ kd.trans.z = (1+v)/2; } },
};


void serialInterface()
{
  // Process serial packets
  static uint8_t buf[CMD_LEN];
  static int idx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    buf[idx++] = b;
    //if (idx >= 8) {idx = 0;}
    
    if (idx >= CMD_LEN) {
      idx = 0;
      //Serial.printf("Packet cmd=%d dx=%d dy=%d checksumOK\n", cmd, dx, dy);

      if (buf[0] != HEADER) continue;

      uint8_t checksum = 0;
      for (int i=0; i<CMD_LEN-1; i++) checksum ^= buf[i];
      if (checksum != buf[CMD_LEN-1]) {
        Serial.println("Bad checksum");
        continue;
      }

      uint8_t key = buf[1];
      float val = 0;
      memcpy(&val, &buf[2], sizeof(float)); // little-endian float
      val = ((val>0.01f)|(val<-0.01f))?val:0.0f;

      //Serial.printf("Got key %d value=%f\n", key, val);

      // Lookup in dictionary
      auto it = keyMap.find((char)key);
      if (it != keyMap.end()) {
        it->second(val);  // call the mapped handler
      }
      else{
        Serial.print("Unknown Key");
      }
    }
  }
}

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
  
  if (!LittleFS.begin(true)) {   // true = format if mount fails
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS mounted.");
  //listLittleFS();
  wireframeInit(1<<11, 1<<11, 1<<12);
  const char* wrlname = "/vrml/adder.wrl";
  bool rtn = loadWRL(wrlname,SHIPModel);
  //Serial.println(WRLModel.vertCount);
  //Serial.println(WRLModel.faceIndexCount);
  //Serial.println(rtn);
  centerAndScale(SHIPModel, 1.0f);
  shipbuffer.model = &SHIPModel;
  wrlname = "/vrml/boulder.wrl";
  rtn = loadWRL(wrlname,ROCKModel);
  centerAndScale(ROCKModel, 5.0f);
  rockbuffer.model = &ROCKModel;
  coordbuffer.model = &coordModel;
  shipmover.pos = {0,0,10};
  shipmover.orientation = {1,-1,0,0};
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

    // Apply continuous actions
    moveBufUpdater(shipmover, kd, maxmov, dt);
    applyRotInputAxis(shipmover, kdDir, {0,0,-1}, {0,1,0}, dt*10);

    transformModel(&shipbuffer, shipmover);
    //transformModel(&coordbuffer, shipmover);
    // All logic runs in tasks
    wireframeDrawCulled(&shipbuffer, 3);
    //wireframeDrawAll(&coordbuffer, 3);
  }
}
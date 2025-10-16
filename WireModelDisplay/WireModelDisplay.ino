#include <Arduino.h>
//#include <time.h>
#include <math.h>
#include <esp_heap_caps.h>
#include "FS.h"
#include "LittleFS.h"
//#include "wifi_setup.h"
#include "Drawing.h"
#include <Preferences.h>
#include "esp_system.h"
#include "Wireframe.h"
#include "Models.h"
//#include "WebSocket.h"

// -------- Pins --------
#define LED_BUILTIN 8

#define PIN_MOSI 4
#define PIN_CLK 5
#define PIN_CS_XY 7
#define PIN_LDAC 2
#define PIN_Z_1 20
#define PIN_Z_2 21

// --- Protocol constants ---
#define HEADER 0xAA
#define CMD_MOVE 0x01
#define CMD_KEY  0x02

Preferences prefs;
uint64_t c_millis = 0;
ModelBuf modelbuffer;
ModelBuf coordbuffer;
WireframeModel WRLModel;
MoveBuf mover;
KeyDir kd;
MaxMove maxmov;
float frameTime = 16;

/*
struct Command {
  const char* name;
  float* target;
};

Command commands[] = {
  {"tx", &tx},  {"ty", &ty},  {"tz", &tz},
  {"ax", &ax},  {"ay", &ay},  {"az", &az},
  {"dtx", &dtx},{"dty", &dty},{"dtz", &dtz},
  {"dax", &dax},{"day", &day},{"daz", &daz},
  {"mv", &move}
};

void moveWithConsole(String in) {
  String cmd = in.substring(0,3);
  cmd.trim();
  int val = in.substring(3).toInt();

  bool found = false;
  for (auto &c : commands) {
    if (cmd.equals(c.name)) {
      *(c.target) = val;
      found = true;
      break;
    }
  }
  if (!found) Serial.println("Unknown");

  Serial.printf("tx=%f,ty=%f,tz=%f,\nax=%f,ay=%f,az=%f,\n", tx,ty,tz,ax,ay,az);
  Serial.printf("dtx=%f,dty=%f,dtz=%f,\ndax=%f,day=%f,daz=%f,\n", dtx,dty,dtz,dax,day,daz);
}
*/

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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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
  bool rtn = loadWRL(wrlname,WRLModel);
  Serial.println(WRLModel.vertCount);
  Serial.println(WRLModel.faceIndexCount);
  Serial.println(rtn);
  centerAndScale(WRLModel, 5.0f);
  modelbuffer.model = &WRLModel;
  coordbuffer.model = &coordModel;
  mover.pos = {0,0,3};
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Setup finished");
}

void loop() {

  // Process serial packets
  static uint8_t buf[8];
  static int idx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    buf[idx++] = b;
    //if (idx >= 8) {idx = 0;}
    
    if (idx >= 8) {
      idx = 0;
      //Serial.printf("Packet cmd=%d dx=%d dy=%d checksumOK\n", cmd, dx, dy);

      if (buf[0] != HEADER) continue;

      uint8_t checksum = 0;
      for (int i=0; i<7; i++) checksum ^= buf[i];
      if (checksum != buf[7]) {
        Serial.println("Bad checksum");
        continue;
      }
      //if (checksum != buf[7]) continue;

      uint8_t cmd = buf[1];
      int16_t dx = buf[2] | (buf[3]<<8);
      int16_t dy = buf[4] | (buf[5]<<8);

      if (cmd == CMD_MOVE) {
        // Mouse move
        applyMouseInputDirect(mover, dx, dy, 0.005f);
      }
      else if (cmd == CMD_KEY) {
        char c = (char)dx;
        bool pressed = (dy != 0);
        //Serial.printf("Got key %c pressed=%d\n", c, pressed);
        switch (c) {
          case 'w': kd.trans.z = pressed; break;
          case 's': kd.trans.z = -pressed; break;
          case 'a': kd.angle.z = pressed; break;
          case 'd': kd.angle.z = -pressed; break;
          case 'q': kd.angle.y = -pressed; break;
          case 'e': kd.angle.y = pressed; break;
          case 'i': kd.angle.x = pressed; break;
          case 'k': kd.angle.x = -pressed; break;
          case 'j': kd.trans.y = -pressed; break;
          case 'l': kd.trans.y = pressed; break;
          case 'r': if (pressed) { clearMovBuf(mover); } break;
        }
      }
    }
    
  }
  //Serial.printf("cmd=%d dx=%d dy=%d kd: ax=%d ay=%d az=%d tx=%d ty=%d tz=%d\n",
  //            cmd, dx, dy, kd.max, kd.may, kd.maz, kd.mtx, kd.mty, kd.mtz);


  unsigned long now = millis();
  if((now - c_millis) > frameTime)
  {
    float dt = (now - c_millis) / 1000.0f; // convert ms → seconds
    if (dt > 0.1f) dt = 0.1f;              // clamp to avoid spikes
    c_millis = now;

    // Apply continuous actions
    moveBufUpdater(mover, kd, maxmov, dt);

    transformModel(&modelbuffer, mover);

    transformModel(&coordbuffer, mover);
  }
  // All logic runs in tasks
  wireframeDrawCulled(&modelbuffer, 3);
  wireframeDrawAll(&coordbuffer, 3);
}
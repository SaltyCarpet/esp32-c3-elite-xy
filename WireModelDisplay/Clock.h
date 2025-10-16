#include <Arduino.h>
#include <time.h>

// -------- Clock layout --------
#define CLOCK_X (1 << 11)
#define CLOCK_Y (1 << 11)
#define CLOCK_R (1 << 11)

const int N = 5;
uint16_t cx[N * 60];
uint16_t cy[N * 60];

// ---------- Drawing functions ----------
static void drawHand(int angleDeg, int length, uint16_t brightness) {
  // SIN_DEG/COS_DEG are scaled by 1024; keep math in float, cast at end.
  int xEnd = (int)(CLOCK_X + (length * (SIN_DEG(angleDeg) / 2048.0f)));
  int yEnd = (int)(CLOCK_Y + (length * (COS_DEG(angleDeg) / 2048.0f)));
  drawLineBresenham(CLOCK_X, CLOCK_Y, xEnd, yEnd, brightness);
}

static void drawClock(int hour, int minute, int second) {
  //Serial.printf("Drawing clock: %02d:%02d:%02d\n", hour, minute, second);

  flushRing();
  // Convert to integer degrees
  int hourAngle   = ((hour % 12) * 30) + (minute / 2);
  int minuteAngle = minute * 6;
  int secondAngle = second * 6;

  // Hands
  addPoint(CLOCK_X, CLOCK_Y, 0);
  drawHand(hourAngle,   (int)(CLOCK_R * 0.5f), 3);
  addPoint(CLOCK_X, CLOCK_Y, 0);
  drawHand(minuteAngle, (int)(CLOCK_R * 0.8f), 2);
  addPoint(CLOCK_X, CLOCK_Y, 0);
  drawHand(secondAngle, (int)(CLOCK_R * 1.0f), 1);

  // Hour ticks using LUT (degrees)
  for (int i = 0; i < 12; i++) {
    int angle = i * 30;
    float s = SIN_DEG(angle) / 2048.0f;
    float c = COS_DEG(angle) / 2048.0f;

    int x1 = (int)(CLOCK_X + ((CLOCK_R * 0.9f) * s));
    int y1 = (int)(CLOCK_Y + ((CLOCK_R * 0.9f) * c));
    int x2 = (int)(CLOCK_X + (CLOCK_R * s));
    int y2 = (int)(CLOCK_Y + (CLOCK_R * c));
    drawLineBresenham(x1, y1, x2, y2, 2);
  }
  moveToBlank(cx[0], cy[0]);
}

// ---------- Tasks ----------
static void timeTask(void* pvParameters) {
  (void)pvParameters;
  struct tm timeinfo;
  const TickType_t timeout = pdMS_TO_TICKS(100);  // 500 ms timeout
  TickType_t startTick;

  while (true) {
    bool gotTime = false;
    startTick = xTaskGetTickCount();

    // Try to get time within timeout window
    while ((xTaskGetTickCount() - startTick) < timeout) {
      if (getLocalTime(&timeinfo, 8)) {  // 8 ms internal timeout
        gotTime = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(4));  // Yield briefly
    }

    if (gotTime) {
      drawClock(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      // Fallback: show 12:30:15
      drawClock(0, 30, 15);
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // Regular update interval
  }
}
  // Optional WiFi + NTP setup
  //setupWiFiFromConsole();
  //xTaskCreate(timeTask, "Time Task", 4096, NULL, 1, NULL);
  //configTime(7200, 0, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  //waitForNTP();

  //setupWebsocket();
  /*
  xTaskCreate([](void*) {
      for (;;) {
        loopWebsocket();
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }, "WS Task", 8192, nullptr, 1, nullptr);
  */
#include "freertos/idf_additions.h"
#include <sys/_stdint.h>
#include <Arduino.h>
#include <math.h>
#include "Drawing.h"
#include "MCP4922DMA.h"

// -------- Ring buffer storage (internal) --------
static VectorPoint g_ring[RB_CAP];
static volatile uint32_t g_rb_head = 0;
static volatile uint32_t g_rb_tail = 0;

// DMA frame buffer (internal)
static uint16_t* g_frame_words = nullptr;

// -------- Barriers and RB helpers (internal) --------
static inline void rb_barrier() { __asm__ __volatile__("" ::: "memory"); }

static inline void rb_push_overwrite(VectorPoint p) {
  uint32_t head = g_rb_head;
  if (head - g_rb_tail >= RB_CAP) g_rb_tail++;
  g_ring[head & RB_MASK] = p;
  rb_barrier();
  g_rb_head = head + 1;
}

static inline bool rb_pop(VectorPoint& out) {
  uint32_t tail = g_rb_tail;
  if (tail == g_rb_head) return false;
  out = g_ring[tail & RB_MASK];
  rb_barrier();
  g_rb_tail = tail + 1;
  return true;
}

// ---------- Task ----------
static void DACTask(void* pvParameters) {
  (void)pvParameters;
  VectorPoint p;

  while (true) {
    int n = 0;
    //Serial.printf("Length: %d\n", g_rb_head - g_rb_tail);
    while (n < MAX_POINTS && rb_pop(p)) {
      g_frame_words[3 * n + 0] = p.x;
      g_frame_words[3 * n + 1] = p.y;
      g_frame_words[3 * n + 2] = p.brightness;
      ++n;
    }
    if (n > 0) {
      // Append an explicit blank to clear Z between bursts
      if (n < MAX_POINTS) {
        g_frame_words[3*n + 0] = p.x;
        g_frame_words[3*n + 1] = p.y;
        g_frame_words[3*n + 2] = 0;            // Z=0 blank
        ++n;
      }
      //Serial.printf("Sending %d points\n", n);

      //Serial.println("Calling MCP4922_DMA_send_XYZ...");
      MCP4922_DMA_send_XYZ(g_frame_words, n);
      //Serial.println("Returned from MCP4922_DMA_send_XYZ");

      // Loop immediately to keep the pipe full if more points are available
      continue;
    }
    static TickType_t last = 0;
    if (xTaskGetTickCount() - last > pdMS_TO_TICKS(10000)) {
      last = xTaskGetTickCount();
      Serial.printf("DACTask HWM: %u words\n", (unsigned)uxTaskGetStackHighWaterMark(NULL));
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- Public drawing helpers ----------
void flushRing()
{
  g_rb_tail = g_rb_head; // flush ring
}

void addPoint(uint16_t x, uint16_t y, uint16_t brightness) {
  if(x>=MAX_VAL || y>=MAX_VAL) return;
  VectorPoint p2 = { MCP4922_cmdX((uint16_t)(x * SCALE_X)), MCP4922_cmdY((uint16_t)(y * SCALE_Y)), MCP4922_cmdZ((uint16_t)(brightness)<=3 ? brightness : 3)};
  //Serial.printf("%d.%d.%d\n",x,y,brightness);
  rb_push_overwrite(p2);
}

void drawLineBresenham(int x0, int y0, int x1, int y1, uint16_t brightness) {
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int skip = ((1 + (dx + dy)/5) < MAX_STEP_SIZE) ? (1 + (dx + dy)/5) : MAX_STEP_SIZE;

    moveToBlank(x0, y0);
    int count = 0;
    while (true) {
        if (count++ % skip == 0) {
            addPoint((uint16_t)x0, (uint16_t)y0, brightness);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

void moveToBlank(uint16_t x, uint16_t y) {
  addPoint(x, y, 0);
}

// ---------- Setup ----------
void Drawing_Setup(int mosi, int clk, int csxy, int z1, int z2, int ldac, int clock_hz, int queue_depth) {

  g_frame_words = (uint16_t*)heap_caps_malloc(3 * MAX_POINTS * sizeof(uint16_t), MALLOC_CAP_DMA);
  if (!g_frame_words) {
    Serial.println("DMA buffer alloc failed");
    abort();
  }

  MCP4922_DMA_init_dual(mosi, clk, csxy, z1, z2, ldac, clock_hz, queue_depth);
  xTaskCreatePinnedToCore(DACTask, "DAC Task", DAC_TASK_STACK, NULL, DAC_TASK_PRIO, NULL, 0);
}

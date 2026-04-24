#include "freertos/idf_additions.h"
#include <atomic>
#include <sys/_stdint.h>
#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Drawing.h"
#include "MCP4922DMA.h"

static_assert(sizeof(VectorPoint) == (3 * sizeof(uint16_t)), "VectorPoint must stay tightly packed");

struct FrameBuf {
  VectorPoint points[MAX_FRAME_POINTS];
  int count = 0;
};

static FrameBuf g_buffers[3];
static FrameBuf* g_writeBuf = &g_buffers[0];
static FrameBuf* g_readyBuf = &g_buffers[1];
static FrameBuf* g_displayBuf = &g_buffers[2];
static std::atomic<bool> g_swapReady{false};
static TaskHandle_t g_dacTaskHandle = nullptr;

static std::atomic<uint32_t> g_displayScans{0};
static std::atomic<uint32_t> g_frameSwaps{0};
static std::atomic<uint32_t> g_idleWaits{0};
static std::atomic<uint32_t> g_beginRejected{0};
static std::atomic<uint32_t> g_presentRejected{0};
static std::atomic<uint32_t> g_lastDisplayPointCount{0};

static inline uint16_t clamp12(int v)
{
  if (v < 0) return 0;
  if (v > DAC_MAX) return DAC_MAX;
  return (uint16_t)v;
}

static inline bool inLogicalBounds(int x, int y)
{
  return x >= 0 && x <= COORD_MAX_X && y >= 0 && y <= COORD_MAX_Y;
}

static inline uint16_t mapLogicalXToDac(int x)
{
  return clamp12(x + OFFSET_X);
}

static inline uint16_t mapLogicalYToDac(int y)
{
  return clamp12(y + OFFSET_Y);
}

int drawingCenterX()
{
  return COORD_MAX_X / 2;
}

int drawingCenterY()
{
  return COORD_MAX_Y / 2;
}

int drawingProjectionScale()
{
  return COORD_MAX_Y / 2;
}

void drawingGetStats(DrawingStats& out)
{
  out.displayScans = g_displayScans.load(std::memory_order_relaxed);
  out.frameSwaps = g_frameSwaps.load(std::memory_order_relaxed);
  out.idleWaits = g_idleWaits.load(std::memory_order_relaxed);
  out.beginRejected = g_beginRejected.load(std::memory_order_relaxed);
  out.presentRejected = g_presentRejected.load(std::memory_order_relaxed);
  out.lastDisplayPointCount = g_lastDisplayPointCount.load(std::memory_order_relaxed);
}

enum { INSIDE = 0, LEFT = 1, RIGHT = 2, BOTTOM = 4, TOP = 8 };

static inline int outcode(int x, int y, int xmin, int ymin, int xmax, int ymax)
{
  int code = INSIDE;
  if (x < xmin) code |= LEFT;
  else if (x > xmax) code |= RIGHT;
  if (y < ymin) code |= BOTTOM;
  else if (y > ymax) code |= TOP;
  return code;
}

static bool clipLine(int& x0, int& y0, int& x1, int& y1,
                     int xmin, int ymin, int xmax, int ymax)
{
  int code0 = outcode(x0, y0, xmin, ymin, xmax, ymax);
  int code1 = outcode(x1, y1, xmin, ymin, xmax, ymax);

  while (true) {
    if (!(code0 | code1)) return true;
    if (code0 & code1) return false;

    const int code = code0 ? code0 : code1;
    int x = 0;
    int y = 0;

    if ((code & TOP) && (y1 != y0)) {
      x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
      y = ymax;
    } else if ((code & BOTTOM) && (y1 != y0)) {
      x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
      y = ymin;
    } else if ((code & RIGHT) && (x1 != x0)) {
      y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
      x = xmax;
    } else if ((code & LEFT) && (x1 != x0)) {
      y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
      x = xmin;
    } else {
      return false;
    }

    if (code == code0) {
      x0 = x;
      y0 = y;
      code0 = outcode(x0, y0, xmin, ymin, xmax, ymax);
    } else {
      x1 = x;
      y1 = y;
      code1 = outcode(x1, y1, xmin, ymin, xmax, ymax);
    }
  }
}

static inline void swapReadyFrameIfNeeded()
{
  if (!g_swapReady.load(std::memory_order_acquire)) {
    return;
  }

  FrameBuf* tmp = g_displayBuf;
  g_displayBuf = g_readyBuf;
  g_readyBuf = tmp;
  g_swapReady.store(false, std::memory_order_release);
  g_frameSwaps.fetch_add(1, std::memory_order_relaxed);
}

static void DACTask(void* pvParameters)
{
  (void)pvParameters;

  while (true) {
    swapReadyFrameIfNeeded();

    FrameBuf* display = g_displayBuf;
    const int n = display->count;
    g_lastDisplayPointCount.store((uint32_t)n, std::memory_order_relaxed);

    if (n > 0) {
      MCP4922_DMA_send_XYZ(reinterpret_cast<const uint16_t*>(display->points), n);

      uint16_t restartBlank[3] = {
        display->points[0].x,
        display->points[0].y,
        MCP4922_cmdZ(0)
      };
      MCP4922_DMA_send_XYZ(restartBlank, 1);
      g_displayScans.fetch_add(1, std::memory_order_relaxed);

      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DAC_REFRESH_WAIT_MS));
      continue;
    }

    g_idleWaits.fetch_add(1, std::memory_order_relaxed);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
  }
}

void flushRing()
{
  g_buffers[0].count = 0;
  g_buffers[1].count = 0;
  g_buffers[2].count = 0;
  g_writeBuf = &g_buffers[0];
  g_readyBuf = &g_buffers[1];
  g_displayBuf = &g_buffers[2];
  g_swapReady.store(false, std::memory_order_release);

  g_displayScans.store(0, std::memory_order_relaxed);
  g_frameSwaps.store(0, std::memory_order_relaxed);
  g_idleWaits.store(0, std::memory_order_relaxed);
  g_beginRejected.store(0, std::memory_order_relaxed);
  g_presentRejected.store(0, std::memory_order_relaxed);
  g_lastDisplayPointCount.store(0, std::memory_order_relaxed);
}

bool beginFrame()
{
  if (g_swapReady.load(std::memory_order_acquire)) {
    g_beginRejected.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  g_writeBuf->count = 0;
  return true;
}

bool presentFrame()
{
  if (g_swapReady.load(std::memory_order_acquire)) {
    g_presentRejected.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  FrameBuf* tmp = g_readyBuf;
  g_readyBuf = g_writeBuf;
  g_writeBuf = tmp;
  g_swapReady.store(true, std::memory_order_release);

  if (g_dacTaskHandle != nullptr) {
    xTaskNotifyGive(g_dacTaskHandle);
  }
  return true;
}

void addPoint(uint16_t x, uint16_t y, uint16_t brightness)
{
  if (!inLogicalBounds(x, y)) return;

  FrameBuf* buf = g_writeBuf;
  if (buf->count >= MAX_FRAME_POINTS) return;

  VectorPoint& p = buf->points[buf->count++];
  p.x = MCP4922_cmdX((uint16_t)lrintf((float)mapLogicalXToDac((int)x) * SCALE_X));
  p.y = MCP4922_cmdY((uint16_t)lrintf((float)mapLogicalYToDac((int)y) * SCALE_Y));
  p.brightness = MCP4922_cmdZ((brightness <= 3) ? brightness : 3);
}

void moveToBlank(uint16_t x, uint16_t y)
{
  addPoint(x, y, 0);
}

void drawLineBresenham(int x0, int y0, int x1, int y1, uint16_t brightness)
{
  if (!clipLine(x0, y0, x1, y1, 0, 0, COORD_MAX_X, COORD_MAX_Y)) {
    return;
  }

  moveToBlank((uint16_t)x0, (uint16_t)y0);

  const int dist = abs(x1 - x0) + abs(y1 - y0);
  int steps = dist / 64;
  if (steps < 1) steps = 1;
  if (steps > MAX_STEP_SIZE) steps = MAX_STEP_SIZE;

  for (int i = 0; i <= steps; ++i) {
    const int x = x0 + ((x1 - x0) * i) / steps;
    const int y = y0 + ((y1 - y0) * i) / steps;
    if (!inLogicalBounds(x, y)) continue;
    addPoint((uint16_t)x, (uint16_t)y, brightness);
  }
}

void Drawing_Setup(int mosi, int clk, int csxy, int z1, int z2, int ldac, int clock_hz, int queue_depth)
{
  flushRing();
  MCP4922_DMA_init_dual(mosi, clk, csxy, z1, z2, ldac, clock_hz, queue_depth);
  if (g_dacTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(DACTask, "DAC Task", DAC_TASK_STACK, NULL, DAC_TASK_PRIO, &g_dacTaskHandle, DAC_TASK_CORE);
  }
}

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "MCP4922DMA.h"

#define SCALE_X 0.35f
#define SCALE_Y (1.25f * SCALE_X)
#define MAX_VAL (1 << 12)
#define DAC_MAX (MAX_VAL - 1)
#define MAX_STEP_SIZE 40

#define DAC_TASK_STACK 8192
#define DAC_TASK_PRIO 3
#define DAC_TASK_CORE 1
#define DAC_REFRESH_WAIT_MS 1
#define MAX_FRAME_POINTS 2048

#define SCREEN_ASPECT_NUM 5
#define SCREEN_ASPECT_DEN 4
#define SCREEN_ASPECT ((float)SCREEN_ASPECT_NUM / (float)SCREEN_ASPECT_DEN)

#if SCREEN_ASPECT_NUM >= SCREEN_ASPECT_DEN
  #define COORD_MAX_X DAC_MAX
  #define COORD_MAX_Y ((DAC_MAX * SCREEN_ASPECT_DEN) / SCREEN_ASPECT_NUM)
  #define OFFSET_X 0
  #define OFFSET_Y ((DAC_MAX - COORD_MAX_Y) / 2)
#else
  #define COORD_MAX_X ((DAC_MAX * SCREEN_ASPECT_NUM) / SCREEN_ASPECT_DEN)
  #define COORD_MAX_Y DAC_MAX
  #define OFFSET_X ((DAC_MAX - COORD_MAX_X) / 2)
  #define OFFSET_Y 0
#endif

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t brightness;
} VectorPoint;

struct DrawingStats {
  uint32_t displayScans = 0;
  uint32_t frameSwaps = 0;
  uint32_t idleWaits = 0;
  uint32_t beginRejected = 0;
  uint32_t presentRejected = 0;
  uint32_t lastDisplayPointCount = 0;
};

void flushRing();
bool beginFrame();
bool presentFrame();
void addPoint(uint16_t x, uint16_t y, uint16_t brightness);
void drawLineBresenham(int x0, int y0, int x1, int y1, uint16_t brightness);
void moveToBlank(uint16_t x, uint16_t y);
int drawingCenterX();
int drawingCenterY();
int drawingProjectionScale();
void drawingGetStats(DrawingStats& out);
void Drawing_Setup(int mosi, int clk, int csxy, int z1, int z2, int ldac, int clock_hz, int queue_depth);

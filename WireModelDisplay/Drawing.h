#pragma once

#include <Arduino.h>
#include "MCP4922DMA.h"

// Debug Scaling
#define SCALE_X 0.35f
#define SCALE_Y 1.25f * SCALE_X
#define MAX_VAL (1<<12)
#define MAX_STEP_SIZE 40

// -------- Task config --------
#define DAC_TASK_STACK 8192
#define DAC_TASK_PRIO 3

// -------- Frame/points --------
#define RB_CAP (1 << 11)
#define RB_MASK (RB_CAP - 1)
#define MAX_POINTS RB_MASK

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t brightness;
} VectorPoint;

void flushRing();
void addPoint(uint16_t x, uint16_t y, uint16_t brightness);
void drawLineBresenham(int x0, int y0, int x1, int y1, uint16_t brightness);
void moveToBlank(uint16_t x, uint16_t y);
void Drawing_Setup(int mosi, int clk, int csxy, int z1, int z2, int ldac,  int clock_hz, int queue_depth);
#include "WebSocket.h"
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include "Drawing.h"

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static const int xres = 32;
static const int yres = 25;
static uint8_t wsFrame[xres][yres];

static inline uint16_t mapX(int x) { return (uint16_t)((uint32_t)x * 4095u / (xres - 1)); }
static inline uint16_t mapY(int y) { return (uint16_t)((uint32_t)y * 4095u / (yres - 1)); }

static bool parseU32(const char*& s, uint32_t& out) {
  while (*s == ' ' || *s == '\t' || *s == ',' ) ++s;
  if (*s < '0' || *s > '9') return false;
  uint32_t v = 0;
  while (*s >= '0' && *s <= '9') { v = v*10 + (*s - '0'); ++s; }
  out = v; return true;
}

static void frameToVectors(uint8_t threshold = 16) {
  for (int y = 0; y < yres; ++y) {
    int runStart = -1;
    uint8_t runB = 0;

    for (int x = 0; x <= xres; ++x) {
      uint8_t v = (x < xres) ? wsFrame[x][y] : 0;
      bool on = v >= threshold;

      if (on) {
        if (runStart < 0) { runStart = x; runB = v; }
      } else if (runStart >= 0) {
        int runEnd = x - 1;
        uint16_t x0 = mapX(runStart);
        uint16_t x1 = mapX(runEnd);
        uint16_t yy = mapY(y);
        uint16_t b12 = ((uint16_t)runB) << 4;

        addPoint(x0, yy, 0);
        drawLineBresenham(x0, yy, x1, yy, b12);
        addPoint(x1, yy, 0);
        runStart = -1;
      }
    }
  }
}

static void handleTextCommand(const char* s) {
  while (*s) {
    while (*s == '\r' || *s == '\n') ++s;
    if (!*s) break;
    char cmd = *s++;
    if (cmd == 'L') {
      uint32_t x0,y0,x1,y1,b;
      if (parseU32(s,x0) && parseU32(s,y0) && parseU32(s,x1) && parseU32(s,y1) && parseU32(s,b)) {
        uint16_t X0 = min<uint32_t>(4095, x0);
        uint16_t Y0 = min<uint32_t>(4095, y0);
        uint16_t X1 = min<uint32_t>(4095, x1);
        uint16_t Y1 = min<uint32_t>(4095, y1);
        uint16_t B  = min<uint32_t>(4095, (b <= 255 ? (b << 4) : b));
        addPoint(X0, Y0, 0);
        drawLineBresenham(X0, Y0, X1, Y1, B);
        addPoint(X1, Y1, 0);
      }
    } else if (cmd == 'P') {
      uint32_t x,y,b;
      if (parseU32(s,x) && parseU32(s,y) && parseU32(s,b)) {
        uint16_t X = min<uint32_t>(4095, x);
        uint16_t Y = min<uint32_t>(4095, y);
        uint16_t B = min<uint32_t>(4095, (b <= 255 ? (b << 4) : b));
        addPoint(X, Y, B);
      }
    } else {
      while (*s && *s != '\n') ++s;
    }
  }
}

static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      Serial.printf("[WS] Client connected: %lu\n", (unsigned long)client->id());
      client->text("Connected");
      break;
    }

    case WS_EVT_DISCONNECT: {
      Serial.printf("[WS] Client disconnected: %lu\n", (unsigned long)client->id());
      break;
    }

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len) {
        if (info->opcode == WS_TEXT) {
          data[len] = '\0'; // null-terminate
          handleTextCommand((const char*)data);
        } else if (info->opcode == WS_BINARY) {
          size_t needed = (xres * yres + 1) / 2;
          if (len < needed) return;

          const uint8_t* pix = data;
          size_t i = 0;
          for (int y = 0; y < yres; ++y) {
            for (int x = 0; x < xres; x += 2) {
              uint8_t p = pix[i++];
              wsFrame[x + 0][y] = (p & 0x0F) * 0x11;
              if (x + 1 < xres) wsFrame[x + 1][y] = ((p >> 4) & 0x0F) * 0x11;
            }
          }
          frameToVectors(16);
        }
      }
      break;
    }

    default:
      break;
  }
}

void setupWebsocket() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loopWebsocket() {
  ws.cleanupClients(); // Optional
}

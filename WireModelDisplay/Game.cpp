#include <Arduino.h>
#include <vector>
#include "Game.h"
#include "MathHelpers.h"
#include "Models.h"
#include "Wireframe.h"

// --- Protocol constants ---
#define HEADER 0xAA
#define KEY_OFFSET 150
#define CMD_LEN  7

#define DEPTH 15
#define POSLIM DEPTH/2.1f

float asteroidTime = 0;
float asteroidDif = 1;

int score = 0;

ModelBuf shipbuffer;
ModelBuf asteroidbuffer;
ModelBuf missilebuffer;
ModelBuf coordbuffer;
MoveBuf shipmover;
std::vector<MoveBuf> asteroidmover;
std::vector<MoveBuf> missilemover;
MaxMove maxmov;
MaxMove linlim;
KeyDir kd;
KeyDir Zero;
Vec3 kdDir;

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
  { 'r', [](float v){ if (v > 0.5f) resetGame(); } },

  { 0, [](float v){ if (v > 0.5f) playerShoot(); } },
  { 1, [](float v){ if (v > 0.5f) resetGame(); } },
  { 2, [](float v){ if (v > 0.5f) createRandomAsteroid(); } },
  { 3, [](float v){ if (v > 0.5f) printShipPos(shipmover); } },
  { KEY_OFFSET, [](float v){ kdDir.x = v;} },
  { KEY_OFFSET + 1, [](float v){ kdDir.y = -v;} },
  { KEY_OFFSET + 4, [](float v){ kd.trans.z = -(1+v)/2; } },
  { KEY_OFFSET + 5, [](float v){ kd.trans.z = (1+v)/2; } },
};

bool initGame()
{
    wireframeInit(1<<11, 1<<11, 1<<12);
    const char* wrlname = "/vrml/adder.wrl";
    bool rtn = initModelBuf(wrlname, 1.0f, shipbuffer);
    wrlname = "/vrml/boulder.wrl";
    rtn &= initModelBuf(wrlname, 1.0f, asteroidbuffer);
    wrlname = "/vrml/missile.wrl";
    rtn &= initModelBuf(wrlname, 0.5f, missilebuffer);
    rtn &= centerAndScale(coordbuffer, coordModel, 1.0f);
    if(!rtn) {return false;}
    shipmover.pos = {0,0,DEPTH};
    shipmover.orientation = {1,-1,0,0};
    linlim.maxtransdamp = {0,0,0};
    linlim.maxangdamp = {0,0,0};
    return true;
}

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

inline void shootMissile(const MoveBuf& atkr)
{
    MoveBuf missile;
    missile.pos = atkr.pos;
    missile.orientation = atkr.orientation;
    missile.angAcc = {0,0,0};
    missile.linAcc = {0,0,0};
    missile.angVel = {0,0,0};
    missile.linVel = {0,0,20};
    missilemover.push_back(missile);
}

inline bool isModelOnScreen(const MoveBuf& mov)
{
    // Check bounds
    float x = mov.pos.x;
    float y = mov.pos.y;
    float lim = (mov.pos.z/2.0f) * 1.1f;
    return (x >= -lim && x < lim && y >= -lim && y < lim);
}


void playerShoot()
{
    shootMissile(shipmover);
}

inline void updateMissiles(float dt)
{
    for (auto& missile : missilemover)
    {
        moveBufUpdater(missile, Zero, linlim, dt);
        for (auto& asteroid : asteroidmover)
        {
            if (collideSphere(missile, asteroid, missilebuffer.radius, asteroidbuffer.radius))
            {
                asteroid.health = 0;
                missile.health = 0;
                score += 1;
            }
        }
        if (!isModelOnScreen(missile))
        {
            missile.health = 0;
        }
    }
    // Remove all missiles with health == 0
    missilemover.erase(
        std::remove_if(missilemover.begin(), missilemover.end(),
                   [](const MoveBuf& m){ return m.health == 0; }),
        missilemover.end()
    );
}

// Create a random asteroid MoveBuf
void createRandomAsteroid()
{
    if (asteroidmover.size() > 10) return;

    MoveBuf asteroid;

    // --- pick a random side ---
    int side = (int)randFloat(0, 4); // 0=left,1=right,2=top,3=bottom
    Vec3 start, target;
    float a = randFloat(-POSLIM, POSLIM);
    float b = randFloat(-POSLIM, POSLIM);

    switch (side) {
        case 0: // left edge
            start = {-POSLIM, a, DEPTH};
            target = {POSLIM, b, DEPTH};
            break;
        case 1: // right edge
            start = {POSLIM, a, DEPTH};
            target = {-POSLIM, b, DEPTH};
            break;
        case 2: // top edge
            start = {a, POSLIM, DEPTH};
            target = {b, -POSLIM, DEPTH};
            break;
        case 3: // bottom edge
            start = {a, -POSLIM, DEPTH};
            target = {b, POSLIM, DEPTH};
            break;
    }

    // --- position ---
    asteroid.pos = start;

    // --- orientation ---
    asteroid.orientation = quatIdentity();
    float maxrot = 20.0f;
    asteroid.angVel = {randFloat(-maxrot, maxrot),
                       randFloat(-maxrot, maxrot),
                       randFloat(-maxrot, maxrot)};
    asteroid.angAcc = {0,0,0};

    // --- linear velocity ---
    Vec3 dir = target - start;
    dir = normalize(dir);
    if (dir.x == 0 && dir.y == 0 && dir.z == 0) return;

    asteroid.linVel = dir * randFloat(0.5f, 2);
    asteroid.linAcc = {0,0,0};

    asteroidmover.push_back(asteroid);
}

inline void updateAsteroids(float dt)
{
    for (auto& asteroid : asteroidmover)
    {
        moveBufUpdater(asteroid, Zero, linlim, dt);
        if (collideSphere(shipmover, asteroid, shipbuffer.radius, asteroidbuffer.radius))
        {
            shipmover.health = 0;
        }
        if (!isModelOnScreen(asteroid))
        {
            asteroid.health = 0;
        }
    }
    // Remove all missiles with health == 0
    asteroidmover.erase(
        std::remove_if(asteroidmover.begin(), asteroidmover.end(),
                   [](const MoveBuf& m){ return m.health == 0; }),
        asteroidmover.end()
    );
}


void updateGame(float dt)
{
    // Apply continuous actions
    moveBufUpdater(shipmover, kd, maxmov, dt);
    applyRotInputAxis(shipmover, kdDir, {0,0,-1}, {0,1,0}, dt*10);
    shipmover.pos.z = DEPTH;

    updateMissiles(dt);
    updateAsteroids(dt);

    asteroidTime += dt;
    if (asteroidDif < asteroidTime)
    {
        createRandomAsteroid();
        asteroidTime = 0.0f;
    }

    if(shipmover.health == 0)
    {
        shipmover.health = 1;
        resetGame();
    }
}

void drawModels()
{
    //transformModel(&coordbuffer, shipmover);
    //wireframeDrawAll(&coordbuffer, 3);
    transformModel(&shipbuffer, shipmover);
    wireframeDrawCulled(&shipbuffer, 3);

    for (auto& missile : missilemover)
    {
        transformModel(&missilebuffer, missile);
        wireframeDrawCulled(&missilebuffer, 3);
    }

    for (auto& asteroid : asteroidmover)
    {
        transformModel(&asteroidbuffer, asteroid);
        wireframeDrawCulled(&asteroidbuffer, 3);
    }

    drawText(std::to_string(score), 2048, 2048, 10, 3);
}

void resetGame()
{
    score = 0;
    missilemover.clear();
    asteroidmover.clear();
    clearMovBufs(shipmover, kd);
    Serial.println("RESET");
}
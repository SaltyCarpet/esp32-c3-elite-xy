#pragma once

#include <Arduino.h>
#include <vector>
#include "Wireframe.h"
#include "MathHelpers.h"


bool initGame();
void serialInterface();
void playerShoot();
void createRandomAsteroid();
void updateGame(float dt);
void drawModels();
void resetGame();

static inline void printShipPos(MoveBuf mov)
{
  Serial.print("Ship pos: x=");
  Serial.print(mov.pos.x);
  Serial.print(" y=");
  Serial.print(mov.pos.y);
  Serial.print(" z=");
  Serial.println(mov.pos.z);
}
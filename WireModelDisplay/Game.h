#pragma once

#include <Arduino.h>
#include <vector>
#include "Wireframe.h"
#include "MathHelpers.h"


bool initGame();
void serialInterface();
void playerShoot();
void updateGame(float dt);
void drawModels();
void resetGame();
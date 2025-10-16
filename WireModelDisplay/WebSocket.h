#pragma once
#include <stdint.h>

// Call once in setup()
void setupWebsocket();

// Call regularly in a task
void loopWebsocket();
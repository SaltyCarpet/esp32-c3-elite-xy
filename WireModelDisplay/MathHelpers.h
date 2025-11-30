#pragma once

#include <Arduino.h>
#include <math.h>
#include "esp_dsp.h"   // ESP-DSP library
#include "ErrorHandler.h"

struct __attribute__((aligned(16))) Quaternion { float w = 1, x = 0, y = 0, z = 0; };
struct __attribute__((aligned(16))) Vec3 { float x = 0, y = 0, z = 0; };
struct Vec2 { int x = 0, y = 0; };

// -----------------------------
// Math helpers
// -----------------------------

float fastSin(float x);

float fastCos(float x);

float fastSqrt(float x);

float fastInvSqrt(float x);

float dot(const float* u, const float* v, int dim = 3);


Vec3 normalize(const Vec3& v);

Vec3 cross(const Vec3& u, const Vec3& v);

Quaternion quatIdentity();

Quaternion quatConjugate(const Quaternion& q);

Quaternion quatNormalize(const Quaternion& q);

Quaternion quatMultiply(const Quaternion& a, const Quaternion& b);

Quaternion quatFromAxisAngle(float ax, float ay, float az, float angle);

Vec3 rotateVector(const Quaternion& q, const Vec3& v);

void integrateOrientation(Quaternion& q, const Vec3& angVel, float dt);

void quatToMatrix(const Quaternion& q, float M[3][3]);

Quaternion quatFromRotationMatrix(float m[3][3]);
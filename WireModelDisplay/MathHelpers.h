#pragma once

#include <Arduino.h>
#include <math.h>
#include "esp_dsp.h"   // ESP-DSP library

struct __attribute__((aligned(16))) Quaternion {
    float w = 1, x = 0, y = 0, z = 0;

    // Addition
    Quaternion operator+(const Quaternion& rhs) const {
        return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z};
    }

    // Subtraction
    Quaternion operator-(const Quaternion& rhs) const {
        return {w - rhs.w, x - rhs.x, y - rhs.y, z - rhs.z};
    }

    // Scalar multiplication
    Quaternion operator*(float s) const {
        return {w * s, x * s, y * s, z * s};
    }

    // Scalar division
    Quaternion operator/(float s) const {
        return {w / s, x / s, y / s, z / s};
    }

    // Quaternion multiplication (Hamilton product)
    Quaternion operator*(const Quaternion& rhs) const {
        return {
            w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z,
            w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y,
            w*rhs.y - x*rhs.z + y*rhs.w + z*rhs.x,
            w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w
        };
    }

    // Compound assignment versions
    Quaternion& operator+=(const Quaternion& rhs) {
        w += rhs.w; x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }
    Quaternion& operator-=(const Quaternion& rhs) {
        w -= rhs.w; x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }
    Quaternion& operator*=(float s) {
        w *= s; x *= s; y *= s; z *= s;
        return *this;
    }
    Quaternion& operator/=(float s) {
        w /= s; x /= s; y /= s; z /= s;
        return *this;
    }
    Quaternion& operator*=(const Quaternion& rhs) {
        *this = *this * rhs;
        return *this;
    }
};

// Allow scalar * Quaternion
inline Quaternion operator*(float s, const Quaternion& q) {
    return {q.w * s, q.x * s, q.y * s, q.z * s};
}


struct __attribute__((aligned(16))) Vec3 {
    float x = 0, y = 0, z = 0;

    // Addition
    Vec3 operator+(const Vec3& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    // Subtraction
    Vec3 operator-(const Vec3& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    // Scalar multiplication
    Vec3 operator*(float s) const {
        return {x * s, y * s, z * s};
    }

    // Scalar division
    Vec3 operator/(float s) const {
        return {x / s, y / s, z / s};
    }

    //Vec3 operator*(const Vec3& rhs) const {
    //  return dot(&x, &rhs.x);
    //}

    // Compound assignment versions
    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }
    Vec3& operator*=(float s) {
        x *= s; y *= s; z *= s;
        return *this;
    }
    Vec3& operator/=(float s) {
        x /= s; y /= s; z /= s;
        return *this;
    }
};
// Allow scalar * Vec3
inline Vec3 operator*(float s, const Vec3& v) {
    return {v.x * s, v.y * s, v.z * s};
}

struct Vec2 { int x = 0, y = 0; };


float randFloat(float min, float max);

// -----------------------------
// Math helpers
// -----------------------------

float fastSin(const float x);

float fastCos(const float x);

float fastSqrt(const float x);

float fastInvSqrt(const float x);

float dot(const float* u, const float* v, const int dim = 3);


float magnitude(const float* in, const int dim = 3);

Vec3 normalize(const Vec3& v);

Vec3 cross(const Vec3& u, const Vec3& v);

Quaternion quatIdentity();

Quaternion quatConjugate(const Quaternion& q);

Quaternion quatNormalize(const Quaternion& q);

Quaternion quatFromAxisAngle(const float ax, const float ay, const float az, const float angle);

Vec3 rotateVector(const Quaternion& q, const Vec3& v);

void integrateOrientation(Quaternion& q, const Vec3& angVel, float dt);

void quatToMatrix(const Quaternion& q, float M[3][3]);

Quaternion quatFromRotationMatrix(float m[3][3]);

Quaternion quatAlign(const Vec3& from, const Vec3& to);
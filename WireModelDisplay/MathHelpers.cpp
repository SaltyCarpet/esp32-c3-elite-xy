#include <Arduino.h>
#include <math.h>
#include "esp_dsp.h"   // ESP-DSP library
#include "MathHelpers.h"
#include "ErrorHandler.h"

// -----------------------------
// Math helpers
// -----------------------------

// Detect ESP32-S3 (Arduino core defines ARDUINO_ESP32S3)
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3)
  #include "esp_dsp.h"

  float fastSin(float x) {
    return sinf(x); //dsps_sin_f32_ansi(x);
  }

  float fastCos(float x) {
    return cosf(x); //dsps_cos_f32_ansi(x);
  }

  float fastSqrt(float x) {
    return dsps_sqrtf_f32_ansi(x);
  }

  float fastInvSqrt(float x)
  {
    return 1.0f/fastSqrt(x);
  }

  float dot(const float* u, const float* v, int dim) {
    // ESP-DSP dot product
    float res = 0;
    esp_err_t err = dsps_dotprod_f32_aes3(u, v, &res, dim);
    if(err != ESP_OK) setStatus(STATUS_ERROR);
    return res;
  }

#else
  // Fallback for other ESP32 variants or generic Arduino
  float fastSin(float x) {
    return sinf(x);
  }

  float fastCos(float x) {
    return cosf(x);
  }

  float fastSqrt(float x) {
    return sqrtf(x);
  }

  float fastInvSqrt(float x) {
    float half = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    float y = *(float*)&i;
    return y * (1.5f - half * y * y);
  }

  float dot(const float* u, const float* v, int dim) {
    float res = 0;
    for(int i = 0; i < dim; i++)
    {
      res += u[i] * v[i];
    }
    return res;
  }

#endif


Vec3 normalize(const Vec3& v) {
    float mag = fastInvSqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (mag > 1e+6f) return {0,0,0};
    return {v.x*mag, v.y*mag, v.z*mag};
}

Vec3 cross(const Vec3& u, const Vec3& v) {
  // No direct intrinsic, but we can inline
  return {
    u.y * v.z - u.z * v.y,
    u.z * v.x - u.x * v.z,
    u.x * v.y - u.y * v.x
  };
}

Quaternion quatIdentity() {
  return {1.0f, 0.0f, 0.0f, 0.0f};
}

Quaternion quatConjugate(const Quaternion& q) {
  return { q.w, -q.x, -q.y, -q.z };
}

Quaternion quatNormalize(const Quaternion& q) {
  float vals[4] = { q.w, q.x, q.y, q.z };
  float mag2 = dot(vals, vals, 4);
  if (mag2 < 1e-8f) return quatIdentity();
  float invmag = fastInvSqrt(mag2);
  return { q.w*invmag, q.x*invmag, q.y*invmag, q.z*invmag };
}

Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
  // Compiler will fuse multiply-adds with -Ofast
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

Quaternion quatFromAxisAngle(float ax, float ay, float az, float angle) {
  float half = 0.5f * angle;
  float s = fastSin(half);
  float c = fastCos(half);
  return quatNormalize({ c, ax*s, ay*s, az*s });
}

Vec3 rotateVector(const Quaternion& q, const Vec3& v) {
  // v' = q * (0,v) * q^-1
  Quaternion vq = {0, v.x, v.y, v.z};
  Quaternion qConj = quatConjugate(q);
  Quaternion rq = quatMultiply(quatMultiply(q, vq), qConj);
  return { rq.x, rq.y, rq.z };
}

void integrateOrientation(Quaternion& q, const Vec3& angVel, float dt) {
  float vals[3] = { angVel.x, angVel.y, angVel.z };
  float mag2 = dot(vals, vals, 3);
  if (mag2 > 1e-12f) {
    float invmag = fastInvSqrt(mag2);
    float half = 0.5f * dt / invmag;
    float s = fastSin(half) * invmag;
    float c = fastCos(half);
    Quaternion dq = { c, angVel.x*s, angVel.y*s, angVel.z*s };
    q = quatNormalize(quatMultiply(q, dq));
  }
}

void quatToMatrix(const Quaternion& q, float M[3][3]) {
  float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
  float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
  float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

  M[0][0] = 1 - 2*(yy + zz);
  M[0][1] = 2*(xy - wz);
  M[0][2] = 2*(xz + wy);

  M[1][0] = 2*(xy + wz);
  M[1][1] = 1 - 2*(xx + zz);
  M[1][2] = 2*(yz - wx);

  M[2][0] = 2*(xz - wy);
  M[2][1] = 2*(yz + wx);
  M[2][2] = 1 - 2*(xx + yy);
}

// --- Convert rotation matrix to quaternion ---
Quaternion quatFromRotationMatrix(float m[3][3]) {
    Quaternion q;
    float trace = m[0][0] + m[1][1] + m[2][2];
    if (trace > 0.0f) {
        float s = fastInvSqrt(trace + 1.0f) / 2.0f;
        q.w = 0.25f / s;
        q.x = (m[2][1] - m[1][2]) * s;
        q.y = (m[0][2] - m[2][0]) * s;
        q.z = (m[1][0] - m[0][1]) * s;
    } else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
        float s = fastInvSqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) / 2.0f;
        q.w = (m[2][1] - m[1][2]) * s;
        q.x = 0.25f / s;
        q.y = (m[0][1] + m[1][0]) * s;
        q.z = (m[0][2] + m[2][0]) * s;
    } else if (m[1][1] > m[2][2]) {
        float s = fastInvSqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) / 2.0f;
        q.w = (m[0][2] - m[2][0]) * s;
        q.x = (m[0][1] + m[1][0]) * s;
        q.y = 0.25f / s;
        q.z = (m[1][2] + m[2][1]) * s;
    } else {
        float s = fastInvSqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) / 2.0f;
        q.w = (m[1][0] - m[0][1]) * s;
        q.x = (m[0][2] + m[2][0]) * s;
        q.y = (m[1][2] + m[2][1]) * s;
        q.z = 0.25f / s;
    }
    return q;
}
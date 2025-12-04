#include <Arduino.h>
#include <math.h>
#include "esp_dsp.h"   // ESP-DSP library
#include "MathHelpers.h"
#include <random>

// Random helper
float randFloat(float min, float max) {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

// -----------------------------
// Math helpers
// -----------------------------

// Detect ESP32-S3 (Arduino core defines ARDUINO_ESP32S3)
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3)
  #include "esp_dsp.h"

  float fastSin(const float x) {
    return sinf(x); //dsps_sin_f32_ansi(x);
  }

  float fastCos(const float x) {
    return cosf(x); //dsps_cos_f32_ansi(x);
  }

  float fastSqrt(const float x) {
    return dsps_sqrtf_f32_ansi(x);
  }

  float fastInvSqrt(const float x)
  {
    return 1.0f/fastSqrt(x);
  }

  float dot(const float* u, const float* v, const int dim) {
    // ESP-DSP dot product
    float res = 0;
    esp_err_t err = dsps_dotprod_f32_aes3(u, v, &res, dim);
    if(err != ESP_OK) setStatus(STATUS_ERROR);
    return res;
  }

#else
  // Fallback for other ESP32 variants or generic Arduino
  float fastSin(const float x) {
    return sinf(x);
  }

  float fastCos(const float x) {
    return cosf(x);
  }

  float fastSqrt(const float x) {
    return sqrtf(x);
  }

  float fastInvSqrt(const float x) {
    float half = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    float y = *(float*)&i;
    return y * (1.5f - half * y * y);
  }

  float dot(const float* u, const float* v, const int dim) {
    float res = 0;
    for(int i = 0; i < dim; i++)
    {
      res += u[i] * v[i];
    }
    return res;
  }

#endif


float magnitude(const float* in, const int dim) {
  return fastSqrt(dot(in, in, dim));
}

Vec3 normalize(const Vec3& v) {
  float mag = magnitude(&v.x);
  if (mag < 1e-8f) return {0,0,0};
  return v/mag;
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
  float mag = magnitude(&q.w, 4);
  if (mag < 1e-6f) return quatIdentity();
  return q/mag;
}

Quaternion quatFromAxisAngle(const float ax, const float ay, const float az, const float angle) {
  float half = 0.5f * angle;
  float s = fastSin(half);
  float c = fastCos(half);
  return quatNormalize({ c, ax*s, ay*s, az*s });
}

Vec3 rotateVector(const Quaternion& q, const Vec3& v) {
  // v' = q * (0,v) * q^-1
  Quaternion vq = {0, v.x, v.y, v.z};
  Quaternion qConj = quatConjugate(q);
  Quaternion rq = ((q * vq) * qConj);
  return { rq.x, rq.y, rq.z };
}

void integrateOrientation(Quaternion& q, const Vec3& angVel, float dt) {
  float mag = magnitude(&angVel.x);
  if (mag > 1e-8f) return;
  float half = 0.5f * dt * mag;
  float s = fastSin(half) / mag;
  float c = fastCos(half);
  Quaternion dq = { c, angVel.x*s, angVel.y*s, angVel.z*s };
  q = quatNormalize(q * dq);
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
        float s = fastSqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m[2][1] - m[1][2]) / s;
        q.y = (m[0][2] - m[2][0]) / s;
        q.z = (m[1][0] - m[0][1]) / s;
    } else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
        float s = fastSqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
        q.w = (m[2][1] - m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (m[0][1] + m[1][0]) / s;
        q.z = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        float s = fastSqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
        q.w = (m[0][2] - m[2][0]) / s;
        q.x = (m[0][1] + m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (m[1][2] + m[2][1]) / s;
    } else {
        float s = fastSqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
        q.w = (m[1][0] - m[0][1]) / s;
        q.x = (m[0][2] + m[2][0]) / s;
        q.y = (m[1][2] + m[2][1]) / s;
        q.z = 0.25f * s;
    }
    return q;
}

Quaternion quatAlign(const Vec3& from, const Vec3& to) {
    Vec3 f = normalize(from);
    Vec3 t = normalize(to);

    float dotVal = fmaxf(-1.0f, fminf(1.0f, dot(&f.x, &t.x)));
    Vec3 axis = cross(f, t);

    if (magnitude(&axis.x) < 1e-8f) {
        // Vectors are parallel
        if (dotVal > (1.0f - 1e-8f)) {
            return {1,0,0,0}; // identity
        } else {
            // 180° rotation around any perpendicular axis
            Vec3 ortho = (fabs(f.x) > fabs(f.z)) ? Vec3{-f.y, f.x, 0} : Vec3{0, -f.z, f.y};
            ortho = normalize(ortho);
            return quatFromAxisAngle(ortho.x, ortho.y, ortho.z, M_PI);
        }
    }

    axis = normalize(axis);
    float angle = acosf(dotVal);
    return quatFromAxisAngle(axis.x, axis.y, axis.z, angle);
}
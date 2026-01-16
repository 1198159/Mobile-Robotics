#ifndef CONTROLLER_H
#define CONTROLLER_H

/* ===== Constants / Tunables ===== */
constexpr float kPa = 1.0f;
constexpr float kIa = 0.0f;
constexpr float kDa = 0.0f;

constexpr float maxLinSpeed = 100.0f;     // mm/s
constexpr float maxAngSpeed = 1.0f;       // rad/s
constexpr float targetDistance = 127.0f;  // mm
constexpr float maxAngSpeedIntegral = 0.1f;

/* ===== State variables (must NOT be constexpr) ===== */
inline float angSpeedIntegral = 0.0f;
inline float errorDistLast = 0.0f;
inline bool isReady = false;

/* ===== Utility ===== */
inline float clamp(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

/* ===== API ===== */
void calculate(
  float dt,
  float linDistance,
  float angDistance,
  float* linOutput,
  float* angOutput
);

#endif

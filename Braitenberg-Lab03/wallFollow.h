/*
Controller for following a wall or two

clamp is a utility function
calculate does the controller calculations

*/
#ifndef CONTROLLER_H
#define CONTROLLER_H

/* ===== Constants / Tunables ===== */
constexpr float kPl = 0.007f;
constexpr float kIl = 0.0f;
constexpr float kDl = 0.001f;

constexpr float kPa = 0.1f;
constexpr float kIa = 0.0f;
constexpr float kDa = 0.001f;

constexpr float maxLinSpeed = 200.0f;     // mm/s
constexpr float aroundOutsideCornerLinSpeed = 50.0f;     // mm/s. Using sonar to go around an outside corner
constexpr float maxStraightAngSpeed = 0.3f;       // rad/s. Using lidar to go approx straight along a wall
constexpr float maxOutsideCornerAngSpeed = 2.0f;       // rad/s. Using sonar to go around an outside corner
constexpr float targetDistance = 100.0f;  // mm
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
  float errorDist,
  float angDistance,
  float* linOutput,
  float* angOutput
);

#endif

#include "wallFollow.h"

void calculate(
  float dt,
  float linDistance,
  float angDistance,
  float* linOutput,
  float* angOutput
) {
  float errorDist = linDistance - targetDistance;

  angSpeedIntegral = clamp(
    angSpeedIntegral + errorDist,
    -maxAngSpeedIntegral,
    maxAngSpeedIntegral
  );

  *angOutput = kPl * errorDist + kIl * angSpeedIntegral - kPa * angDistance;

  if (isReady && dt > 0.0f) {
    *angOutput += kDl * (errorDist - errorDistLast) / dt;
  } else {
    isReady = true;
  }

  errorDistLast = errorDist;

  if (linDistance > 80.0f) {
    *linOutput = maxLinSpeed;
  } else {
    *linOutput = 0.0f;
  }
}

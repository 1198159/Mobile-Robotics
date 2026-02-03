#include "goToAngleController.h"
#include "wallFollow.h"
#include <Arduino.h>//for pi



void calculateGoToAngle(
  float dt,
  float angTarget,
  float angCurrent,
  float* angOutput
) {
  float angError = angCurrent - angTarget;
  if(angError<-PI) angError+=2*PI;
  if(angError>PI) angError-=2*PI;

  angErrorIntegral = clamp(
    angErrorIntegral + angError,
    -maxAngErrorIntegral,
    maxAngErrorIntegral
  );

  *angOutput = kPgta * angError + kIgta * angErrorIntegral;

  if (gtaIsReady && dt > 0.0f) {
    *angOutput += kDgta * (angError - angErrorLast) / dt;
  } else {
    gtaIsReady = true;
  }

  angErrorLast = angError;

}

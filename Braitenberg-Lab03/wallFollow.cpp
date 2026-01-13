#include <wallFollow.h>
#include <cmath>

float[] calculate(float dt, float linDistance, float angDistance){
  //calculate angular velo output
  float errorDist = linDistance - targetDistance;
  angSpeedIntegral = std::clamp(angSpeedIntegral + errorDist, -maxAngSpeedIntegral, maxAngSpeedIntegral);

  float angOutput = kPa*errorDist + kIa * angSpeedIntegral;
  if (errorDistLast != NaN) {
    angOutput += (errorDist - errorDistLast) / dt;
  }

  errorDistLast = errorDist;
  //calculate linvel output
  if (errorDist > 80) linOutput = 100;
  else linoutput = 0;



  return [linOutput, angOutput];

}
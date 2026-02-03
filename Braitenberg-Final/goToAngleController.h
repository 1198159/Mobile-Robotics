#ifndef ACONTROLLER_H
#define ACONTROLLER_H


/* ===== Constants / Tunables ===== */
constexpr float kPgta = 2.7f;
constexpr float kIgta = 0.1f;
constexpr float kDgta = 0.5f;

constexpr float maxAngErrorIntegral = 0.1f;

/* ===== State variables (must NOT be constexpr) ===== */
inline float angErrorIntegral = 0.0f;
inline float angErrorLast = 0.0f;
inline bool gtaIsReady = false;


/* ===== API ===== */
void calculateGoToAngle(
  float dt,
  float angTarget,
  float angCurrent,
  float* angOutput
);

#endif

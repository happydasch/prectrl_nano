#ifndef _THROTTLE_H
#define _THROTTLE_H

#include <const.h>

int throttle_reading = 0;                             // current throttle reading
int throttle_low = THROTTLE_MIN;                      // lowest throttle measurement value (will be set at startup)
double throttle = 0.0;                                // current throttle value
double throttle_prev = 0.0;                           // previous throttle value
double throttle_avg = 0.0;                            // average throttle value

/**
 * @brief Sets the initial throttle low value
 *
 */
void init_throttle_low() {
  throttle_low = analogRead(PIN_I_THROTTLE);
  for (int i = 0; i < READINGS_LOW; i++) {
    throttle_low -= throttle_low / READINGS_LOW;
    throttle_low += analogRead(PIN_I_THROTTLE) / READINGS_LOW;
    delay(50);
  }
  throttle_low = min(throttle_low, THROTTLE_MIN);
}

/**
 * @brief Initializes the throttle sensor
 *
 */
void init_throttle() {
    pinMode(PIN_I_THROTTLE, INPUT);       // input throttle
    init_throttle_low();
}

/**
 * @brief Reads in the throttle value
 *
 */
void read_throttle() {
  throttle_reading = analogRead(PIN_I_THROTTLE);
}

/**
 * @brief Updates the throttle values
 *
 * @param throttle_new
 */
void update_throttle(int throttle_new) {
  throttle_prev = throttle;
  if (throttle_new && throttle_low > throttle_new + SAFE_VALUE) {
    throttle_low = throttle_new + SAFE_VALUE;
  }
  throttle = map(throttle_new, throttle_low, THROTTLE_MAX, 0, 1023);
  throttle = constrain(throttle, 0, 1023);
  if (throttle_prev && !throttle) {
    // reset average if throttle is 0
    throttle_avg = 0;
  }
  if (!throttle) {
    throttle_avg = throttle;
  }
  throttle_avg -= throttle_avg / READINGS_AVG_THROTTLE;
  throttle_avg += throttle / READINGS_AVG_THROTTLE;
}

#endif

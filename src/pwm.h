#ifndef _PWM_H
#define _PWM_H

#include <const.h>
#include <timer.h>
#include <torque.h>
#include <throttle.h>

int pwm_output = 0;                                   // new pwm output
int pwm_output_prev = 0;                              // last pwm output

void init_pwm_output() {
  pinMode(PIN_O_PWM, OUTPUT);
  timer1.pwm(PIN_O_PWM, 0);
}

/**
 * @brief Updates the pwm output signal
 *
 */
void update_pwm_output() {
  pwm_output = 0;

  // torque based throttle
  if (pedaling) {
    pwm_output = torque_pid_out;
  }
  // throttle override
  if (throttle_avg > 0 && throttle_avg > pwm_output) {
    pwm_output = throttle_avg;
  }
  // ensure pwm output is within bounds
  pwm_output = constrain(pwm_output, 0, 1023);
  // map pwm output value to throttle min and max values
  pwm_output = map(pwm_output, 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
  if (pwm_output != pwm_output_prev) {
    // update pwm duty cycle
    timer1.pwm(PIN_O_PWM, pwm_output);
    pwm_output_prev = pwm_output;
  }
  // reset the counter for signal update
  signal_counter = 0;
}

#endif

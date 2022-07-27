#ifndef TORQUE_H
#define TORQUE_H

#include <PID_v1_nano.h>
#include <const.h>
#include <timer.h>
#include <serial.h>

volatile bool interrupt_pas = false;                  // flag for pas interrupt

int torque_reading = 0;                               // latest torque reading
int torque_low = TORQUE_MIN;                          // lowest torque measurement value (will be set at startup)
int torque_pas_ticks = 0;                             // current pas duration in ticks
double torque = 0.0;                                  // current torque measurement
double torque_prev = 0.0;                             // previous torque measurement
double torque_avg = 0.0;                              // avg torque calculated
double cadence = 0.0;                                 // current cadence
double cadence_prev = 0.0;                            // previous cadence reading
double cadence_avg = 0.0;                             // cadence average
double power_input = 0.0;                             // current power provided by driver
double power_input_prev = 0.0;                        // previous power provided by driver
double power_input_avg = 0.0;                         // average power provided by driver
bool pedaling = false;                                // flag for pedaling
double factor[] = {0.0, 0.75, 1.0, 1.25, 1.5, 2.0};   // factors for support levels 0-5

double torque_k_p = 1.03, torque_k_i = 0.5, torque_k_d = 0;
double torque_pid_in = 0, torque_pid_out = 0, torque_pid_set = 0;
PID pid_torque(&torque_pid_in, &torque_pid_out, &torque_pid_set,
               torque_k_p, torque_k_i, torque_k_d, REVERSE);

/**
 * @brief Function for interrupt calls from pas
 *
 */
void interrupt_pas_pulse() {
  if (digitalRead(PIN_I_PAS) != HIGH) {
    return;
  }
  torque_pas_ticks = tick_pas_counter;
  tick_pas_counter = 0;
  interrupt_pas = true;
}

/**
 * @brief Initializes the torque PID controller
 *
 */
void init_torque_pid() {
  pid_torque.SetMode(AUTOMATIC);
  pid_torque.SetOutputLimits(0, 1023);
  pid_torque.SetSampleTime(10);  // 10ms
}

/**
 * @brief Sets the initial torque low value
 *
 */
void init_torque_low() {
  torque_low = analogRead(PIN_I_TORQUE);
  for (int i = 0; i < READINGS_LOW; i++) {
    torque_low -= torque_low / READINGS_LOW;
    torque_low += analogRead(PIN_I_TORQUE) / READINGS_LOW;
    delay(50);
  }
  torque_low = min(torque_low, TORQUE_MIN);
}

/**
 * @brief Initializes the torque sensor
 *
 */
void init_torque() {
  pinMode(PIN_I_PAS, INPUT_PULLUP);     // input pas
  pinMode(PIN_I_TORQUE, INPUT);         // input torque
  attachInterrupt(digitalPinToInterrupt(PIN_I_PAS), interrupt_pas_pulse, RISING);
  init_torque_low();
  init_torque_pid();
}

/**
 * @brief Reads in the torque value
 *
 */
void read_torque() {
  torque_reading = analogRead(PIN_I_TORQUE);
  interrupt_pas = false;
}

/**
 * @brief Updates the torque values
 *
 */
void update_torque_values(double torque_new) {
  torque_prev = torque;
  if (torque_new && torque_low > torque_new) {
    torque_low = torque_new;
  }
  torque = torque_new;
  if (!torque_avg || !torque) {
    torque_avg = torque;
  }
  torque_avg -= torque_avg / READINGS_AVG_TORQUE;
  torque_avg += torque / READINGS_AVG_TORQUE;
}

/**
 * @brief Updates the cadence values
 *
 */
void update_cadence_values() {
  // update cadence in rpm
  cadence_prev = cadence;
  cadence = 0;
  if (torque_pas_ticks > 0) {
    cadence = PAS_FACTOR / torque_pas_ticks;
  } else {
    cadence = 0;
    cadence_avg = 0;
  }
  cadence_avg -= cadence_avg / READINGS_AVG_CADENCE;
  cadence_avg += cadence / READINGS_AVG_CADENCE;
}

/**
 * @brief Updates the power input values
 *
 */
void update_power_input_values() {
  // update power input in nm
  //
  // power = 2 * pi * cadence_in_rpm * torque_in_nm / 60s
  // multiplication constant for SEMPU and T9 is approx. 0.33Nm/count
  //
  //   2 * pi / 60 * cadence * torque_nm
  // = 2 * pi / 60 = 2 * 3.14159 / 60
  // = 0.1047196667
  // = 0.1047196667 * cadence * torque_nm
  double torque_nm = (torque - torque_low) * TORQUE_FACTOR_NM;
  power_input_prev = power_input;
  power_input = 0.1047196667 * cadence * torque_nm;
  power_input_avg -= power_input_avg / READINGS_AVG_POWER_INPUT;
  power_input_avg += power_input / READINGS_AVG_POWER_INPUT;
}

/**
 * @brief Updates the torque, cadence and power input values
 *
 * @param torque_new new torque reading
 */
void update_torque(double torque_new) {
  update_torque_values(torque_new);
  update_cadence_values();
  update_power_input_values();
}

/**
 * @brief Updates the state of pas
 *
 */
void update_pas_state() {
  if (tick_pas_counter >= PAS_TIMEOUT) {
    pedaling = false;
    if (tick_timeout_counter >= PAS_PULSE_TIMEOUT) {
      tick_timeout_counter = 0;
      torque_pas_ticks = 0;
      update_torque(0);
    }
  } else {
    pedaling = true;
    tick_timeout_counter = 0;
  }
}

/**
 * @brief Upates the set_point value for the PID controller
 *
 */
void set_point() {
  double torque_diff = torque - torque_avg;

  // set pid in for torque
  torque_pid_in = torque_diff;
  torque_pid_in *= factor[level_current];

  // shrink pid if not pedaling or no torque is provided
  if (!pedaling || torque_diff <= 0) {
    if (PID_RESET_ON_NO_TORQUE) {
      pid_torque.ResetIntegral();
    } else {
      pid_torque.ShrinkIntegral();
    }
  }

  // reset pid if brake is pressed
  if (brake_current) {
    if (PID_RESET_ON_BRAKE) {
      pid_torque.ResetIntegral();
    } else {
      pid_torque.ShrinkIntegral();
    }
  }

  // compute torque_pid_out
  pid_torque.Compute();
}

#endif

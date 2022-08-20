#ifndef _CONST_H
#define _CONST_H

// pin definitions
#define PIN_O_PWM 10                                  // D10 timer1
#define PIN_O_LGHT 3                                  // D3  Light request from controller
#define PIN_I_PAS 2                                   // D2
#define PIN_I_TORQUE A0                               // A0
#define PIN_I_THROTTLE A1                             // A1
#define PIN_D_RX 5                                    // D5  RX from display
#define PIN_C_RX 6                                    // D6  RX from controller

// misc constants
#define TICK_INTERVAL 100                             // time interval between two timer interrupts in µs (100 == 0.1ms, 10khz)
#define SERIAL_UPDATE 50000/TICK_INTERVAL             // interval for reading serial values (in ticks: µs/tick_interval)
#define SERIAL_TIMEOUT 1000000/TICK_INTERVAL          // timeout of serial in serial update count (in µs/tick_interval)
#define SIGNAL_UPDATE 10000/TICK_INTERVAL             // interval for updating signal values (in ticks: µs/tick_interval)
#define PAS_TIMEOUT 600000/TICK_INTERVAL             // timeout for PWM with no PAS signal (in ticks: µs/tick_interval)
#define PAS_PULSE_TIMEOUT 20000/TICK_INTERVAL         // timeout for single PAS pulses (in ticks: µs/tick_interval)
#define PAS_FACTOR 60000000/TICK_INTERVAL/PAS_PULSES  // factor to convert PAS pulses to RPM (60.000.000ms == 60s == 1min / PAS pulses) / 2
                                                      // duration for 1 pas pulse in ticks: µs/tick_interval
                                                      // duration for 1 pas pulse in ticks: µs/tick_interval
#define THROTTLE_MIN 250                              // min throttle
#define THROTTLE_MAX 750                              // max throttle
#define TORQUE_MIN 300                                // min torque = 0 (1,5V -> needs to be calibrated)
#define TORQUE_MAX 450                                // max torque (around 3,0V -> needs to be calibrated)
#define TORQUE_FACTOR_NM 0.33                         // Torque reading to Nm factor
#define PAS_PULSES 36                                 // number of pulses per revolution for PAS signal
#define PID_RESET_ON_NO_TORQUE false                  // reset PID on no torque / no pedaling
#define PID_RESET_ON_BRAKE true                       // reset PID on brake
#define READINGS_LOW 5                                // number of readings to average low values
#define READINGS_AVG_TORQUE PAS_PULSES                // number of readings to average for torque (2*PAS_PULSES)
#define READINGS_AVG_POWER_INPUT (PAS_PULSES/2)       // number of readings to average for power input (PAS_PULSES/2)
#define READINGS_AVG_CADENCE (PAS_PULSES/2)           // number of readings to average for cadence (PAS_PULSES/2)
#define READINGS_AVG_THROTTLE 4                       // number of readings to average for throttle
#define SAFE_VALUE 5                                  // Safe value for throttle and torque idle when calculating low

// debug mode (for Serial output 115200)
#define DEBUG false

#endif

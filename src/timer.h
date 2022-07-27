#ifndef _TIMER_H
#define _TIMER_H

#include <const.h>
#include <TimerOne.h>

volatile unsigned int tick_pas_counter = 0xFFFF;      // counter for ticks between two pas pulses
volatile unsigned int tick_timeout_counter = 0;       // counter for timeout between two pas pulses
volatile unsigned int signal_counter = 0;             // counter for signal updates
volatile unsigned int serial_counter = 0;             // counter for serial updates

extern TimerOne timer1;                               // timer for pwm and updates

/**
 * @brief Function for interrupt calls every tick
 *
 */
void interrupt_time_tick() {
  if (tick_pas_counter < 0xFFFF) {
    tick_pas_counter++;
  }
  if (tick_timeout_counter < 0xFFFF) {
    tick_timeout_counter++;
  }
  if (signal_counter < 0xFFFF) {
    signal_counter++;
  }
  if (serial_counter < 0xFFFF) {
    serial_counter++;
  }
}

void init_timer() {
  timer1.initialize(TICK_INTERVAL);
  timer1.attachInterrupt(interrupt_time_tick);
}

#endif

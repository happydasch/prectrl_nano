#include <const.h>
#include <timer.h>
#include <serial.h>
#include <throttle.h>
#include <torque.h>
#include <pwm.h>

/**
 * @brief Setup method
 *
 */
void setup() {
  // start serial port
  #if DEBUG == true
  Serial.begin(115200);
  #endif

  // initialize timer1
  init_timer();

  // software serial initialization to listen for communication
  // between ctrl <-> display
  init_software_serial();

  // initialize sensors
  init_torque();
  init_throttle();

  // initialize output signal
  init_pwm_output();
}

/**
 * @brief Main loop
 *
 */
void loop() {
  bool update_state = false;

  // serial update part
  if (serial_counter >= SERIAL_UPDATE) {
    read_software_serial();
  }

  // pas pulse part, controlled by pas interrupt
  if (interrupt_pas) {
    read_torque();
    update_torque(torque_reading);
  }

  // signal update part
  if (signal_counter >= SIGNAL_UPDATE) {
    read_throttle();
    update_throttle(throttle_reading);
    update_state = true;
  }

  // state and output update part
  if (update_state) {
    set_point();
    update_pas_state();
    update_light_state();
    update_pwm_output();
  }
}

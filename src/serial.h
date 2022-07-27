#ifndef _SERIAL_H
#define _SERIAL_H

#include <SoftwareSerial.h>
#include <const.h>
#include <timer.h>

uint8_t serial_buffer[13];                            // buffer for received data (used for both sides)
byte serial_pos = 0;                                  // counter for recieved bytes

// serial readings (see reset_serial_values() for initialization)
int level_current;                                    // current support level (default 3)
int light_current;                                    // current light status
double wheel_size_current;                            // current wheel size
int max_speed_current;                                // current max speed
int power_current;                                    // current power
bool brake_current;                                   // current brake status
double speed_current;                                 // calculated speed

SoftwareSerial display_serial = SoftwareSerial(PIN_D_RX, -1);
SoftwareSerial controller_serial = SoftwareSerial(PIN_C_RX, -1);

/**
 * @brief Resets serial values to default values
 *
 */
void reset_serial_values() {
  level_current = 3;
  light_current = 0;
  wheel_size_current = 0.0;
  max_speed_current = 0;
  speed_current = 0;
  power_current = 0;
  brake_current = false;
}

/**
 * @brief Initializes the software serial ports
 *
 */
void init_software_serial() {
  pinMode(PIN_O_LGHT, OUTPUT);          // output light request
  controller_serial.begin(9600);
  display_serial.begin(9600);
  reset_serial_values();
}

/**
 * @brief Processes the serial message recieved from display
 *
 */
void process_display_message() {
  int curr_light = serial_buffer[1] & 0x80;  // 128: 0b10000000
  int curr_level = serial_buffer[1] & 0x07;  // 7: 0b00000111
  int max_speed = 10 + (((serial_buffer[2] & 0xF8) >> 3) | (serial_buffer[4] & 0x20));
  int wheel_size = ((serial_buffer[4] & 0xC0) >> 6) | ((serial_buffer[2] & 0x07) << 2);

  light_current = curr_light;
  level_current = curr_level;
  max_speed_current = max_speed;
  switch (wheel_size) {
    case 0x12: // 6''
    wheel_size_current = 468.75;
    break;

    case 0x0A: // 8''
    wheel_size_current = 628.47;
    break;

    case 0x0E: // 10''
    wheel_size_current = 788.19;
    break;

    case 0x02: // 12''
    wheel_size_current = 947.91;
    break;

    case 0x06: // 14''
    wheel_size_current = 1107.64;
    break;

    case 0x00: // 16''
    wheel_size_current = 1267.36;
    break;

    case 0x04: // 18''
    wheel_size_current = 1427.08;
    break;

    case 0x08: // 20''
    wheel_size_current = 1576.39;
    break;

    case 0x0C: // 22''
    wheel_size_current = 1743.05;
    break;

    case 0x10: // 24''
    wheel_size_current = 1895.83;
    break;

    case 0x18: // 700c
    wheel_size_current = 2173.61;
    break;

    case 0x1C: // 28''
    wheel_size_current = 2194.44;
    break;

    case 0x1E: // 29''
    wheel_size_current = 2250;
    break;

    case 0x14: // 26''
    default: // 26''
    wheel_size_current = 2060;
    break;
  }
}

/**
 * @brief Processes the serial message recieved from controller
 *
 */
void process_controller_message() {
  double rotation_ms = (serial_buffer[3] * 0xFF) + serial_buffer[4];
  int power = serial_buffer[8] * 13;  // 13W per unit
  bool brake = bool(serial_buffer[7] & 0x20);

  power_current = power;
  brake_current = brake;
  speed_current = 3600 * (wheel_size_current / rotation_ms / 1000);
  if (rotation_ms >= 6895) {
    // if a wheel rotation takes over 6 seconds the speed
    // is set to 0
    // the highest value returned for rotation_ms is 6895
    speed_current = 0;
  }
}

/**
 * @brief Processes current byte from display
 *
 * @param reading current byte
 * @return bool true if message is complete else false
 */
bool process_display_serial(uint8_t reading) {
  /*
  S-LCD3 to S12SN communication protocol.

  Packet consist of 13 bytes. 9600 baud, 8-n-1, byte-by-byte
  B0	B1	B2	B3	B4	B5	B6	B7	B8	B9	B10	B11	B12
  (e.g: 12 0 149 160 41 102 18 74 4 20 0 50 14)
  */
  serial_buffer[serial_pos] = reading;
  serial_pos += 1;
  if (serial_pos == 13 && serial_buffer[12] == 0xE) {
    process_display_message();
    return true;
  }
  if (serial_pos >= 13 || reading == 0xE) {
    serial_pos = 0;
  }
  return false;
}

/**
 * @brief Processes current byte from controller
 *
 * @param reading current byte
 * @return bool true if message is complete else false
 */
bool process_controller_serial(byte reading) {
  /*
  S12SN to LCD3 communication protocol.

  Packet consist of 12 bytes. 9600 baud, 8-n-1, byte-by-byte, no separators
  B0	B1	B2	B3	B4	B5	B6	B7	B8	B9	B10	B11
  (e.g: 65 16 48 0 139 0 164 2 13 0 0 0)
  */
  if (reading == 0x41) {
    serial_pos = 0;
  }
  serial_buffer[serial_pos] = reading;
  serial_pos += 1;
  if (serial_pos >= 12 && serial_buffer[0] == 0x41) {
    process_controller_message();
    return true;
  }
  return false;
}

/**
 * @brief Reads display and controller messages from serial
 *
 * This function reads a complete message from a serial port
 * and then switches to another. After a whole message is recieved
 * it is being processed.
 */
bool read_software_serial() {
  uint8_t reading;
  bool complete;

  // if a timeout occurs then reset the serial values
  if (serial_counter > SERIAL_TIMEOUT) {
    reset_serial_values();
  }

  // read bytes from currently selected serial port
  // until message is complete
  if (display_serial.isListening()) {
    // read message from display serial
    if (!display_serial.available()) {
      return false;
    }
    while (display_serial.available()) {
      reading = display_serial.read();
      complete = process_display_serial(reading);
      if (complete) {
        break;
      }
    }
  } else {
    // read message from controller serial
    if (!controller_serial.available()) {
      return false;
    }
    while (controller_serial.available()) {
      reading = controller_serial.read();
      complete = process_controller_serial(reading);
      if (complete) {
        break;
      }
    }
  }

  // switch to other serial port if message is complete
  if (complete) {
    serial_pos = 0;
    for (int i = 0; i < (int)sizeof(serial_buffer); i++) {
      serial_buffer[i] = 0;
    }
    if (!display_serial.isListening()) {
      display_serial.listen();
    } else {
      controller_serial.listen();
    }
  }
  serial_counter = 0;
  return true;
}

/**
 * @brief Updates the light state on pin
 *
 */
void update_light_state() {
  digitalWrite(PIN_O_LGHT, light_current);
}

#endif

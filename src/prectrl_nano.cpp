#include <PID_v1_nano.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>

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
#define SERIAL_TIMEOUT 2000000/TICK_INTERVAL          // timeout of serial in serial update count (in µs/tick_interval)
#define SIGNAL_UPDATE 10000/TICK_INTERVAL             // interval for updating signal values (in ticks: µs/tick_interval)
#define PAS_TIMEOUT 400000/TICK_INTERVAL              // timeout for PWM with no PAS signal (in ticks: µs/tick_interval)
#define PAS_PULSE_TIMEOUT 20000/TICK_INTERVAL         // timeout for single PAS pulses (in ticks: µs/tick_interval)
#define PAS_FACTOR 60000000/TICK_INTERVAL/PAS_PULSES  // factor to convert PAS pulses to RPM (60.000.000ms == 60s == 1min / PAS pulses) / 2
                                                      // duration for 1 pas pulse in ticks: µs/tick_interval
                                                      // duration for 1 pas pulse in ticks: µs/tick_interval
#define TORQUE_MIN 330                                // min torque = 0 (1,5V -> needs to be calibrated)
#define TORQUE_MAX 480                                // max torque (around 3,0V -> needs to be calibrated)
#define TORQUE_MAX_NM 102                             // max torque in Nm -> 306 torque value readings starting from torque_low
#define TORQUE_FACTOR_NM 0.33                         // Torque reading to Nm factor
#define THROTTLE_MIN 250                              // min throttle
#define THROTTLE_MAX 750                              // max throttle
#define PAS_PULSES 36                                 // number of pulses per revolution for PAS signal
#define READINGS_AVG_TORQUE PAS_PULSES                // number of readings to average for torque
#define READINGS_AVG_POWER_INPUT PAS_PULSES/2         // number of readings to average for power input
#define READINGS_AVG_CADENCE PAS_PULSES/2             // number of readings to average for cadence
#define READINGS_AVG_THROTTLE 4                       // number of readings to average for throttle

// debug mode (for Serial output 115200)
#define DEBUG true

// global variables
volatile unsigned int tick_pas_counter = 0xFFFF;      // counter for ticks between two pas pulses
volatile unsigned int tick_timeout_counter = 0;       // counter for timeout between two pas pulses
volatile unsigned int signal_counter = 0;             // counter for throttle updates
volatile unsigned int serial_counter = 0;             // counter for serial updates
volatile bool interrupt_pas = false;                  // flag for pas interrupt

uint8_t serial_buffer[13];                            // buffer for received data (used for both sides)
byte serial_pos = 0;                                  // counter for recieved bytes
bool pedaling = false;                                // flag for pedaling
double cadence = 0.0;                                 // current cadence
double cadence_prev = 0.0;                            // previous cadence reading
double cadence_avg = 0.0;                             // cadence average
double factor[] = {                                   // factors for support levels 0-5
  0.0, 0.75, 1, 1.25, 1.5, 2.0};
int throttle_reading = 0;                             // current throttle reading
int throttle_low = THROTTLE_MIN;                      // lowest throttle measurement value (will be set at startup)
double throttle = 0.0;                                // current throttle value
double throttle_prev = 0.0;                           // previous throttle value
double throttle_avg = 0.0;                            // average throttle value
int torque_reading = 0;                               // latest torque reading
int torque_low = TORQUE_MIN;                          // lowest torque measurement value (will be set at startup)
int torque_pas_ticks = 0;                             // current pas duration in ticks
double torque = 0.0;                                  // current torque measurement in Nm
double torque_prev = 0.0;                             // previous torque measurement in Nm
double torque_avg = 0.0;                              // avg torque calculated in Nm
double power_input = 0.0;                             // current power provided by driver
double power_input_prev = 0.0;                        // previous power provided by driver
double power_input_avg = 0.0;                         // average power provided by driver
int pwm_output = 0;                                   // new pwm output
int pwm_output_prev = 0;                              // last pwm output

// serial readings (see reset_serial_values() for initialization)
int level_current;                                    // current support level (default 3)
int light_current;                                    // current light status
double wheel_size_current;                            // current wheel size
int max_speed_current;                                // current max speed
int power_current;                                    // current power
bool brake_current;                                   // current brake status
double speed_current;                                 // calculated speed

extern TimerOne timer1;
double torque_k_p = 1.03, torque_k_i = 0.5, torque_k_d = 0;
double torque_pid_in = 0, torque_pid_out = 0, torque_pid_set = 0;
PID pid_torque(&torque_pid_in, &torque_pid_out, &torque_pid_set,
               torque_k_p, torque_k_i, torque_k_d, DIRECT);
SoftwareSerial display_serial = SoftwareSerial(PIN_D_RX, -1);
SoftwareSerial controller_serial = SoftwareSerial(PIN_C_RX, -1);

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
  return true;
}

/**
 * @brief Reads in the throttle value
 *
 */
void read_throttle() {
  throttle_reading = analogRead(PIN_I_THROTTLE);
}

/**
 * @brief Reads in the torque value
 *
 */
void read_torque() {
  torque_reading = analogRead(PIN_I_TORQUE);
}

/**
 * @brief Upates the set_point value for the PID controller
 *
 */
void set_point() {
  // set target to a mix of current power input and average power input to maintain some speed
  // instead of just using current power input which is not accurate enough
  //
  // example 1 (torque only in nm 1/3 current and 2/3 average)
  //   torque_pid_set = (torque_avg * 0.67 + torque * 0.33) * 0.33;  // multiplied with 0.33 for nm
  //
  // example 2 (torque only 1/2 current and 1/2 average):
  //   torque_pid_set = (torque_avg + torque)/2;
  //
  // for torque based pid set:
  //torque_pid_in = (torque - torque_avg) * factor[level_current] * -1;
  //
  // to obtain more power the pid set value can be scaled by a factor
  // torque and cadence based (power input)
  //   torque_pid_set = (power_input_avg + power_input) / 2;
  // the target is no power input (the prectrl will try to balance out power input)
  // scaled by current support level
  //   torque_pid_in = (power_input - power_input_avg) * factor[level_current] * -1;

  torque_pid_set = (power_input_avg * 0.7 + power_input * 0.3) * pedaling;
  torque_pid_in = (power_input_avg * 0.7 + power_input * 0.3) * factor[level_current] * pedaling * -1;

  // update pid if not pedaling
  if (!pedaling || torque_avg <= 0) {
    pid_torque.ShrinkIntegral();
  }
  // reset pid if brake is pressed
  if (brake_current) {
    pid_torque.ResetIntegral();
  }
  pid_torque.Compute();
}

/**
 * @brief Updates the throttle values
 *
 * @param throttle_new
 */
void update_throttle(int throttle_new) {
  throttle_prev = throttle;
  throttle = map(throttle_new, throttle_low, THROTTLE_MAX, 0, 1023);
  throttle = constrain(throttle, 0, 1023);
  if (throttle_prev > 0 && throttle == 0) {
    // reset average if throttle is 0
    throttle_avg = 0;
  }
  throttle_avg -= throttle_avg / READINGS_AVG_THROTTLE;
  throttle_avg += throttle / READINGS_AVG_THROTTLE;
}

/**
 * @brief Updates the torque, cadence and power input values
 *
 * @param torque_new new torque reading
 */
void update_torque(double torque_new) {
  torque_prev = torque;
  torque = torque_new;
  if (torque > 0) {
    // if there is torque then subtract the low value from torque
    torque = torque - torque_low;
  }
  torque_avg -= torque_avg / READINGS_AVG_TORQUE;
  torque_avg += torque / READINGS_AVG_TORQUE;

  // update cadence in rpm
  cadence_prev = cadence;
  cadence = 0;
  if (torque_pas_ticks > 0) {
    cadence = PAS_FACTOR / torque_pas_ticks;
  }
  cadence_avg -= cadence_avg / READINGS_AVG_CADENCE;
  cadence_avg += cadence / READINGS_AVG_CADENCE;

  // update power input in nm
  //
  // power = 2 * pi * cadence_in_rpm * torque_in_nm / 60s
  // multiplication constant for SEMPU and T9 is approx. 0.33Nm/count
  //
  // 2 * pi / 60 * cadence * torque_nm
  // = 2 * pi / 60 = 2 * 3.14159 / 60
  // = 0.1047196667
  // = 0.1047196667 * cadence * torque_nm
  double torque_nm = torque * 0.33;
  power_input_prev = power_input;
  power_input = (2 * 3.14159 * cadence * torque_nm) / 60;
  Serial.print(power_input);
  Serial.print(" - ");
  Serial.print(0.1047196667 * cadence * torque_nm);
  Serial.print(" <- should be the same");
  Serial.println();
  power_input_avg -= power_input_avg / READINGS_AVG_POWER_INPUT;
  power_input_avg += power_input / READINGS_AVG_POWER_INPUT;
}

/**
 * @brief Updates the light state on pin
 *
 */
void update_light_state() {
  digitalWrite(PIN_O_LGHT, light_current);
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
}

/**
 * @brief Setup method
 *
 */
void setup() {
  // start serial port
  #if DEBUG == true
  Serial.begin(115200);
  #endif

  // start software serial port
  reset_serial_values();
  controller_serial.begin(9600);
  display_serial.begin(9600);

  // prepare pins
  pinMode(PIN_O_PWM, OUTPUT);           // output pwm
  pinMode(PIN_O_LGHT, OUTPUT);          // output light request
  pinMode(PIN_I_PAS, INPUT_PULLUP);     // input pas
  pinMode(PIN_I_TORQUE, INPUT);         // input torque
  pinMode(PIN_I_THROTTLE, INPUT);       // input throttle

  // prepare timer1
  timer1.initialize(TICK_INTERVAL);
  timer1.attachInterrupt(interrupt_time_tick);
  timer1.setPwmDuty(PIN_O_PWM, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_I_PAS), interrupt_pas_pulse, RISING);

  // pid_torque controller
  pid_torque.SetMode(AUTOMATIC);
  pid_torque.SetOutputLimits(0, 1023);
  pid_torque.SetSampleTime(10);  // 10ms

  // store inital readings for torque and and throttle for min values
  torque_low = min(analogRead(PIN_I_TORQUE) + 5, TORQUE_MIN);
  throttle_low = min(analogRead(PIN_I_THROTTLE) + 5, THROTTLE_MIN);
}

/**
 * @brief Main loop
 *
 */
void loop() {
  bool update_output = false;

  // pas pulse part in main loop, controlled by pas interrupt
  if (interrupt_pas) {
    interrupt_pas = false;
    update_output = true;
    read_torque();
    update_torque(torque_reading);

  }

  // signal update part in main loop
  if (signal_counter >= SIGNAL_UPDATE) {
    signal_counter = 0;
    update_output = true;
    read_throttle();
    update_throttle(throttle_reading);
    set_point();
  }

  // serial update part in main loop
  if (serial_counter >= SERIAL_UPDATE) {
    if (read_software_serial()) {
      serial_counter = 0;
      update_output = true;
    }
  }

  // the pwm output needs to be updated
  if (update_output) {
    update_output = false;
    update_light_state();
    update_pas_state();
    update_pwm_output();
  }
}

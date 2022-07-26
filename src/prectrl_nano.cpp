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
#define THROTTLE_UPDATE 10000/TICK_INTERVAL           // interval for reading throttle values (in ticks: µs/tick_interval)
#define PAS_TIMEOUT 400000/TICK_INTERVAL              // timeout for PWM with no PAS signal (in ticks: µs/tick_interval)
#define PAS_PULSE_TIMEOUT 20000/TICK_INTERVAL         // timeout for single PAS pulses (in ticks: µs/tick_interval)
#define PAS_FACTOR 60000000/TICK_INTERVAL/PAS_PULSES  // factor to convert PAS pulses to RPM (60.000.000ms == 60s == 1min / PAS pulses)
                                                      // duration for 1 pas pulse in ticks: µs/tick_interval
                                                      // duration for 1 pas pulse in ticks: µs/tick_interval
#define TORQUE_MIN 330                                // min torque = 0 (1,5V -> needs to be calibrated)
#define TORQUE_MAX 480                                // max torque (around 3,0V -> needs to be calibrated)
#define THROTTLE_MIN 250                              // min throttle
#define THROTTLE_MAX 750                              // max throttle
#define PAS_PULSES 36                                 // number of pulses per revolution for PAS signal
#define READINGS_AVG 20                               // number of readings to average
#define RESET_PID_ON_PAUSE true                       // reset PID controller when brake is pressed or driver is not pedaling

#define DEBUG false

// global variables
volatile unsigned int tick_pas_counter = 0xFFFF;      // counter for ticks between two pas pulses
volatile unsigned int tick_timeout_counter = 0;       // counter for timeout between two pas pulses
volatile unsigned int throttle_counter = 0;           // counter for throttle updates
volatile unsigned int serial_counter = 0;             // counter for serial updates
volatile bool interrupt_pas = false;                  // flag for pas interrupt

uint8_t serial_buffer[13];                            // buffer for received data (used for both sides)
byte serial_pos = 0;                                  // counter for recieved bytes
bool pedaling = false;                                // flag for pedaling
double cadence_current = 0.0;                         // current cadence
double cadence_prev = 0.0;                            // previous cadence reading
double cadence_avg = 0.0;                             // cadence average
double factor[] = {                                   // factors for support levels 0-5
  0.0, 0.75, 1, 1.25, 1.5, 2.0};
int torque_reading = 0;                               // latest torque reading
int torque_low = 0;                                   // lowest torque measurement value (will be set at startup)
double torque_current = 0.0;                          // current torque measurement in Nm
double torque_prev = 0.0;                             // previous torque measurement in Nm
double torque_avg = 0.0;                              // avg torque calculated in Nm
double torque_sum = 0.0;                              // sum of torque measurements
int torque_values[PAS_PULSES];                        // current torque measurement values
double power_input_current = 0.0;                     // current power provided by driver (current)
double power_input_prev = 0.0;                        // previous power provided by driver (current)
double power_input_avg = 0.0;                         // current power provided by driver (avg)
unsigned int torque_pas_ticks = 0;                    // current pas duration in ticks
unsigned int torque_pas_idx = 0;                      // current pas pulse index
unsigned int throttle_reading = 0;                    // current throttle reading
unsigned int throttle_low = 0;                        // lowest throttle measurement value (will be set at startup)
unsigned int pwm_output = 0;                          // new pwm output
unsigned int last_pwm_output = 0;                     // last pwm output

// serial readings (see reset_serial_values() for initialization)
unsigned int level_current;                           // current support level (default 3)
unsigned int light_current;                           // current light status
double wheel_size_current;                            // current wheel size
unsigned int max_speed_current;                       // current max speed
unsigned int power_current;                           // current power
bool brake_current;                                   // current brake status
double speed_current;                                 // calculated speed

extern TimerOne timer1;
double k_p = 1.03, k_i = 0.5, k_d = 0;
double pid_in = 0, pid_out = 0, pid_set = 0;
PID pid_throttle(&pid_in, &pid_out, &pid_set, k_p, k_i, k_d, DIRECT);
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
  if (throttle_counter < 0xFFFF) {
    throttle_counter++;
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
  unsigned int curr_light = serial_buffer[1] & 0x80;  // 128: 0b10000000
  unsigned int curr_level = serial_buffer[1] & 0x07;  // 7: 0b00000111
  unsigned int max_speed = 10 + (((serial_buffer[2] & 0xF8) >> 3) | (serial_buffer[4] & 0x20));
  unsigned int wheel_size = ((serial_buffer[4] & 0xC0) >> 6) | ((serial_buffer[2] & 0x07) << 2);

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

  #if DEBUG == true
  Serial.println("process_display_message");
  Serial.print("light:");
  Serial.print(light_current);
  Serial.print(", level:");
  Serial.print(level_current);
  Serial.print(", max_speed:");
  Serial.print(max_speed_current);
  Serial.print(", wheel_size:");
  Serial.print(wheel_size_current);
  Serial.println();
  #endif
}

/**
 * @brief Processes the serial message recieved from controller
 *
 */
void process_controller_message() {
  double rotation_ms = (serial_buffer[3] * 0xFF) + serial_buffer[4];
  unsigned int power = serial_buffer[8] * 13;  // 13W per unit
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

  #if DEBUG == true
  Serial.println("process_controller_message");
  Serial.print("power:");
  Serial.print(power_current);
  Serial.print(", brake:");
  Serial.print(brake);
  Serial.print(", rotation_ms:");
  Serial.print(rotation_ms);
  Serial.print(", speed:");
  Serial.print(speed_current);
  Serial.println();
  #endif
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
  power_current = 0;
  brake_current = false;
  speed_current = 0;

  #if DEBUG == true
  Serial.println("reset_serial_values");
  #endif
}

/**
 * @brief Reads display and controller messages from serial
 *
 * This function reads a complete message from a serial port
 * and then switches to another. After a whole message is recieved
 * it is being processed.
 */
void read_software_serial() {
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
      return;
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
      return;
    }
    while (controller_serial.available()) {
      reading = controller_serial.read();
      complete = process_controller_serial(reading);
      if (complete) {
        break;
      }
    }
  }
  serial_counter = 0;

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
}

/**
 * @brief Reads in the throttle value
 *
 */
void read_throttle() {
  throttle_counter = 0;
  throttle_reading = analogRead(PIN_I_THROTTLE);

  #if DEBUG == true
  Serial.println("read_throttle");
  Serial.print("throttle_reading:");
  Serial.print(throttle_reading);
  Serial.println();
  #endif
}

/**
 * @brief Reads in the torque value
 *
 */
void read_torque() {
  torque_reading = analogRead(PIN_I_TORQUE);

  #if DEBUG == true
  Serial.println("read_torque");
  Serial.print("torque_reading:");
  Serial.print(torque_reading);
  Serial.println();
  #endif
}

/**
 * @brief Upates the set_point value for the PID controller
 *
 */
void set_point() {
  // 1/3 of torque avg 2/3 of toque current and multiplied with 0.33 nm
  // see power input calculation in update_torque() for more info
  //pid_in = (torque_avg * 0.33 + torque_current * 0.67) * 0.33 * factor[level_current] * -1;

  pid_in = (torque_avg + torque_current)/2 * factor[level_current] * pedaling;
  //pid_set = (torque_current - torque_prev) * -1;
  pid_set = torque_current - torque_prev;

  //pid_in = (power_input_avg * 0.33 + power_input_current * 0.67) * factor[level_current] * -1;
  //pid_set = (power_input_current - power_input_prev) * -1;
  pid_throttle.Compute();

  #if DEBUG == true
  Serial.println("set_point");
  Serial.print("pid_in:");
  Serial.print(pid_in);
  Serial.print(", pid_set:");
  Serial.print(pid_set);
  Serial.print(", pid_out:");
  Serial.print(pid_out);
  Serial.println();
  #endif
}

/**
 * @brief Updates the torque values for avg torque calculation
 *
 * @param torque_new new torque reading
 */
void update_torque(double torque_new) {
  torque_new -= torque_low;
  // set very low values to zero to avaid stuttering
  if (torque_new < 5) {
    torque_new = 0;
  }
  // add current reading to valeus array
  torque_values[torque_pas_idx] = torque_new;
  if (torque_pas_idx++ >= PAS_PULSES - 1) {
    torque_pas_idx = 0;
  }
  // update torque sum
  torque_avg -= torque_avg / READINGS_AVG;
  torque_avg += torque_current / READINGS_AVG;
  // update torque
  torque_prev = torque_current;
  torque_current = torque_new;
  // update cadence
  cadence_prev = cadence_current;
  cadence_current = 0;
  if (torque_pas_ticks > 0) {
    cadence_current = PAS_FACTOR / torque_pas_ticks;
    cadence_current *= 0.5;
  }
  cadence_avg -= cadence_avg / READINGS_AVG;
  cadence_avg += cadence_current / READINGS_AVG;
  // update power input in nm
  // power = 2 * pi * cadence_in_rpm * torque_in_nm / 60s
  // multiplication constant for SEMPU and T9 is approx. 0.33Nm/count
  // 2 * pi / 60 * cadence * torque * 0.33
  // = 2 * pi / 60 * 0.33       = 2 * 3.14159 / 60 * 0.33
  // = 0.1047196667 * 0.33      = 0.03455749
  // = 0.03455749 * cadence * torque
  power_input_prev = power_input_current;
  power_input_current = 0.03455749 * cadence_current * torque_current;
  power_input_avg -= power_input_avg / READINGS_AVG;
  power_input_avg += power_input_current / READINGS_AVG;
  // update pid if no driver input is given
  if (brake_current || !torque_current || !pedaling) {
    if (RESET_PID_ON_PAUSE) {
      pid_throttle.ResetIntegral();
    } else {
      pid_throttle.ShrinkIntegral();
    }
  }

  #if DEBUG == true
  Serial.println("update_torque");
  Serial.print("torque_current:");
  Serial.print(torque_current);
  Serial.print(" torque_avg:");
  Serial.print(torque_avg);
  Serial.print(" cadence_current:");
  Serial.print(cadence_current);
  Serial.print(" power_input_avg:");
  Serial.print(power_input_avg);
  Serial.print(" power_input_current:");
  Serial.print(power_input_current);
  Serial.println();
  #endif
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
  #if DEBUG == true
  Serial.println("update_pas_state");
  Serial.print("pedaling:");
  Serial.print(pedaling);
  Serial.print(" tick_pas_counter:");
  Serial.print(tick_pas_counter);
  Serial.print(" tick_timeout_counter:");
  Serial.print(tick_timeout_counter);
  Serial.println();
  #endif
}

/**
 * @brief Updates the pwm output signal
 *
 */
void update_pwm_output() {
  pwm_output = 0;
  // torque based throttle
  if (pedaling) {
    // pid_out is in range 0-1023
    pwm_output = pid_out;
  }
  // throttle override
  if (throttle_reading > throttle_low) {
    // extend throttle range to 0-1023 from real low to constant high
    unsigned int throttle_temp = map(
      throttle_reading, throttle_low, THROTTLE_MAX, 0, 1023);
    if (throttle_temp > THROTTLE_MIN && throttle_temp > pwm_output) {
      pwm_output = throttle_temp;
    }
  }
  // ensure pwm output is within bounds
  pwm_output = constrain(pwm_output, 0, 1023);
  // map pwm output value to throttle min and max values
  pwm_output = map(pwm_output, 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
  if (pwm_output != last_pwm_output) {
    // update pwm duty cycle
    timer1.pwm(PIN_O_PWM, pwm_output);
    last_pwm_output = pwm_output;

    #if DEBUG == true
    Serial.println("update_pwm_output");
    Serial.print("pwm_output:");
    Serial.print(pwm_output);
    Serial.println();
    #endif
  }
}

/**
 * @brief Prints some debug output
 *
 */
void print_debug() {
  Serial.println("print_debug");
  Serial.print("torque:");
  Serial.print(torque_avg);
  Serial.print(", torque_current:");
  Serial.print(torque_current);
  Serial.print(", torque_reading:");
  Serial.print(torque_reading);
  Serial.print(", torque_sum:");
  Serial.print(torque_sum);
  Serial.print(", torque_pas_ticks:");
  Serial.print(torque_pas_ticks);
  Serial.print(", torque_pas_idx:");
  Serial.print(torque_pas_idx);
  Serial.print(", torque_values:");
  for (int i=0; i < PAS_PULSES; i++){
    Serial.print(torque_values[i]);
    Serial.print(", ");
  }
  Serial.print(", cadence_current:");
  Serial.print(cadence_current);
  Serial.print(", pedaling:");
  Serial.print(pedaling);
  Serial.print(", pwm_output:");
  Serial.print(pwm_output);
  Serial.print(", pid_out:");
  Serial.print(pid_out);
  Serial.print(", throttle_reading:");
  Serial.print(throttle_reading);
  Serial.print(", throttle_low:");
  Serial.print(throttle_low);
  Serial.print(", throttle_values:");
  Serial.print(", tick_pas_counter:");
  Serial.print(tick_pas_counter);
  Serial.print(", tick_timeout_counter:");
  Serial.print(tick_timeout_counter);
  Serial.println();
}

/**
 * @brief Setup method
 *
 */
void setup() {
  // start serial port
  Serial.begin(115200);
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
  // pid_throttle controller
  pid_throttle.SetMode(AUTOMATIC);
  pid_throttle.SetOutputLimits(0, 1023);
  pid_throttle.SetSampleTime(10);  // 10ms
  // initialze variables
  for (int i=0; i < PAS_PULSES; i++) {
    torque_values[i] = 0;
  }
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
    read_torque();
    update_torque(torque_reading);
    set_point();
    update_output = true;
  }

  // throttle update part in main loop
  if (throttle_counter > THROTTLE_UPDATE) {
    read_throttle();
    update_output = true;
  }

  // serial update part in main loop
  if (serial_counter > SERIAL_UPDATE) {
    read_software_serial();
    update_output = true;
  }

  // the pwm output needs to be updated
  if (update_output) {
    update_output = false;
    update_light_state();
    update_pas_state();
    update_pwm_output();

    #if DEBUG == true
      print_debug();
    #endif
  }
}

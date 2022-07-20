#include <PID_v1_nano.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>

#define PIN_O_PWM 10                    // D10 timer1
#define PIN_O_LGHT 3                    // D3  Light request from controller
#define PIN_I_PAS 2                     // D2
#define PIN_I_TORQUE A0                 // A0
#define PIN_I_THROTTLE A1               // A1
#define PIN_D_RX 5                      // D5  RX from display
#define PIN_C_RX 6                      // D6  RX from controller
#define PIN_TX -1                       // -1, TX, -1 = do not use
#define DEBUG false

#define TICK_INTERVAL 100               // time interval between two timer interrupts in µs (100 == 0.1ms, 10khz)

#define TORQUE_MIN 300                  // min torque = 0 (1,5V -> needs to be calibrated)
#define TORQUE_MAX 450                  // max torque (around 3,0V -> needs to be calibrated)
#define THROTTLE_MIN 200                // min throttle
#define THROTTLE_MAX 750                // max throttle
#define PAS_PULSES 36                   // number of pulses per revolution for PAS signal
#define SIGNAL_UPDATE 1000/TICK_INTERVAL             // interval for reading signal values (in ticks)
#define PAS_TIMEOUT 500000/TICK_INTERVAL              // timeout for PWM with no PAS signal (in ticks)
#define PAS_PULSE_TIMEOUT 20000/TICK_INTERVAL         // timeout for single PAS pulses (in ticks)
#define PAS_FACTOR 60000000/TICK_INTERVAL/PAS_PULSES  // factor to convert PAS pulses to RPM (60.000.000ms == 60s == 1min / PAS pulses)
#define TORQUE_THROTTLE_MAX 20                        // torque in nm that will give full throttle

volatile unsigned int tick_pas_counter = 0xFFFF;      // counter for ticks between two pas pulses
volatile unsigned int tick_timeout_counter = 0;       // counter for timeout between two pas pulses
volatile unsigned int signal_counter = 0;             // counter for signal updates
volatile bool interrupt_pas = false;                  // flag for pas interrupt

uint8_t serial_buffer[13];              // buffer for received data (used for both sides)
byte serial_pos = 0;                    // counter for recieved bytes
bool pedaling = false;                  // flag for pedaling
double cadence = 0.0;                   // current cadence
double factor[] = {                     // factors for support levels 0-5
  0.0, 0.75, 1, 1.25, 1.5, 2.0};
int torque_current = 0;                 // current torque reading
int torque_low = 0;                     // lowest torque measurement value (will be set at startup)
double torque_latest = 0.0;             // latest torque measurement in Nm
double torque = 0.0;                    // current torque calculated in Nm
double torque_sum = 0.0;                // sum of torque measurements
int torque_values[PAS_PULSES];          // current torque measurement values
double power_input = 0.0;               // current power provided by driver
unsigned int torque_pas_ticks = 0;      // current pas duration in ticks
unsigned int torque_pas_idx = 0;        // current pas pulse index
int throttle_current = 0;               // current throttle value
unsigned int throttle_low = 0;          // lowest throttle measurement value (will be set at startup)
unsigned int pwm_output = 0;            // pwm output

// serial readings
unsigned int reading_level = 3;         // reading support level (default 3)
unsigned int reading_light = 0;         // reading light status
float reading_wheel_size = 0.0;         // reading wheel size
unsigned int reading_max_speed = 0;     // reading max speed
unsigned int reading_power = 0;         // reading power

float calc_speed = 0;            // calculated speed

extern TimerOne timer1;
double k_p = 1.03, k_i = 0.5, k_d = 0;
double pid_in, pid_out, pid_set;
PID m_pid(&pid_in, &pid_out, &pid_set, k_p, k_i, k_d, DIRECT);
SoftwareSerial display_serial = SoftwareSerial(PIN_D_RX, PIN_TX);
SoftwareSerial controller_serial = SoftwareSerial(PIN_C_RX, PIN_TX);

/**
 * @brief Upates the set_point value for the PID controller
 *
 */
void set_point() {
  /*
  sollGas=minGas+500.0*Faktor[actstuf]*sumTorque/actPAS*(1+actV/maxV);   //Berechunung Gasstellwert: Faktor*MenschlicheLeistung*(1+v/vmax) 7m/s*10^6µs/1000mm  *(1+Radumfang/(actTacho*7000.0*TIC_RATE))
  if (sollGas>65535){                                                 //Auf 16Bit Schreiben PWM-Ausgabe beschränken.
    sollGas=65535;
    }
  */

  Serial.print("power_input: ");
  Serial.print(power_input);
  Serial.print(", torque: ");
  Serial.print(torque);
  Serial.print(", cadence: ");
  Serial.print(cadence);
  Serial.print(", diff: ");
  Serial.print((power_input-torque));
  Serial.println();
  /*
  #ifdef SUPPORT_TORQUE_THROTTLE                              //we want to trigger throttle just by pedal torque
  if (abs(torque_instant)>torque_throttle_min)            //we are above the threshold to trigger throttle
  {
    double power_torque_throttle = abs(torque_instant/torque_throttle_full*poti_stat/1023*curr_power_max);  //translate torque_throttle_full to maximum power
    power_throttle = max(power_throttle,power_torque_throttle); //decide if thumb throttle or torque throttle are higher
    power_throttle = constrain(power_throttle,0,curr_power_max); //constrain throttle value to maximum power
  }
  #endif
  */
    // power_poti = poti_stat/102300.0* curr_power_poti_max*power_human*(1+spd/20.0); //power_poti_max is in this control mode interpreted as percentage. Example: power_poti_max=200 means; motor power = 200% of human power

    //    power_poti = poti_stat/102300.0* curr_power_poti_max*power_human*(1+spd/20.0);  //power_poti_max is in this control mode interpreted as percentage. Example: power_poti_max=200 means; motor power = 200% of human power
    //double power_torque_throttle = abs(torque_instant/torque_throttle_full*poti_stat/1023*curr_power_max);  //translate torque_throttle_full to maximum power
  // FIXME this needs to be tested, intention is to grow throttle while human power is added

  /*pid_in = (torque/TORQUE_THROTTLE_MAX) * 1024;
  pid_set = ((power_input/TORQUE_THROTTLE_MAX) / factor[current_level]) * 1024;
  // pid_in = (torque/TORQUE_THROTTLE_MAX);
  m_pid.Compute();
  Serial.print("pid_in: ");
  Serial.print(pid_in);
  Serial.print(", pid_set: ");
  Serial.print(pid_set);
  Serial.print(", pid_out: ");
  Serial.print(pid_out);
  Serial.println();
  */

  return;
}

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
void _process_display_message() {
  unsigned int curr_light = serial_buffer[1] & 0x80;  // 128: 0b10000000
  unsigned int curr_level = serial_buffer[1] & 0x07;  // 7: 0b00000111
  unsigned int max_speed = 10 + (((serial_buffer[2] & 0xF8) >> 3) | (serial_buffer[4] & 0x20));
  unsigned int wheel_size = ((serial_buffer[4] & 0xC0) >> 6) | ((serial_buffer[2] & 0x07) << 2);

  if (curr_light != reading_light) {
    reading_light = curr_light;
    digitalWrite(PIN_O_LGHT, reading_light);
  }
  reading_level = curr_level;
  reading_max_speed = max_speed;
  switch (wheel_size) {
    case 0x12: // 6''
    reading_wheel_size = 468.75;
    break;

    case 0x0A: // 8''
    reading_wheel_size = 628.47;
    break;

    case 0x0E: // 10''
    reading_wheel_size = 788.19;
    break;

    case 0x02: // 12''
    reading_wheel_size = 947.91;
    break;

    case 0x06: // 14''
    reading_wheel_size = 1107.64;
    break;

    case 0x00: // 16''
    reading_wheel_size = 1267.36;
    break;

    case 0x04: // 18''
    reading_wheel_size = 1427.08;
    break;

    case 0x08: // 20''
    reading_wheel_size = 1576.39;
    break;

    case 0x0C: // 22''
    reading_wheel_size = 1743.05;
    break;

    case 0x10: // 24''
    reading_wheel_size = 1895.83;
    break;

    case 0x18: // 700c
    reading_wheel_size = 2173.61;
    break;

    case 0x1C: // 28''
    reading_wheel_size = 2194.44;
    break;

    case 0x1E: // 29''
    reading_wheel_size = 2250;
    break;

    case 0x14: // 26''
    default: // 26''
    reading_wheel_size = 2060;
    break;
  }

  #if DEBUG == true
  Serial.print("light: ");
  Serial.print(reading_light);
  Serial.print(", level: ");
  Serial.print(reading_level);
  Serial.print(", max_speed: ");
  Serial.print(reading_max_speed);
  Serial.print(", wheel_size: ");
  Serial.print(reading_wheel_size);
  Serial.println();
  #endif
}

/**
 * @brief Processes the serial message recieved from controller
 *
 */
void _process_controller_message() {
  float rotation_ms = (serial_buffer[3] * 0xFF) + serial_buffer[4];
  unsigned int power = serial_buffer[8] * 13;  // 13W per unit
  reading_power = power;
  calc_speed = 3600 * (reading_wheel_size / rotation_ms / 1000);
  if (rotation_ms >= 6895) {
    // if a wheel rotation takes over 6 seconds the speed
    // is set to 0
    // the highest value returned for rotation_ms is 6895
    calc_speed = 0;
  }
  #if DEBUG == true
  Serial.print("power: ");
  Serial.print(reading_power);
  Serial.print(", rotation_ms: ");
  Serial.print(rotation_ms);
  Serial.print(", speed: ");
  Serial.print(calc_speed);
  Serial.println()
  #endif
}

/**
 * @brief Processes current byte from display
 *
 * @param reading current byte
 * @return bool true if message is complete else false
 */
bool _process_display_serial(uint8_t reading) {
  /*
  S-LCD3 to S12SN communication protocol.

  Packet consist of 13 bytes. 9600 baud, 8-n-1, byte-by-byte
  B0	B1	B2	B3	B4	B5	B6	B7	B8	B9	B10	B11	B12
  (e.g: 12 0 149 160 41 102 18 74 4 20 0 50 14)
  */
  serial_buffer[serial_pos] = reading;
  serial_pos += 1;
  if (serial_pos == 13 && serial_buffer[12] == 0xE) {
    _process_display_message();
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
bool _process_controller_serial(byte reading) {
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
    _process_controller_message();
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
void read_software_serial() {
  uint8_t reading;
  bool complete;

  // read bytes from currently selected serial port
  // until message is complete
  if (display_serial.isListening()) {
    // read message from display serial
    if (!display_serial.available()) {
      return;
    }
    while (display_serial.available()) {
      reading = display_serial.read();
      complete = _process_display_serial(reading);
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
      complete = _process_controller_serial(reading);
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
}

/**
 * @brief Reads in the the current throttle value
 *
 */
void read_throttle() {
  /**
   * @brief Reads the current throttle value and normalizes it
   *
   */
  throttle_current = analogRead(PIN_I_THROTTLE);
  #if DEBUG == true
  Serial.print("throttle_current before: ");
  Serial.print(throttle_current);
  #endif
  throttle_current = map(throttle_current, throttle_low, THROTTLE_MAX, 0, 1023);
  if (throttle_current < 10) {
    throttle_current = 0;
  }
  throttle_current = constrain(throttle_current, 0, 1023);
  #if DEBUG == true
  Serial.print(" throttle_current after: ");
  Serial.print(throttle_current);
  Serial.println();
  #endif
}

/**
 * @brief Reads in the current torque value
 *
 */
void read_torque() {
  /**
   * @brief Reads current torque value and normalizes it
   *
   */
  torque_current = analogRead(PIN_I_TORQUE) - torque_low;
  #if DEBUG == true
  Serial.print("torque_current before: ");
  Serial.print(torque_current);
  #endif
  if (torque_current < 20) {
    torque_current = 0;
  }
  torque_current = constrain(torque_current, 0, 1023);
  #if DEBUG == true
  Serial.print(" torque_current after: ");
  Serial.print(torque_current);
  Serial.println();
  #endif
}

/**
 * @brief Updates the torque values for avg torque calculation
 *
 * @param torque_new new torque reading
 */
void update_torque(int torque_new) {
  // add current reading to valeus array
  torque_values[torque_pas_idx] = torque_new;
  if (torque_pas_idx++ >= PAS_PULSES - 1) {
    torque_pas_idx = 0;
  }
  // update torque sum
  torque_sum = 0;
  for (int i=0; i < PAS_PULSES; i++){
    torque_sum += torque_values[i];
  }
  // update cadence
  cadence = 0;
  if (torque_pas_ticks > 0) {
    cadence = (PAS_FACTOR / torque_pas_ticks) * pedaling;
    cadence /= 2;
  }
  // update latest torque in Nm
  // multiplication constant for SEMPU and T9 is approx. 0.33Nm/count
  torque_latest = 0.33 * torque_current * pedaling;
  // update avg torque in Nm
  // 36/1023 = 0.03519061584: T9 has 36 pulses per revolution
  torque = 0.03519061584 * torque_sum * pedaling;
  // power = 2 * pi * cadence * torque / 60s -> (2 * pi / 60) * cadence * torque
  power_input = 0.10471975512 * cadence * torque;
  // FIXME is this needed: update pid if no torque
  if (torque_new == 0) {
    //m_pid.ShrinkIntegral();
    m_pid.ResetIntegral();
  }
  #if DEBUG == true
  Serial.print("torque_latest: ");
  Serial.print(torque_latest);
  Serial.print(" torque: ");
  Serial.print(torque);
  Serial.print(" cadence: ");
  Serial.print(cadence);
  Serial.print(" power_input: ");
  Serial.print(power_input);
  Serial.println();
  #endif
}

/**
 * @brief Updates the state of pedaling
 *
 */
void update_state() {
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
  Serial.print("pedaling: ");
  Serial.print(pedaling);
  Serial.print(" tick_pas_counter: ");
  Serial.print(tick_pas_counter);
  Serial.print(" tick_timeout_counter: ");
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
    pwm_output = pid_out + throttle_low;
  }
  // throttle override
  if (throttle_current) {
    // use average throttle value instead of latest reading
    pwm_output = throttle_current;
  }
  // ensure pwm output is within bounds
  if (pwm_output < 0) {
    pwm_output = 0;
  }
  if (pwm_output > 1023) {
    pwm_output = 1023;
  }
  pwm_output = map(pwm_output, 0, 1023, 0, THROTTLE_MAX);
  // update pwm duty cycle
  timer1.pwm(PIN_O_PWM, pwm_output);
  #if DEBUG == true
  Serial.print("pwm_output: ");
  Serial.print(pwm_output);
  Serial.println();
  #endif
}

/**
 * @brief Prints some debug output
 *
 */
void print_debug() {
  Serial.print("torque: ");
  Serial.print(torque);
  Serial.print(", torque_latest: ");
  Serial.print(torque_latest);
  Serial.print(", torque_current: ");
  Serial.print(torque_current);
  Serial.print(", torque_sum: ");
  Serial.print(torque_sum);
  Serial.print(", torque_pas_ticks: ");
  Serial.print(torque_pas_ticks);
  Serial.print(", torque_pas_idx: ");
  Serial.print(torque_pas_idx);
  Serial.print(", torque_values: ");
  for (int i=0; i < PAS_PULSES; i++){
    Serial.print(torque_values[i]);
    Serial.print(", ");
  }
  Serial.print(", cadence: ");
  Serial.print(cadence);
  Serial.print(", pedaling: ");
  Serial.print(pedaling);
  Serial.print(", pwm_output: ");
  Serial.print(pwm_output);
  Serial.print(", pid_out: ");
  Serial.print(pid_out);
  Serial.print(", throttle_current: ");
  Serial.print(throttle_current);
  Serial.print(", throttle_low: ");
  Serial.print(throttle_low);
  Serial.print(", throttle_values: ");
  Serial.print(", tick_pas_counter: ");
  Serial.print(tick_pas_counter);
  Serial.print(", tick_timeout_counter: ");
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
  // pid controller
  m_pid.SetMode(AUTOMATIC);
  m_pid.SetOutputLimits(0, 1023);
  m_pid.SetSampleTime(10);  // 10ms
  // initialze variables
  for (int i=0; i < PAS_PULSES; i++) {
    torque_values[i] = 0;
  }
  torque_low = analogRead(PIN_I_TORQUE);
  throttle_low = analogRead(PIN_I_THROTTLE);
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
    // read current torque value
    read_torque();
    // update torque reading
    update_torque(torque_current);
    // update setpoint
    set_point();
    update_output = true;
  }

  // signal update part in main loop
  if (signal_counter > SIGNAL_UPDATE) {
    signal_counter = 0;
    // update settings from display reading
    read_software_serial();
    // update throttle reading
    read_throttle();
    update_output = true;
  }

  // the pwm output needs to be updated
  if (update_output) {
    update_output = false;
    // update current readings of state
    update_state();
    // update new pwm duty cycle
    update_pwm_output();
    #if DEBUG == true
      //print_debug();
    #endif
  }

}

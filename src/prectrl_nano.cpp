#include <PID_v1_nano.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>


#define PIN_O_PWM 10                    // D10 timer1
#define PIN_O_LGHT 3                    // D3  Light request from controller
#define PIN_I_PAS 2                     // D2
#define PIN_I_TORQUE A0                 // A0
#define PIN_I_THROTTLE A1               // A1
#define PIN_RX 5                        // D5  RX
#define PIN_TX -1                       // -1, TX, -1 = do not use
#define DEBUG true

#define TICK_INTERVAL 64                // time interval between two timer interrupts in µs (100 == 0.1ms, 10khz)

#define TORQUE_MIN 300                  // min torque = 0 (1,5V -> needs to be calibrated)
#define TORQUE_MAX 450                  // max torque (around 3,0V -> needs to be calibrated)
#define THROTTLE_MIN 170                // min throttle
#define THROTTLE_MAX 780                // max throttle
// TODO check a whole revolution
#define PAS_PULSES 36                   // number of pulses per revolution for PAS signal
#define SIGNAL_UPDATE 10000/TICK_INTERVAL             // interval for reading signal values (in ticks)
#define PAS_TIMEOUT 500000/TICK_INTERVAL              // timeout for PWM with no PAS signal (in ticks)
#define PAS_PULSE_TIMEOUT 20000/TICK_INTERVAL         // timeout for single PAS pulses (in ticks)
#define PAS_FACTOR 60000000/TICK_INTERVAL/PAS_PULSES  // factor to convert PAS pulses to RPM (60.000.000ms == 60s == 1min / PAS pulses)
#define TORQUE_THROTTLE_MAX 20                        // torque in nm that will give full throttle

volatile unsigned int tick_pas_counter = 0xffff;      // counter for ticks between two pas pulses
volatile unsigned int tick_timeout_counter = 0;       // counter for timeout between two pas pulses
volatile unsigned int signal_counter = 0;             // counter for signal updates
volatile bool interrupt_pas = false;                  // flag for pas interrupt


byte slcd_received[12];                 // buffer for received data
byte slcd_receive_counter = 0;          // counter for recieved bytes
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
unsigned int reading_speed = 0;         // reading speed
unsigned int reading_power = 0;         // reading power


extern TimerOne timer1;
double k_p = 1.03, k_i = 0.5, k_d = 0;
double pid_in, pid_out, pid_set;
PID m_pid(&pid_in, &pid_out, &pid_set, k_p, k_i, k_d, DIRECT);
SoftwareSerial display_serial = SoftwareSerial(PIN_RX, PIN_TX);


// TODO:
// - serial reading
// - set throttle based on throttle / torque
// - set light based on serial reading
// - if possible to read in speed and power from serial, set torque based on speed and power

void set_point() {
  /*
  Serial.print("power_input: ");
  Serial.print(power_input);
  Serial.print(", torque: ");
  Serial.print(torque);
  Serial.print(", cadence: ");
  Serial.print(cadence);
  Serial.print(", diff: ");
  Serial.print((power_input-torque));
  Serial.println();
  ifdef SUPPORT_TORQUE_THROTTLE                              //we want to trigger throttle just by pedal torque
    if (abs(torque_instant)>torque_throttle_min)            //we are above the threshold to trigger throttle
    {
      double power_torque_throttle = abs(torque_instant/torque_throttle_full(20)*poti_stat/1023*curr_power_max);  //translate torque_throttle_full to maximum power
      power_throttle = max(power_throttle,power_torque_throttle); //decide if thumb throttle or torque throttle are higher
      power_throttle = constrain(power_throttle,0,curr_power_max); //constrain throttle value to maximum power
    }
   */
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

void interrupt_time_tick() {
  if (tick_pas_counter < 0xffff) {
    tick_pas_counter++;
  }
  if (tick_timeout_counter < 0xffff) {
    tick_timeout_counter++;
  }
  if (signal_counter < 0xffff) {
    signal_counter++;
  }
}

void interrupt_pas_pulse() {
  if (digitalRead(PIN_I_PAS) != HIGH) {
    return;
  }
  torque_pas_ticks = tick_pas_counter;
  tick_pas_counter = 0;
  interrupt_pas = true;
}

void set_reading_light(unsigned int light) {
  reading_light = light;
}

void set_reading_level(unsigned int level) {
  if (level < 0 || level > 5) {
    return;
  }
  reading_level = level;
}

void read_display_serial() {
  if (!display_serial.available()) {
    return;
  }
  // TODO try to get display and controller messages
  bool complete = false;
  byte receivedbyte = display_serial.read();  // read in byte
  // collect bytes from message
  slcd_received[slcd_receive_counter] = receivedbyte; // store byte in buffer
  if (slcd_receive_counter >= 0) {
    slcd_receive_counter++;                   // increment byte counter
  }
  if (receivedbyte == 0x0E) {
    if (slcd_receive_counter == 12) {
      complete = true;
    }
    slcd_receive_counter = 0;
  } else if (slcd_receive_counter == 12) {
    slcd_receive_counter = 0;
  }
  if (slcd_receive_counter == 1) {
    if (receivedbyte == 0x41) {               // 1 byte: communication controller -> display
      slcd_receive_counter = -1;
    }
  }
  for (int i = 0; i < 12; i++) {
    Serial.print(slcd_received[i], HEX);
    Serial.print(" ");
  }
  // only process complete messages
  if (complete == false) {
    return;
  }
  Serial.print("Complete message ");
  Serial.println();
  // 2 byte: assist level, etc.
  set_reading_light(slcd_received[2] & 0b10000000);
  set_reading_level(slcd_received[2] & 0b00000111);
  //set_reading_speed(receivedbyte & 0b00001000);
  Serial.print("assist level: ");
  Serial.print(reading_level);
  Serial.print(", light: ");
  Serial.print(reading_light);
  Serial.print(", speed: ");
  Serial.print(reading_speed);
  Serial.print(", power: ");
  Serial.print(reading_power);
  Serial.println();
}

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
  Serial.print("throttle_current after: ");
  Serial.print(throttle_current);
  Serial.println();
  #endif
}

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
  Serial.print("torque_current after: ");
  Serial.print(torque_current);
  Serial.println();
  #endif
}

void update_torque(int torque_new) {
  // add current reading to valeus array
  torque_values[torque_pas_idx] = torque_new;
  if (torque_pas_idx++ >= PAS_PULSES - 1) {
    #if DEBUG == true
    Serial.println("Complete revolution");
    #endif
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
    m_pid.ShrinkIntegral();
  }
}

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
}

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
}

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

void setup() {
  // start serial port
  Serial.begin(9600);
  // start software serial port
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
  // set light request to low
  digitalWrite(PIN_O_LGHT, reading_light);
}

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
    read_display_serial();
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
  }

  #if DEBUG == true
    //print_debug();
  #endif
}

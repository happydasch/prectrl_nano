# PreController for BLDC Controllers (prectrl_nano)

The pre-controller was started to add support for a torque sensor to the KT-controller. The pre-controller is a simple controller that can be used to control the motor with a torque sensor together with a throttle signal. It reads in the analog torque signal, a PAS signal and a throttle signal and calculates the output to the throttle connector of a commercial E-bike BLDC-Controller. The support factor is taken from KT-controller KT-LCD3 communication.

The torque sensor is connected to the analog input of the pre-controller. The pre-controller then calculates the torque signal from the analog input and the PAS signal. The torque and throttle signal is then used to calculate the output to the throttle connector.

## Functionality

* Reads in the analog torque signal, a PAS signal and a throttle signal

* Writes the output to the connected throttle connector

* Mirrors the light signal from controller by setting a pin to HIGH or LOW
  This is used together with another project called turn_signals to control the lights of a bike

## Hardware

* Arduino Nano
* Low-Pass Filter on PWM Output
* Soldering
* Sempu Torque Sensor
* Handlebar Throttle
* KT-controller
* Optional: [turn_signals](https://github.com/happydasch/turn_signals/) connected to light request pin

# PreController for BLDC Controllers (prectrl_nano)

The precontroller was started to add support for a torque sensor to the KT-controller. The precontroller is a simple controller that can be used to control the motor with a torque sensor. It reads in the analog torque signal, a PAS signal and a throttle signal and calculates the output to the throttle connector of a commercial E-bike BLDC-Controller. The support factor is taken from KT-controller KT-LCD3 communication.

The torque sensor is connected to the analog input of the precontroller. The precontroller then calculates the torque signal from the analog input and the PAS signal. The torque and throttle signal is then used to calculate the output to the throttle connector.

## Functionality

* Reads in the analog torque signal, a PAS signal and a throttle signal
* Calculates the output to the throttle connector
* Writes the output to the throttle connector
* Mirrors the light signal from controller by setting a pin to HIGH or LOW

## Hardware

* Arduino Nano
* Sempu Torque Sensor
* Handlebar Throttle
* KT-controller

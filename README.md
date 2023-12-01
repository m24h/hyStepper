# hyStepper
A NEMA17 close-loop stepper motor controller with RS485 interface, using MT6701 as encoder

##Features:

* Using cheap MCU AIR001 and cheap magnet encoder MT6701
* An anti-lost semi-open-loop algorithm
* A FOC/PID algorithm 
* A RS485 interface and protocols (including an anti-lost stepping protocol)
* Having the basis of sampling and adcanced moving for future usage
* A simple UART ANSI shell is provided for test and configuration

## Making/Downloading

The hardware is designed by lcEDA. 
The software is only for AIR001, compiled and linked by GNU Tools for STM32 (10.3-2021.10)
The binary release can be downloading using pyOCD with customized enhancement

## Known issues
* The first thing is that I haven't actually used this system, the default parameters may be wrong.
* Not all ideas about protocols of RS485 has been done.
* When baudate of RS485 exceeds 250kbps, interference occurs due to communication with the encoder, This seems to be a problem within the MCU (pins are adjacent) but I don't have the energy to re-modify the hardware and software. 

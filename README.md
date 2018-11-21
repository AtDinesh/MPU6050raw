# MPU6050raw
===============
program to get data from an MPU6050 using an STM32 microcontroller

## Notes: 
### Requirements

In order to use the STM32 with Arduino Software, some manipulations must be done.
First, copy the Arduino_STM32 folder in the hardware folder of your Arduino installation directory. (tested with Arduino 1.6.11)
Then run Arduino_STM32/<linux version>/install.sh
**Please read the wiki (https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki) for full details**
Arduino_STM32 folder originialy comes from : 
* https://github.com/rogerclarkmelbourne/Arduino_STM32

Arduino sketches (.ino extension) must be compiled with Arduino IDE :
* https://www.arduino.cc/en/Main/Software

### Reason to use STM32:
the STM32 has a 'real' USB compared to most of Arduino boards. In our application, this serial communication is of real importance since we will receive all the data through here ! We need full speed USB.


## How to use:
### Objectives:
Allows to receive raw data from a MPU6050 at 1 Khz using a STM32.
The MPU is set to work at full scale 4g (accelerometer) and 250 degree/sec (gyroscope)
you can either receive data in ASCII (to plot using arduino 1.6.11) or receive data on USB ports.
Once received on USB, the log program will convert data into m/sec^2 for accelerometer and rad/sec for gyroscope. These conversions are done using constant variables with double precision.

### Building:

For log programs :
 - Open Terminal and go in the log folder
 - run 'gcc -o <wished_program_name> log_IMU_4g.c'
Replace <wished_program_name> with the name that you want for the output program.

### Running:
run <wished_program_name> <output_file>.txt

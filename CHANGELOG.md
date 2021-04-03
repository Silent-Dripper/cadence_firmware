# Changelog

## 6.0.1 - (2021-03-xx)

In an effort to tune the drive config to make less noise, changes were made to create less frequent, larger steps. 

* Decreased the amount of time between drips during calibration mode.
* Decreased the step signal frequency from 9615.4hz -> 4464.3hz, we will ask for steps less often.
* Decreased microsteps from 16->8, steps will be larger.
* Decreased the max value for the steps per drip pot from 1500->500.
* Decreased the actuator enable time from 400ms->250ms.

## 6.0.0 - (2021-03-20)

* Added support to control stepper motor pumps using the TMC2208.
* Added `config.h` to be able to switch between different platforms. The Cadence PCB, the Adafruit Motor Shield v2 and the new Silent Dripper PCB.
* Used a lot of compiler switching to abstract out differences in these different platforms.
* Added a python script, `protocol_tester.py` to this repo to be able to debug the serial communication protocol.
* Formatted the `c++` code using `clang-format`.
* Added doxygen style docstrings to each of the functions/objects in the codebase.
* Added images/some history of the project to the top README.md

## 5.2.0 - (2020-11-7)

* Set different enable times/pwm values depending on if the firmware is using the Adafruit Motor Shield v2 or the MOSFETs on the Cadence PCB.
* Sound comparison script can drive steppers via TMCs and DC's via the prod mosfets.

## 5.1.0 - (2020-10-27)

* Commands will not be echo'd back to the host until they have been completed by the Arduino and can accept another command.

## 5.0.0 - (2020-10-21)

* Added compiler switches to use the Adafruit Motor Shield v2 as the pump driver
* Improved no-finger detection.
* Added sketch to mess around with driving stepper motors.

## 4.2.0

* Added per-motor values for motor enable time and pwm value to form better drips.


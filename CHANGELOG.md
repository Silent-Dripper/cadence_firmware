# Changelog

## 6.0.0 - (2021-2-xx)

* Added support to control stepper motor pumps using the TMC2208

## 5.2.0 - (2020-11-7)

* Set different enable times/pwm values depending on if the firmware is using the Adafruit Motor Sheild v2 or the MOSFETs on the Cadence PCB.
* Sound comparison script can drive steppers via TMCs and DC's via the prod mosfets.

## 5.1.0 - (2020-10-27)

* Commands will not be echo'd back to the host until they have been completed by the Arduino and can accept another command.

## 5.0.0 - (2020-10-21)

* Added compiler switches to use the Adafruit Motor Shield v2 as the pump driver
* Improved no-finger detection.
* Added sketch to mess around with driving stepper motors.

## 4.2.0

* Added per-motor values for motor enable time and pwm value to form better drips.


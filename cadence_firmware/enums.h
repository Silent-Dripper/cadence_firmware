/*
  Enums have to be in their own file because of the shortcomings of the Arduino IDE.
*/

enum ActuatorStatus {
  unconfigured,
  good_config,
  bad_config,
  actuator_stopped,
  actuator_running,
};

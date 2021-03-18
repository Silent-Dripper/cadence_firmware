/*
  Enums have to be in their own file because of the shortcomings of the Arduino IDE.
*/

/**
 * @brief Different actuator states. This will be used to communicate to the user via LEDs
 * which state a given actuator is in.
 * 
 */
enum ActuatorStatus {
  unconfigured,
  good_config,
  bad_config,
  actuator_stopped,
  actuator_running,
};

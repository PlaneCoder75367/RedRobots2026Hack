#include <stdint.h>

/* Edit this integer to be equal to your team number.
*/
const uint8_t TEAM_NUMBER = 62;

/* Edit this string to include your team name! When you connect your robot to the Bluetooth
 * controller app, the name that will show up is "RR-[TEAM_NUMBER]-[TEAM_NAME]"

   Will be truncated after 15 characters
*/
const char *TEAM_NAME = "samplebot";

/* Uncomment this to receive more verbose debug packets over serial.
   e.g. when motor 2 is driven, you may see
   [DC_MOTOR] Motor 2 | Target Speed: -50 | Duty: 50% | Phase Pin: HIGH
*/
// #define REDROBOT_DEBUG 1 
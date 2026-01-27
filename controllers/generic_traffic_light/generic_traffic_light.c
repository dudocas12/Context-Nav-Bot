/*
 * Description:  Pedestrian traffic light controller (Red <-> Green only)
 * Arguments: 1) red time seconds (double, default 30s)
 * 2) green time seconds (double, default 30s)
 * 3) start color (r: red, g: green)
 */

#include <webots/led.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>

#define TIME_STEP 512
// Removed ORANGE states
enum { GREEN_STATE, RED_STATE };

int main(int argc, char **argv) {
  wb_robot_init();
  double red_time = 1.0;   // Default 30s
  double green_time = 1.0; // Default 30s
  int current_state = RED_STATE; // Default to RED for safety

  // ARGUMENT PARSING
  if (argc > 1) {
    sscanf(argv[1], "%lf", &red_time);
    if (argc > 2) {
      sscanf(argv[2], "%lf", &green_time);
      if (argc > 3) {
        if (strcmp(argv[3], "r") == 0)
          current_state = RED_STATE;
        else if (strcmp(argv[3], "g") == 0)
          current_state = GREEN_STATE;
      }
    } else
      green_time = red_time;
  }

  WbDeviceTag red_light, orange_light, green_light;
  red_light = wb_robot_get_device("red light");
  orange_light = wb_robot_get_device("orange light"); // We still get it to force it OFF
  green_light = wb_robot_get_device("green light");
  double last_phase_change_time = 0.0;

  // INITIAL STATE SETUP
  wb_led_set(orange_light, 0); // Always OFF
  
  if (current_state == GREEN_STATE) {
    wb_led_set(green_light, 1);
    wb_led_set(red_light, 0);
    wb_robot_set_custom_data("green");
  } else {
    wb_led_set(red_light, 1);
    wb_led_set(green_light, 0);
    wb_robot_set_custom_data("red");
  }

  // MAIN LOOP
  while (wb_robot_step(TIME_STEP) != -1) {
    double current_time = wb_robot_get_time();

    if (current_state == GREEN_STATE) {
      // GREEN -> RED (No Orange)
      if ((current_time - last_phase_change_time) >= green_time) {
        current_state = RED_STATE;
        last_phase_change_time = current_time;
        
        wb_led_set(green_light, 0);
        wb_led_set(red_light, 1);
        wb_robot_set_custom_data("red");
      }
    } else { // RED_STATE
      // RED -> GREEN (No Orange)
      if ((current_time - last_phase_change_time) >= red_time) {
        current_state = GREEN_STATE;
        last_phase_change_time = current_time;
        
        wb_led_set(red_light, 0);
        wb_led_set(green_light, 1);
        wb_robot_set_custom_data("green");
      }
    }
  };

  wb_robot_cleanup();

  return 0;
}
/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A controller moving the khepera III and its gripper.
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>



#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))
#define GRIPPER_MOTOR_MAX_SPEED 2.0
#define TIME_STEP 64

enum BLOB_TYPE { RED, GREEN, BLUE, NONE };

static WbDeviceTag sensors[MAX_SENSOR_NUMBER];
static WbDeviceTag gripper_motors[3];
static WbDeviceTag left_motor, right_motor;
static const double matrix[9][2] = {{-2.67, -2.67},  {-10.86, 21.37}, {-16.03, 26.71}, {-37.4, 37.4}, {37.4, -32.06},
                                    {26.71, -21.37}, {21.37, -10.86}, {-2.67, -2.67},  {-5.34, -5.34}};
static const int num_sensors = 9;
static const double range = 2000.0;
static int time_step = 0;
static const double max_speed = 19.1;

static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  char sensors_name[5];
  sprintf(sensors_name, "%s", "ds0");

  int i;
  for (i = 0; i < num_sensors; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);

    if ((i + 1) >= 10) {
      sensors_name[2] = '1';
      sensors_name[3]++;

      if ((i + 1) == 10) {
        sensors_name[3] = '0';
        sensors_name[4] = '\0';
      }
    } else {
      sensors_name[2]++;
    }
  }

  gripper_motors[0] = wb_robot_get_device("horizontal_motor");
  gripper_motors[1] = wb_robot_get_device("left_finger_motor");
  gripper_motors[2] = wb_robot_get_device("right_finger_motor");

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  const char *robot_name = wb_robot_get_name();
  printf("The %s robot is initialized, it uses %d distance sensors\n", robot_name, num_sensors);
}

void step(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void moveArms(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}

void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
  wb_motor_set_position(gripper_motors[2], -position);
}

void moveForwards(double speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
}

void turn(double speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, -speed);
}

void stop(double seconds) {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  step(seconds);
}

int main() {
  initialize();

    
  stop(1.0);

  moveArms(-2.7);

  moveFingers(0.6);
  step(1.0);
  
  while (true)
  {
    moveForwards(3.5);
    step(3.0);
    moveForwards(-3.5);
    step(3.0);
  }
  wb_robot_cleanup();
  return 0;
}

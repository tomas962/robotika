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
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/camera.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };
enum BLOB_TYPE { RED, GREEN, BLUE, NONE };

enum BLOB_TYPE get_image_color(WbDeviceTag camera, int width, int height){
    const unsigned char *image = NULL;
    image = wb_camera_get_image(camera);

    enum BLOB_TYPE current_blob;
    int red = 0;
    int blue = 0;
    int green = 0;
    for (int i = width / 3; i < 2 * width / 3; i++) {
        for (int j = height / 2; j < 3 * height / 4; j++) {
            red += wb_camera_image_get_red(image, width, i, j);
            blue += wb_camera_image_get_blue(image, width, i, j);
            green += wb_camera_image_get_green(image, width, i, j);
        }
    }

    if ((red > 3 * green) && (red > 3 * blue))
        current_blob = RED;
      else if ((green > 3 * red) && (green > 3 * blue))
        current_blob = GREEN;
      else if ((blue > 3 * red) && (blue > 3 * green))
        current_blob = BLUE;
      else
        current_blob = NONE;

    return current_blob;
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  
  int counter = 0, i = 0;
  int state = WAITING;
  enum BLOB_TYPE block_color = NONE;
  const double red_box_target_positions[] = {-1.88, -2.14, -2.38, -1.51};
  const double green_box_target_positions[] = {-0.5, -2, -2, 0};
  const double blue_box_target_positions[] = {-2.5, 0, -3, 0};
  
  double speed = 1.0;

  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  for (i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  
  // init camera
  
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  printf("height: %d, width: %d \n", height, width);

  while (wb_robot_step(TIME_STEP) != -1) {
    if (counter <= 0) {
      switch (state) {
        case WAITING:
          //if (wb_distance_sensor_get_value(distance_sensor) < 1500) {
          //  printf("Getting color \n");
          //}
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = GRASPING;
            counter = 8;
            printf("Grasping can\n");
            printf("Getting color\n");
            block_color = get_image_color(camera, width, height);
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85);
          }
          break;
        case GRASPING:
          switch (block_color)
            {
            case RED:
              for (i = 0; i < 4; ++i)
                wb_motor_set_position(ur_motors[i], red_box_target_positions[i]);
              break;
            case GREEN:
              for (i = 0; i < 4; ++i)
                wb_motor_set_position(ur_motors[i], green_box_target_positions[i]);
              break;
            case BLUE:
              for (i = 0; i < 4; ++i)
                wb_motor_set_position(ur_motors[i], blue_box_target_positions[i]);
              break;
            default:
              break;
            }
          state = ROTATING;
          break;
        case ROTATING:
          if (wb_position_sensor_get_value(position_sensor) < -2) {
            counter = 8;
            printf("Releasing can\n");
            state = RELEASING;
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0);
          }
          break;
        case RELEASING:
          for (i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], 0.0);
          printf("Rotating arm back\n");
          state = ROTATING_BACK;
          break;
        case ROTATING_BACK:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
            printf("Waiting can\n");
          }
          break;
      }
    }
    counter--;
  };

  wb_robot_cleanup();
  return 0;
}

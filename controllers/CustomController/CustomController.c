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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>


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

void motor_set_position_sync(double target, int delay) {
  const double DELTA = 0.001;  // max tolerated difference
  WbDeviceTag tag_sensor=wb_robot_get_device("right wheel sensor");
  wb_motor_set_position(left_motor, target);
  wb_motor_set_position(right_motor, target);
  wb_position_sensor_enable(tag_sensor, TIME_STEP);
  double effective;  // effective position
  do {
    if (wb_robot_step(TIME_STEP) == -1)
      break;
    delay -= TIME_STEP;
    effective = wb_position_sensor_get_value(tag_sensor);
  } while (fabs(target - effective) > DELTA && delay > 0);
  wb_position_sensor_disable(tag_sensor);
}



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

void braitenberg() {
  while (wb_robot_step(time_step) != -1) {  // Run simulation
    int i, j;
    double speed[2];
    double sensors_value[num_sensors];

    for (i = 0; i < num_sensors; i++)
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);

    /*
     * The Braitenberg algorithm is really simple, it simply computes the
     * speed of each wheel by summing the value of each sensor multiplied by
     * its corresponding weight. That is why each sensor must have a weight
     * for each wheel.
     */
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;

      for (j = 0; j < num_sensors; j++) {
        /*
         * We need to recenter the value of the sensor to be able to get
         * negative values too. This will allow the wheels to go
         * backward too.
         */
        speed[i] += matrix[j][i] * (1.0 - (sensors_value[j] / range));
      }

      speed[i] = BOUND(speed[i], -max_speed, max_speed);
    }

    /* Set the motor speeds */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
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

void rotateToHeading(WbDeviceTag tag_IMU,double speed,double heading_rad){
  wb_inertial_unit_enable(tag_IMU,10);
  step(0.020);
  const double error=0.01;
  double sauce_rad=*(wb_inertial_unit_get_roll_pitch_yaw(tag_IMU)+2);
  bool rotate_clockwise;
  bool sauceMoreThanHeading=sauce_rad>heading_rad;
  if(sauceMoreThanHeading){
    rotate_clockwise=fabs(sauce_rad-heading_rad)<fabs(sauce_rad-(heading_rad+2*M_PI));
  }else{
    rotate_clockwise=fabs(sauce_rad-heading_rad)>fabs(sauce_rad-(heading_rad-2*M_PI));
  }
  
  if(rotate_clockwise){
    wb_motor_set_velocity(left_motor, speed);
    wb_motor_set_velocity(right_motor, -speed);
  }else{
    wb_motor_set_velocity(left_motor, speed);
    wb_motor_set_velocity(right_motor, -speed);
  }
  
  
  while(fmin(fabs(sauce_rad-heading_rad),fabs(sauce_rad-(heading_rad-2*M_PI)))>error){
     wb_robot_step(time_step);
     sauce_rad=*(wb_inertial_unit_get_roll_pitch_yaw(tag_IMU)+2);
     
   printf("rotateToHeading: sauce_rad %f\n",sauce_rad);
    printf("rotateToHeading: abs %f\n",fabs(sauce_rad-heading_rad));
  }
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_inertial_unit_disable(tag_IMU);
  step(0.0);
  printf("finished\n");
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

void pick_up_brick() {
  //Place grip for placement
  moveArms(-2.8);
  moveFingers(-0.4);
  step(1.0);
  
  //move the grips into the cube space and grip
  moveForwards(1.0);
  step(1.0);
  stop(0.75);
  
  //pick it up
  moveFingers(0.6);
  step(1.0);
  moveArms(1.0);
  step(1.0);
}

void drop_brick() {
  moveArms(-1.3);
  step(1.0);
  
  moveFingers(-0.3);
  step(1.0);
  
  moveFingers(0.3);
  moveArms(1.3);
  step(1.0);
}
int main() {
  WbDeviceTag camera,IMU, left_motor, right_motor;
  int sampling_period=10;
  int width, height;
  int pause_counter = 0;
  int left_speed, right_speed;
  const char *color_names[3] = {"red", "green", "blue"};
  const char *ansi_colors[3] = {ANSI_COLOR_RED, ANSI_COLOR_GREEN, ANSI_COLOR_BLUE};
  const char *filenames[3] = {"red_blob.png", "green_blob.png", "blue_blob.png"};
  enum BLOB_TYPE current_blob;
  initialize();
  IMU=wb_robot_get_device("IMU");
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  /*
  const double* gps;
  while (wb_robot_step(TIME_STEP) != -1) {
    printf("test\n");
    const unsigned char *image = wb_camera_get_image(camera); 
    gps=wb_gps_get_values(GPS);
    printf("Gps_pos: x:%d,y:%d\n",gps[0],gps[1]);
    printf("%.100s\n", image);
  }//*/
  
  /*
  printf("Rotating\n");
  rotateToHeading(IMU,1.0,M_PI);
  printf("Rotating 2\n");
  rotateToHeading(IMU,1.0,0);
  // Move from boxes to the 
  printf("Walking\n");
  moveForwards(7.5);
  step(2.7);
  stop(1.0);
  printf("Picking up\n");
  pick_up_brick();
  printf("Rotating\n");
  rotateToHeading(IMU,2.4,M_PI);
  printf("Moving back\n");
  moveForwards(7.5);
  step(2.7);
  stop(1.0);
  printf("Dropping\n");
  drop_brick();
  //printf("Moving to heading 180\n");
  //rotateToHeading(IMU,2.4,M_PI);
  */
  return 0;
}
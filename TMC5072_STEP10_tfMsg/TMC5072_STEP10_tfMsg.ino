// Copyright 2024 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// clang-format off
// メッセージヘッダーファイルを見つけるため、micro_ros_arduino.hを先にインクルードすること

#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <stdio.h>
#include <tf2_msgs/msg/tf_message.h>
#include "stdlib.h"
#include "SPI.h"
#include "TMC5072.h"
// clang-format on

double spd_r, spd_l;

geometry_msgs__msg__Twist g_msg;
tf2_msgs__msg__TFMessage * g_tf_message;
sensor_msgs__msg__JointState g_jstate;
rosidl_runtime_c__String g_joint_name[2];
double g_positions[2];

rcl_subscription_t g_subscriber;
rcl_publisher_t g_publisher, g_publisher2;
rclc_executor_t g_executor;
rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;

#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41

#define MOTOR_EN 9

#define SPI_CS 3
#define SPI_CLK 46
#define SPI_MISO 47
#define SPI_MOSI 48

#define MIN_HZ 80
#define TIRE_DIAMETER (48.0)
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#define MIN_SPEED (MIN_HZ * PULSE)
#define TREAD_WIDTH (65.0)

hw_timer_t * g_timer0 = NULL;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

volatile double g_accel = 0.0;
volatile double g_speed = 0.0;

double g_max_omega;
double g_min_omega;
double g_acc_omega = 0.0;
double g_omega = 0;
double g_position_r, g_position_l;

double g_odom_x, g_odom_y, g_odom_theta;

t_TMC5072Mode g_TMC5072_mode;

//microROS function
#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      errorLoop();                 \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      errorLoop();                 \
    }                              \
  }

void errorLoop()
{
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(200);
  }
}

void eulerToQuat(float x, float y, float z, double * q)
{
  float c1 = cos(y / 2);
  float c2 = cos(z / 2);
  float c3 = cos(x / 2);

  float s1 = sin(y / 2);
  float s2 = sin(z / 2);
  float s3 = sin(x / 2);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

//twist message cb
void subscriptionCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * g_msg = (const geometry_msgs__msg__Twist *)msgin;

  //linearは[m/s]の単位で入力されるのでPi:Coのシステムに合わせて[mm/s]にする
  g_speed = g_msg->linear.x * 1000.0;
  g_omega = g_msg->angular.z;
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);  //motor enable
  TMC5072Init();
  digitalWrite(MOTOR_EN, HIGH);  //motor disable

  digitalWrite(LED1, HIGH);
  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);
  digitalWrite(LED2, HIGH);
  delay(2000);

  g_timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer0, &onTimer0, false);
  timerAlarmWrite(g_timer0, 1000, true);
  timerAlarmEnable(g_timer0);

  g_allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

  // create g_node
  RCCHECK(rclc_node_init_default(&g_node, "micro_ros_pico_node", "", &g_support));

  //g_publisher
  RCCHECK(rclc_publisher_init_default(
    &g_publisher, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf"));

  RCCHECK(rclc_publisher_init_default(
    &g_publisher2, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  //Subscriber
  RCCHECK(rclc_subscription_init_default(
    &g_subscriber, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  // create g_executor
  RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));
  RCCHECK(rclc_executor_add_subscription(
    &g_executor, &g_subscriber, &g_msg, &subscriptionCallback, ON_NEW_DATA));

  g_tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&g_tf_message->transforms, 1);

  g_tf_message->transforms.data[0].header.frame_id.data = "/odom";
  g_tf_message->transforms.data[0].header.frame_id.size =
    strlen(g_tf_message->transforms.data[0].header.frame_id.data);
  g_tf_message->transforms.data[0].header.frame_id.capacity = 100;

  g_tf_message->transforms.data[0].child_frame_id.data = "/base_footprint";
  g_tf_message->transforms.data[0].child_frame_id.size =
    strlen(g_tf_message->transforms.data[0].child_frame_id.data);
  g_tf_message->transforms.data[0].child_frame_id.capacity = 100;

  g_joint_name[0].data = "left_wheel_joint";
  g_joint_name[0].size = strlen(g_joint_name[0].data);
  g_joint_name[0].capacity = g_joint_name[0].size + 1;

  g_joint_name[1].data = "right_wheel_joint";
  g_joint_name[1].size = strlen(g_joint_name[1].data);
  g_joint_name[1].capacity = g_joint_name[1].size + 1;

  g_jstate.name.data = g_joint_name;
  g_jstate.name.size = 2;
  g_jstate.name.capacity = 2;
  g_jstate.position.data = g_positions;
  g_jstate.position.size = 2;
  g_jstate.position.capacity = 2;

  digitalWrite(MOTOR_EN, LOW);  //motor enable
}

void loop()
{
  double q[4];
  uint32_t current = micros();

  g_jstate.header.stamp.sec = current / 1000000;
  g_jstate.header.stamp.nanosec = current - g_jstate.header.stamp.sec * 1000000;

  eulerToQuat(0, 0, g_odom_theta, q);
  g_tf_message->transforms.data[0].transform.translation.x = g_odom_x;
  g_tf_message->transforms.data[0].transform.translation.y = g_odom_y;
  g_tf_message->transforms.data[0].transform.translation.z = 0.0;

  g_tf_message->transforms.data[0].transform.rotation.x = (double)q[1];
  g_tf_message->transforms.data[0].transform.rotation.y = (double)q[2];
  g_tf_message->transforms.data[0].transform.rotation.z = (double)q[3];
  g_tf_message->transforms.data[0].transform.rotation.w = (double)q[0];
  g_tf_message->transforms.data[0].header.stamp.nanosec = g_jstate.header.stamp.nanosec;
  g_tf_message->transforms.data[0].header.stamp.sec = g_jstate.header.stamp.sec;

  g_jstate.position.data[0] = g_position_l;
  g_jstate.position.data[1] = g_position_r;

  RCSOFTCHECK(rcl_publish(&g_publisher, g_tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&g_publisher2, &g_jstate, NULL));

  RCCHECK(rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(10)));  //for subscription

  if (spd_r > 0) {
    TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
  } else {
    TMC5072Write(TMC5072_RAMPMODE1, 0x02);  //velocity mode(negative)
  }
  TMC5072Write(TMC5072_VMAX1, abs((int)spd_r) / TMC5072_VELOCITY);

  if (spd_l > 0) {
    TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)
  } else {
    TMC5072Write(TMC5072_RAMPMODE2, 0x02);  //velocity mode(negative)
  }
  TMC5072Write(TMC5072_VMAX2, abs((int)spd_l) / TMC5072_VELOCITY);
}

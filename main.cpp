#include <Arduino.h>
#include "WiFi.h"

#include <M5StickCPlus.h>

#include "bugC.h"
// #include "Ultrasonic.h"

// Ultrasonic ultrasonic(7);

// microros ===================================================================
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif

rclc_executor_t executor;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

BUGC BugC;

int move_flag = 0;

#define LED_PIN 10

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

double getBatteryLevel()
{
  uint16_t vbatData = M5.Axp.GetVbatData();
  double vbat = vbatData * 1.1 / 1000;
  return 100.0 * ((vbat - 3.0) / (4.07 - 3.0));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  // if 0 -> stop
  if (move_flag == 0)
  {
    BugC.BugCSetAllSpeed(0, 0, 0, 0);
    BugC.BugCSetColor(0x000000, 0x000000);
  }
  move_flag = 0;
}

// twist message cb
void subscription_callback(const void *msgin)
{
  move_flag = 1;
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float x = msg->linear.x;
  float y = msg->linear.y;
  float z = msg->angular.z;

  int spd0, spd1, spd2, spd3 = 0;

  // float bat_level = getBatteryLevel();

  if (abs(x) < 0.2 && abs(y) < 0.2 && abs(z) < 0.2)
  {
    BugC.BugCSetAllSpeed(0, 0, 0, 0);
    BugC.BugCSetColor(0x000200, 0x000200);
    return;
  }
  else
  {
    spd0 = (int)((-y + x - z)* 33);
    spd1 = (int)((-y - x - z)* 33);
    spd2 = (int)((y + x - z)* 33);
    spd3 = (int)((y - x - z)* 33);

    BugC.BugCSetAllSpeed(spd0, spd1, spd2, spd3);
    BugC.BugCSetColor(0x020000, 0x020000);
  }

  
  M5.Lcd.fillRect(0, 0, M5.Lcd.width(), M5.Lcd.height(), BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("x:%d, y:%d, z:%d", (int)x, (int)y, (int)z);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("spd0:%d, spd1:%d, spd2:%d, spd3:%d", spd0, spd1, spd2, spd3);
  // M5.Lcd.setCursor(0, 40);
  // M5.Lcd.printf("bat:%f", bat_level);
  // update
  M5.update();
}

void setup()
{
  M5.begin();
  BugC.Init();

  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.drawCentreString("BUGC example", 80, 30, 2);

  // initialize the ROS2 ======================================================
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  // set_microros_wifi_transports("ssid", "pass", "192.168.10.100", 2000);
  // set_microros_transports();
  delay(2000);
  allocator = rcl_get_default_allocator();

  float bat_level = getBatteryLevel();
  M5.Lcd.drawCentreString("WiFi connect!", 80, 60, 2);
  M5.Lcd.drawCentreString("Battery: " + String(bat_level) + "%", 80, 80, 2);
  
  // rclc init ================================================================
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "realturtlebot", "", &support);
  // sub init
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "turtle1/cmd_vel");

  // timer init
  const unsigned int timer_timeout = 200;
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);
  
  // rclc_executor_init =======================================================
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  rclc_executor_add_timer(&executor, &timer); // timer
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
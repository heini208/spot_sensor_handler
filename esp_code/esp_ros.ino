#include <Arduino.h>
#include <Wire.h>
// BMM libraries
#include "bmm150.h"
#include "bmm150_defs.h"
//ROS libraries
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ST7789.h>

// ==== Display-Pinbelegung ====
#define TFT_CS     5
#define TFT_RST    16
#define TFT_DC     19
#define TFT_MOSI   23
#define TFT_SCLK   18

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// ---- micro-ROS globals ----
rcl_publisher_t pub_x, pub_y, pub_z;
std_msgs__msg__Int32 msg_x, msg_y, msg_z;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) { delay(100); }
}

// ==== BMM globals ====
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;


void calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if(value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    }
    else if(value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if(value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    }
    else if(value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if(value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    }
    else if(value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }

    Serial.print(".");
    delay(100);

  }

  value_offset.x = value_x_min + (value_x_max - value_x_min)/2;
  value_offset.y = value_y_min + (value_y_max - value_y_min)/2;
  value_offset.z = value_z_min + (value_z_max - value_z_min)/2;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting ESP32...");
  delay(1000);


  // --- micro-ROS WiFi transport ---
  set_microros_wifi_transports(
    "spot-BD-40920003",
    "enter password here",
    "enter ip here", // <--- YOUR agent PC IP!
    8888
  );
  delay(2000);


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_bmm150_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_x, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "bmm150_x"));
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_y, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "bmm150_y"));
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_z, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "bmm150_z"));
  Serial.println("RCCHECK done...");


  if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
  }

  Serial.println("Start figure-8 calibration after 3 seconds.");
  delay(3000);
  calibrate(10000);
  Serial.print("\n\rCalibrate done..");

  // ==== Display initialisieren ====
  tft.initR(INITR_GREENTAB);      // Init ST7735S chip, green tab
  tft.setSPISpeed(40000000);
  tft.setRotation(1);         // Optional: Querformat
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  }



/**
 * @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
 * @param timeout - seconds of calibration period.
*/


void loop()
{
  bmm150_mag_data value;
  bmm.read_mag_data();

  value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
  value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
  value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;

  float xyHeading = atan2(value.x, value.y);
  float zxHeading = atan2(value.z, value.x);
  float heading = xyHeading;

  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI;
  float xyHeadingDegrees = xyHeading * 180 / M_PI;
  float zxHeadingDegrees = zxHeading * 180 / M_PI;

  //Serial.print("Heading: ");
  //Serial.println(headingDegrees);
  Serial.print("xValue: ");
  Serial.println(value.x);
  Serial.print("yValue: ");
  Serial.println(value.y);
  Serial.print("zValue: ");
  Serial.println(value.z);
  Serial.println(" ");
  Serial.println("------------------------------------------");
  Serial.println(" ");

// ==== Anzeige vorbereiten ====
  //tft.fillScreen(ST77XX_BLACK);
  //tft.fillRect(30, 10, 120, 60, ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);

  // ==== X-Wert anzeigen ====
  tft.fillRect(30, 10, 70, 16, ST77XX_BLACK);
  tft.setCursor(0, 10);
  tft.print("X: ");
  tft.println(value.x);

  // ==== Y-Wert anzeigen ====
  tft.fillRect(30, 26, 70, 16, ST77XX_BLACK);
  tft.print("Y: ");
  tft.println(value.y);

  // ==== Z-Wert anzeigen ====
  tft.fillRect(30, 42, 90, 16, ST77XX_BLACK);
  tft.print("Z: ");
  tft.println(value.z);

  //delay(20);


// ==== Daten senden ====
  msg_x.data = value.x;
  msg_y.data = value.y;
  msg_z.data = value.z;

  RCSOFTCHECK(rcl_publish(&pub_x, &msg_x, NULL));
  RCSOFTCHECK(rcl_publish(&pub_y, &msg_y, NULL));
  RCSOFTCHECK(rcl_publish(&pub_z, &msg_z, NULL));
}

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Fill and don't commit ;-)

#define LEDC_CHANNEL_MOTOR1             0
#define LEDC_CHANNEL_MOTOR2             1
#define LEDC_CHANNEL_RESOLUTION_12_BIT  12
#define LEDC_CHANNEL_DUTY               2048 // (2 ^ LEDC_CHANNEL_RESOLUTION_12_BIT) / 2 
#define LEDC_CHANNEL_DEFAULT_SPEED      200

#define PIN_STEPPER_EN  27
#define PIN_STEP_SELECT_M0 26
#define PIN_STEP_SELECT_M1 25
#define PIN_STEP_SELECT_M2 33

#define PIN_DIR1    4
#define PIN_STEP1   16
#define PIN_DIR2    15
#define PIN_STEP2   2

#define PIN_LED1    19
#define PIN_LED2    18

#define PIN_BUTTON1 ERROR("Pin 6 can't be used")
#define PIN_BUTTON2 ERROR("Pin 7 can't be used")

#define PIN_ACC_INT 23

#define LEFT      0
#define RIGHT     1

Adafruit_MPU6050 mpu;

void setup_ota() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Connected to wifi");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_mpu6050() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  // setup MPU
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
}

void setup_gpio() {
  // Setup pins for stepper driver
  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);

  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);

  pinMode(PIN_STEP1, OUTPUT);
  pinMode(PIN_STEP2, OUTPUT);

  ledcSetup(LEDC_CHANNEL_MOTOR1, LEDC_CHANNEL_DEFAULT_SPEED, LEDC_CHANNEL_RESOLUTION_12_BIT);
  ledcSetup(LEDC_CHANNEL_MOTOR2, LEDC_CHANNEL_DEFAULT_SPEED, LEDC_CHANNEL_RESOLUTION_12_BIT);

  ledcAttachPin(PIN_LED1, LEDC_CHANNEL_MOTOR1);
  ledcAttachPin(PIN_STEP1, LEDC_CHANNEL_MOTOR1);
  ledcAttachPin(PIN_STEP2, LEDC_CHANNEL_MOTOR2);

  ledcWrite(LEDC_CHANNEL_MOTOR1, LEDC_CHANNEL_DUTY);
  ledcWrite(LEDC_CHANNEL_MOTOR2, LEDC_CHANNEL_DUTY);

  pinMode(PIN_STEP_SELECT_M0, OUTPUT);
  pinMode(PIN_STEP_SELECT_M1, OUTPUT);
  pinMode(PIN_STEP_SELECT_M2, OUTPUT);
  digitalWrite(PIN_STEP_SELECT_M0, LOW);
  digitalWrite(PIN_STEP_SELECT_M1, LOW);
  digitalWrite(PIN_STEP_SELECT_M2, LOW); 

  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, HIGH); 
}

void setup() {
  Serial.begin(115200);

  setup_gpio();
  setup_ota();
  setup_mpu6050();
}

void print_step_stuff(unsigned int time_in_ms, unsigned int steps_per_second, unsigned int direction) {
  Serial.print("Starting rotation for ");
  Serial.print(time_in_ms);
  Serial.print("ms to ");
  if (direction != LEFT) {
    Serial.print("right ");
  } else {
    Serial.print("left ");
  }
  Serial.print("at ");
  Serial.print(steps_per_second);
  Serial.println(" steps/s");
}

void enable_motors() {
  digitalWrite(PIN_STEPPER_EN, LOW);
  digitalWrite(PIN_LED2, LOW);
}

void disable_motors() {
  digitalWrite(PIN_STEPPER_EN, HIGH);
  digitalWrite(PIN_LED2, HIGH);
}

void rotate(unsigned int time_in_ms, unsigned int steps_per_second, unsigned int direction) {
  print_step_stuff(time_in_ms, steps_per_second, direction);

  digitalWrite(PIN_DIR1, direction == LEFT);
  digitalWrite(PIN_DIR2, direction != LEFT);

  ledcChangeFrequency(LEDC_CHANNEL_MOTOR1, steps_per_second, LEDC_CHANNEL_RESOLUTION_12_BIT);
  ledcChangeFrequency(LEDC_CHANNEL_MOTOR2, steps_per_second / 8, LEDC_CHANNEL_RESOLUTION_12_BIT);

  enable_motors();

  delay(time_in_ms);

  disable_motors();

  Serial.println("Stopped rotating");
}

void loop() {
  // put your main code here, to run repeatedly
  int steps = 200;
  ArduinoOTA.handle();

  rotate(2000, LEDC_CHANNEL_DEFAULT_SPEED, RIGHT);
  delay(2000);

  rotate(2000, LEDC_CHANNEL_DEFAULT_SPEED, LEFT);
  delay(2000);


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

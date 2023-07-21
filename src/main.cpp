#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <math.h>

// Fill and don't commit ;-)
const char* ssid = "ssid";
const char* password = "pw";

#define LEDC_CHANNEL_MOTOR1               1
#define LEDC_CHANNEL_MOTOR2               2
#define LEDC_CHANNEL_RESOLUTION_12_BIT    12
#define LEDC_CHANNEL_DUTY                 2048 // (2 ^ LEDC_CHANNEL_RESOLUTION_12_BIT) / 2
#define LEDC_CHANNEL_DEFAULT_SPEED        200

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

/* Stop OTA after timeout */
#define OTA_TIMEOUT 15000

/* Permanent Z offset, set at boot */
float base_z = 0.00;

/* Controller setup */
float kP = 20000;

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
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
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

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
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
  digitalWrite(PIN_STEP_SELECT_M2, HIGH);

  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, HIGH);
}

void setup() {
  Serial.begin(115200);

  setup_gpio();
  setup_ota();
  setup_mpu6050();
}

void print_step_stuff(unsigned int steps_per_second, unsigned int direction) {
  Serial.print("Starting rotation to ");
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

void rotate(unsigned int steps_per_second, unsigned int direction) {
  uint32_t error, freq;
  uint8_t channel;

  if (!steps_per_second) {
      /* PWM can't handle freq of 0 */
#ifdef DEBUG
      Serial.println("steps_per_second is zero");
#endif
      return;
  }

  print_step_stuff(steps_per_second, direction);

  digitalWrite(PIN_DIR1, direction == LEFT);
  digitalWrite(PIN_DIR2, direction != LEFT);

  for(channel = 1; channel <= LEDC_CHANNEL_MOTOR2; channel++) {
    error = ledcChangeFrequency(channel, steps_per_second, LEDC_CHANNEL_RESOLUTION_12_BIT);
    if (!error) {
      Serial.print("Error on line ");
      Serial.println(__LINE__);
    } else {
#ifdef DEBUG
      Serial.print("Frequency on ");
      Serial.print(channel);
      Serial.print(" set to ");
      Serial.println(error);
#endif
    };
  }

  enable_motors();
}

float get_z_angle() {
  float z_angle, z_real;
  sensors_event_t a, g, temp;

  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  /* Calculate a z angle */
  z_angle = atan(sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y) / a.acceleration.z);

  /* Set a "base" orientation at startup */
  if (base_z <= 0.01 && base_z >= -0.01) {
    base_z = z_angle;
  }

  /* Rotate vector to have 0 pointing "upwards" with a range of -Pi/2 to Pi/2 */
  if (z_angle - base_z < -M_PI_2) {
    z_real = z_angle - base_z + M_PI;
  } else if (z_angle - base_z > M_PI_2) {
    z_real = z_angle - base_z - M_PI;
  } else {
    z_real = z_angle - base_z;
  }

#ifdef DEBUG
  Serial.print("z 'real': ");
  if (z_real < 0) { Serial.print("-"); } else { Serial.print(" "); };
  Serial.println(abs(z_real), 6);
#endif

  return z_real;
}

int control_function(float z_angle) {
  int steps_per_s = int32_t(kP * (z_angle));
  return steps_per_s;
}

void check_control_loop_speed() {
  static int cnt;
  static unsigned long current_second;
  cnt++;

  if (millis() / 1000 > current_second) {
    current_second = millis() / 1000;
    Serial.print("Last second had "); Serial.print(cnt); Serial.println("loops.");
    cnt = 0;
  }
}

void loop() {
  int32_t steps_per_s;
  float angle;

  check_control_loop_speed();

  if (millis() < OTA_TIMEOUT) {
    ArduinoOTA.handle();
  }

  angle = get_z_angle();

  steps_per_s = control_function(angle);

  rotate(abs(steps_per_s), steps_per_s < 0 ? RIGHT : LEFT);
}

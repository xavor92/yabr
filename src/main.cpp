#include <Arduino.h>

#define PIN_DIR   4
#define PIN_STEP  16

#define LEFT      0
#define RIGHT     1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
}

void print_step_stuff(unsigned int steps, unsigned int direction) {
  Serial.print("Starting location for ");
  Serial.print(steps);
  Serial.print(" to ");
  if (direction != LEFT) {
    Serial.println("right");
  } else {
    Serial.println("left");
  }
}

void rotate(unsigned int steps, unsigned int direction) {
  print_step_stuff(steps, direction);

  digitalWrite(PIN_DIR, direction == LEFT);

  for (int i = 0; i < steps; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delay(1);
    digitalWrite(PIN_STEP, LOW);
    delay(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly
  int dir = 1;
  int steps = 200;
  
  rotate(steps, RIGHT);
  delay(1000);
  rotate(steps, LEFT);
  delay(1000);
}

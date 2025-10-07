#include <Wire.h>
#include <Adafruit_MotorShield.h>

int MOTOR_LEFT_PIN = 1;
int MOTOR_RIGHT_PIN = 2;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor_left = AFMS.getMotor(MOTOR_LEFT_PIN);
Adafruit_DCMotor *motor_right = AFMS.getMotor(MOTOR_RIGHT_PIN);

void setup() {
  AFMS.begin();
}

void loop() {
    // Motor, please spin
    motor_left->setSpeed(80);
    motor_left->run(FORWARD);

    motor_right->setSpeed(80);
    motor_right->run(BACKWARD);
    delay(1000);
}

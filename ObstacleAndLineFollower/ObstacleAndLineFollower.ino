#include <Servo.h>

const int ULTRASONIC_TRIGGER_PIN = 19;
const int ULTRASONIC_ECHO_PIN = 18;

const int MOTOR1_IN1 = 7;
const int MOTOR1_IN2 = 8;
const int MOTOR1_ENA = 5;  // PWM

const int MOTOR2_IN3 = 9;
const int MOTOR2_IN4 = 11;
const int MOTOR2_ENB = 6;  // PWM

const int LINE_FOLLOWING_SENSOR_LEFT = 2;
const int LINE_FOLLOWING_SENSOR_MIDDLE = 4;
const int LINE_FOLLOWING_SENSOR_RIGHT = 10;

Servo servo;

void setup() {
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);
  pinMode(LINE_FOLLOWING_SENSOR_LEFT, INPUT);
  pinMode(LINE_FOLLOWING_SENSOR_MIDDLE, INPUT);
  pinMode(LINE_FOLLOWING_SENSOR_RIGHT, INPUT);

  Serial.begin(9600);
  servo.attach(10);
}

long calculateUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

  if (duration == 0)
    return -1;

  return (duration * 0.034 / 2);
}

void moveForwardWithSpeed(int speed) {
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, speed);

  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, speed);
}

void moveBackwardWithSpeed(int speed) {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  analogWrite(MOTOR1_ENA, speed);

  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  analogWrite(MOTOR2_ENB, speed);
}

void turnLeftWithSpeed(int speed) {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  analogWrite(MOTOR1_ENA, speed);

  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, speed);
}

void turnRightWithSpeed(int speed) {
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, speed);

  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  analogWrite(MOTOR2_ENB, speed);
}

void stopCar() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 0);

  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, 0);
}

void avoidObstacle() {
  stopCar();
  moveBackwardWithSpeed(200); 
  delay(500);
  turnRightWithSpeed(200);  
  delay(500);
  stopCar();
}

void followLine(int leftLineFollowerValue, int middleLineFollowerValue, int rightLineFollowerValue) {
  if (leftLineFollowerValue == LOW && middleLineFollowerValue == HIGH && rightLineFollowerValue == LOW) {
    moveForwardWithSpeed(255);  
  } else if (leftLineFollowerValue == HIGH && middleLineFollowerValue == LOW && rightLineFollowerValue == LOW) {
    turnRightWithSpeed(200);  
  } else if (leftLineFollowerValue == LOW && middleLineFollowerValue == LOW && rightLineFollowerValue == HIGH) {
    turnLeftWithSpeed(200);
  } else {
    stopCar();
  }
}

void loop() {
  long distanceToObstacle = calculateUltrasonicDistance();
  int leftLineFollowerSensorValue = digitalRead(LINE_FOLLOWING_SENSOR_LEFT);
  int middleLineFollowerSensorValue = digitalRead(LINE_FOLLOWING_SENSOR_MIDDLE);
  int rightLineFollowerSensorValue = digitalRead(LINE_FOLLOWING_SENSOR_RIGHT);

  Serial.print("Distance: ");
  Serial.print(distanceToObstacle);
  Serial.print(" cm, Left: ");
  Serial.print(leftLineFollowerSensorValue);
  Serial.print(", Middle: ");
  Serial.print(middleLineFollowerSensorValue);
  Serial.print(", Right: ");
  Serial.println(rightLineFollowerSensorValue);

  if (distanceToObstacle < 20)
    avoidObstacle();
  else
    followLine(leftLineFollowerSensorValue,
               middleLineFollowerSensorValue,
               rightLineFollowerSensorValue);

  delay(100);
}

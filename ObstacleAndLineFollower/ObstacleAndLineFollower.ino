#include <Servo.h>

const int ULTRASONIC_TRIGGER_PIN = 19;
const int ULTRASONIC_ECHO_PIN = 18;

const int RIGHT_MOTOR_IN1 = 11;
const int RIGHT_MOTOR_IN2 = 9;
const int RIGHT_MOTOR_ENA = 6;

const int LEFT_MOTOR_IN3 = 8;
const int LEFT_MOTOR_IN4 = 7;
const int LEFT_MOTOR_ENB = 5;

const int LINE_FOLLOWING_SENSOR_LEFT = 2;
const int LINE_FOLLOWING_SENSOR_MIDDLE = 4;
const int LINE_FOLLOWING_SENSOR_RIGHT = 10;

Servo servo;

void setup()
{
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_ENA, OUTPUT);
  pinMode(LEFT_MOTOR_IN3, OUTPUT);
  pinMode(LEFT_MOTOR_IN4, OUTPUT);
  pinMode(LEFT_MOTOR_ENB, OUTPUT);
  pinMode(LINE_FOLLOWING_SENSOR_LEFT, INPUT);
  pinMode(LINE_FOLLOWING_SENSOR_MIDDLE, INPUT);
  pinMode(LINE_FOLLOWING_SENSOR_RIGHT, INPUT);

  Serial.begin(9600);
  servo.attach(3);
  setServoToCenter();
}

void loop()
{
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

  if (distanceToObstacle > 0 && distanceToObstacle < 20)
  {
    avoidObstacle();
  }
  else
  {
    setServoToCenter();
    moveForwardWithSpeed(255);
    Serial.println("Moving forward");
  }

  delay(100);
}

long calculateUltrasonicDistance()
{
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);

  if (duration == 0)
  {
    Serial.println("No obstacle detected (timeout).");
    return 999;
  }

  return duration * 0.034 / 2;
}

void avoidObstacle()
{
  stopCar();

  long distanceFront = checkDistanceAtAngle(90);
  long distanceLeft = checkDistanceAtAngle(45);
  long distanceRight = checkDistanceAtAngle(135);

  Serial.print("Distances - Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right: ");
  Serial.println(distanceRight);

  if (distanceFront > 20)
  {
    moveForwardWithSpeed(200);
    delay(300);
  }
  else if (distanceLeft > distanceRight && distanceLeft > 15)
  {
    turnLeftWithSpeed(200);
    delay(500);
  }
  else if (distanceRight > distanceLeft && distanceRight > 15)
  {
    turnRightWithSpeed(200);
    delay(500);
  }
  else
  {
    moveBackwardWithSpeed(200);
    delay(300);
    turnLeftWithSpeed(200);
    delay(500);
  }

  stopCar();
}

long checkDistanceAtAngle(int angle)
{
  servo.write(angle);
  delay(500);
  long distance = calculateUltrasonicDistance();
  setServoToCenter();
  return distance;
}

void setServoToCenter()
{
  servo.write(90);
}

void moveForwardWithSpeed(int speed)
{
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, HIGH);
  digitalWrite(LEFT_MOTOR_IN4, LOW);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void moveBackwardWithSpeed(int speed)
{
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, LOW);
  digitalWrite(LEFT_MOTOR_IN4, HIGH);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void turnLeftWithSpeed(int speed)
{
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, LOW);
  digitalWrite(LEFT_MOTOR_IN4, HIGH);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void turnRightWithSpeed(int speed)
{
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, HIGH);
  digitalWrite(LEFT_MOTOR_IN4, LOW);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void stopCar()
{
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_ENA, 0);

  digitalWrite(LEFT_MOTOR_IN3, LOW);
  digitalWrite(LEFT_MOTOR_IN4, LOW);
  analogWrite(LEFT_MOTOR_ENB, 0);
}
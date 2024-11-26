#include <Servo.h>

const int TRIG_PIN = 3;
const int ECHO_PIN = 4;
const int MOTOR1_PIN1 = 6;
const int MOTOR1_PIN2 = 7;
const int MOTOR2_PIN1 = 8;
const int MOTOR2_PIN2 = 9;
const int SENSOR_LEFT = 2;
const int SENSOR_MIDDLE = 4;
const int SENSOR_RIGHT = 10;

Servo servo;

void setup()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MIDDLE, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  Serial.begin(9600);
  servo.attach(10);
}

long calculateUltrasonicDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return (duration * 0.034 / 2);
}

void moveForward()
{
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void moveBackward()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
}

void turnLeft()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnRight()
{
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
}

void stopCar()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void avoidObstacle()
{
  stopCar();
  moveBackward();
  delay(500);
  turnRight();
  delay(500);
  stopCar();
}

void followLine(int leftLineFollowerValue, int middleLineFollowerValue, int rightLineFollowerValue)
{
  if (leftLineFollowerValue == LOW && middleLineFollowerValue == HIGH && rightLineFollowerValue == LOW)
  {
    moveForward();
  }
  else if (leftLineFollowerValue == HIGH && middleLineFollowerValue == LOW && rightLineFollowerValue == LOW)
  {
    turnRight();
  }
  else if (leftLineFollowerValue == LOW && middleLineFollowerValue == LOW && rightLineFollowerValue == HIGH)
  {
    turnLeft();
  }

  stopCar();
}

void loop()
{
  long distanceToObstacle = calculateUltrasonicDistance();
  int leftLineFollowerSensorValue = digitalRead(SENSOR_LEFT);
  int middleLineFollowerSensorValue = digitalRead(SENSOR_MIDDLE);
  int rightLineFollowerSensorValue = digitalRead(SENSOR_RIGHT);

  Serial.print("Distance: ");
  Serial.print(distanceToObstacle);
  Serial.print(" cm, Left: ");
  Serial.print(leftLineFollowerSensorValue);
  Serial.print(", Middle: ");
  Serial.print(middleLineFollowerSensorValue);
  Serial.print(", Right: ");
  Serial.println(rightLineFollowerSensorValue);

  moveForward();

  if (distanceToObstacle < 20)
    avoidObstacle();

  else
    followLine(leftLineFollowerSensorValue,
               middleLineFollowerSensorValue,
               rightLineFollowerSensorValue);

  delay(100);
}
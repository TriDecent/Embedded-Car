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

void setup() {
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
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, HIGH);
  digitalWrite(LEFT_MOTOR_IN4, LOW);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void moveBackwardWithSpeed(int speed) {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, LOW);
  digitalWrite(LEFT_MOTOR_IN4, HIGH);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void turnLeftWithSpeed(int speed) {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, HIGH);
  digitalWrite(LEFT_MOTOR_IN4, LOW);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void turnRightWithSpeed(int speed) {
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_ENA, speed);

  digitalWrite(LEFT_MOTOR_IN3, LOW);
  digitalWrite(LEFT_MOTOR_IN4, HIGH);
  analogWrite(LEFT_MOTOR_ENB, speed);
}

void stopCar() {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_ENA, 0);

  digitalWrite(LEFT_MOTOR_IN3, LOW);
  digitalWrite(LEFT_MOTOR_IN4, LOW);
  analogWrite(LEFT_MOTOR_ENB, 0);
}

long checkDistanceAtAngle(int angle) {
  servo.write(angle);
  delay(500);  // Đợi servo di chuyển đến góc mới
  return calculateUltrasonicDistance();
}

void avoidObstacle() {
  stopCar();

  // Kiểm tra khoảng cách tại các góc trái, phải, và phía trước
  long distanceFront = checkDistanceAtAngle(90);   // Phía trước
  long distanceLeft = checkDistanceAtAngle(45);    // Trái
  long distanceRight = checkDistanceAtAngle(135);  // Phải

  Serial.print("Distances - Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right: ");
  Serial.println(distanceRight);

  if (distanceFront > 20) {
    moveForwardWithSpeed(200);  // Tiến lên nếu phía trước không có chướng ngại vật
    delay(300);
  } else if (distanceLeft > distanceRight && distanceLeft > 15) {
    turnLeftWithSpeed(200);  // Quay trái nếu trái rộng hơn
    delay(500);
    moveForwardWithSpeed(255);
  } else if (distanceRight > distanceLeft && distanceRight > 15) {
    turnRightWithSpeed(200);  // Quay phải nếu phải rộng hơn
    delay(500);
    moveForwardWithSpeed(255);
  } else {
    moveBackwardWithSpeed(200);  // Lùi nếu cả hai bên đều hẹp
    delay(300);
    turnLeftWithSpeed(200);  // Quay ngẫu nhiên (ở đây là quay trái) để thử tìm hướng mới
    delay(500);
  }

  stopCar();  // Dừng xe sau khi xử lý né cản
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

  moveForwardWithSpeed(255);
  if (distanceToObstacle < 20) {
    avoidObstacle();  // Gọi hàm né cản nếu vật cản gần
  }
  // else
  // {
  //   followLine(leftLineFollowerSensorValue, middleLineFollowerSensorValue, rightLineFollowerSensorValue); // Dò line nếu không có vật cản
  // }

  delay(100);
}

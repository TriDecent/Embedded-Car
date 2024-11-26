const int MOTOR1_IN1 = 7;
const int MOTOR1_IN2 = 8;
const int MOTOR1_ENA = 5;

const int MOTOR2_IN3 = 9;
const int MOTOR2_IN4 = 11;
const int MOTOR2_ENB = 6;

void setup() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  Serial.println("Motor 1 Forward");
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 255); // Max speed
  delay(2000); // Run motor1 in 2 seconds

  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 0);
  delay(2000); // Stop for 2 seconds

  Serial.println("Motor 2 Forward");
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, 255); // Max speed
  delay(2000); // Run motor1 in 2 seconds

  // Dừng động cơ 2
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, 0);
  delay(2000); // Stop for 2 seconds
}

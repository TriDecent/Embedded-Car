const int MOTOR1_IN1 = 11;
const int MOTOR1_IN2 = 9;
const int MOTOR1_ENA = 6;

const int MOTOR2_IN3 = 8;
const int MOTOR2_IN4 = 7;
const int MOTOR2_ENB = 5;

void setup()
{
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  // Motor 1 testing
  Serial.println("Testing Motor 1 - Forward (Right Wheel)");
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 255); // Max speed
  delay(3000);                  // Run motor1 for 3 seconds

  Serial.println("Testing Motor 1 - Backward (Right Wheel)");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  analogWrite(MOTOR1_ENA, 255); // Max speed
  delay(3000);                  // Run motor1 in reverse for 3 seconds

  // Stop Motor 1
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 0);
  delay(2000); // Stop for 2 seconds

  // Motor 2 testing
  Serial.println("Testing Motor 2 - Forward (Left Wheel)");
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, 255); // Max speed
  delay(3000);                  // Run motor2 for 3 seconds

  Serial.println("Testing Motor 2 - Backward (Left Wheel)");
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  analogWrite(MOTOR2_ENB, 255); // Max speed
  delay(3000);                  // Run motor2 in reverse for 3 seconds

  // Stop Motor 2
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_ENB, 0);
  delay(2000); // Stop for 2 seconds
}

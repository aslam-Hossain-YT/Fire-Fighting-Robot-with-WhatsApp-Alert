// ----- Pin Definitions -----
#define IN1 27  // Motor A IN1
#define IN2 26  // Motor A IN2
#define IN3 25  // Motor B IN3
#define IN4 33  // Motor B IN4

#define ENA 14  // Motor A Enable (PWM)
#define ENB 32  // Motor B Enable (PWM)

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  // Motor A Forward, Motor B Forward
  Serial.println("Motors Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200);  // Speed (0-255)
  analogWrite(ENB, 200);
  delay(3000);

  // Stop Motors
  Serial.println("Motors Stop");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(2000);
}

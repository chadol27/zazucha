#include <Servo.h>
#include <Adafruit_NeoPixel.h>

const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4; 
const int IN4 = 5;
const int ENA = 6;
const int ENB = 11;
const int LED_PIN = 12;
const int trigPin = 8;
const int echoPin = 7;
const int servoPin = 13;

const int NUMPIXELS = 8;
const int MAX_SPEED = 255;
const int DRIVE_SPEED = 125;
const int TURN_SPEED = 175;
const unsigned long DISTANCE_INTERVAL = 100;
const unsigned long SERVO_INTERVAL = 40;

unsigned long lastDistanceTime = 0;
unsigned long lastServoTime = 0;
int servoAngle = 90;
int servoStep = 5;

Servo radarServo;
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  initControls();
}

void loop() {
  runServoSweep();

  while (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'w':
      case 'W':
        motorForward(DRIVE_SPEED);
        break;
      case 's':
      case 'S':
        motorBackward(DRIVE_SPEED);
        break;
      case 'a':
      case 'A':
        motorLeft(TURN_SPEED);
        break;
      case 'd':
      case 'D':
        motorRight(TURN_SPEED);
        break;
      case 'x':
      case 'X':
        stopMotors();
        break;
      case '0':
        clearLED();
        break;
      case '1':
        setLEDAll(150, 0, 0);
        break;
      case '2':
        setLEDAll(150, 100, 0);
        break;
      case '3':
        setLEDAll(0, 150, 0);
        break;
      case '4':
        setLEDAll(0, 0, 150);
        break;
      case '5':
        setLEDAll(150, 150, 150);
        break;
    }
  }

  if (millis() - lastDistanceTime >= DISTANCE_INTERVAL) {
    lastDistanceTime = millis();
    int distance = readDistanceCm();

    Serial.print("{\"distance\":");
    Serial.print(distance);
    Serial.println("}");
  }
}

void initControls() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  radarServo.attach(servoPin);
  radarServo.write(servoAngle);

  pixels.begin();
  clearLED();
  stopMotors();
}

void runServoSweep() {
  if (millis() - lastServoTime < SERVO_INTERVAL) {
    return;
  }

  lastServoTime = millis();
  servoAngle += servoStep;

  if (servoAngle >= 150 || servoAngle <= 30) {
    servoStep = -servoStep;
  }

  radarServo.write(servoAngle);
}

void motorForward(int speed) {
  int motorSpeed = constrain(speed, 0, MAX_SPEED);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void motorBackward(int speed) {
  int motorSpeed = constrain(speed, 0, MAX_SPEED);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void motorLeft(int speed) {
  int motorSpeed = constrain(speed, 0, MAX_SPEED);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void motorRight(int speed) {
  int motorSpeed = constrain(speed, 0, MAX_SPEED);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

int readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) {
    return 0;
  }

  return duration * 0.034 / 2;
}

void setLEDAll(int r, int g, int b) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void clearLED() {
  pixels.clear();
  pixels.show();
}

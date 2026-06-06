#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include <stdlib.h>

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
const unsigned long DISTANCE_INTERVAL = 100;
const unsigned long SERVO_INTERVAL = 40;
const int SERIAL_LINE_MAX = 48;

unsigned long lastDistanceTime = 0;
unsigned long lastServoTime = 0;
int servoAngle = 90;
int servoStep = 5;
char serialLine[SERIAL_LINE_MAX];
int serialLineLength = 0;
bool serialLineOverflow = false;

Servo radarServo;
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  initControls();
}

void loop() {
  runServoSweep();
  readSerialCommands();

  if (millis() - lastDistanceTime >= DISTANCE_INTERVAL) {
    lastDistanceTime = millis();
    int distance = readDistanceCm();

    Serial.print("{\"distance\":");
    Serial.print(distance);
    Serial.println("}");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '\r') {
      continue;
    }

    if (input == '\n') {
      if (!serialLineOverflow && serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        parseSerialCommand(serialLine);
      }

      serialLineLength = 0;
      serialLineOverflow = false;
      continue;
    }

    if (serialLineOverflow) {
      continue;
    }

    if (serialLineLength >= SERIAL_LINE_MAX - 1) {
      serialLineLength = 0;
      serialLineOverflow = true;
      continue;
    }

    serialLine[serialLineLength++] = input;
  }
}

void parseSerialCommand(char *line) {
  if (line[0] == '\0' || line[1] != ':' || line[2] == '\0') {
    return;
  }

  switch (line[0]) {
    case 'm':
      parseMotorCommand(line + 2);
      break;
    case 'l':
      parseLedCommand(line + 2);
      break;
  }
}

void parseMotorCommand(char *args) {
  if (hasEmptyArg(args)) {
    return;
  }

  char *leftDirectionToken = strtok(args, ",");
  char *leftSpeedToken = strtok(NULL, ",");
  char *rightDirectionToken = strtok(NULL, ",");
  char *rightSpeedToken = strtok(NULL, ",");

  if (strtok(NULL, ",") != NULL) {
    return;
  }

  char leftDirection;
  char rightDirection;
  int leftSpeed;
  int rightSpeed;

  if (!parseDirection(leftDirectionToken, &leftDirection) ||
      !parseByteValue(leftSpeedToken, &leftSpeed) ||
      !parseDirection(rightDirectionToken, &rightDirection) ||
      !parseByteValue(rightSpeedToken, &rightSpeed)) {
    return;
  }

  setMotor(IN3, IN4, ENB, leftDirection, leftSpeed);
  setMotor(IN1, IN2, ENA, rightDirection, rightSpeed);
}

void parseLedCommand(char *args) {
  if (hasEmptyArg(args)) {
    return;
  }

  char *redToken = strtok(args, ",");
  char *greenToken = strtok(NULL, ",");
  char *blueToken = strtok(NULL, ",");

  if (strtok(NULL, ",") != NULL) {
    return;
  }

  int red;
  int green;
  int blue;

  if (!parseByteValue(redToken, &red) ||
      !parseByteValue(greenToken, &green) ||
      !parseByteValue(blueToken, &blue)) {
    return;
  }

  setLEDAll(red, green, blue);
}

bool hasEmptyArg(char *text) {
  if (text == NULL || text[0] == '\0') {
    return true;
  }

  for (int i = 0; text[i] != '\0'; i++) {
    if (text[i] == ',' && (i == 0 || text[i + 1] == '\0' || text[i + 1] == ',')) {
      return true;
    }
  }

  return false;
}

bool parseDirection(char *token, char *direction) {
  if (token == NULL || token[0] == '\0' || token[1] != '\0') {
    return false;
  }

  if (token[0] != 'f' && token[0] != 's' && token[0] != 'b') {
    return false;
  }

  *direction = token[0];
  return true;
}

bool parseByteValue(char *token, int *value) {
  if (token == NULL || token[0] == '\0') {
    return false;
  }

  char *end;
  long parsed = strtol(token, &end, 10);
  if (*end != '\0' || parsed < 0 || parsed > 255) {
    return false;
  }

  *value = parsed;
  return true;
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
  setLEDAll(0, 0, 0);
  setMotor(IN3, IN4, ENB, 's', 0);
  setMotor(IN1, IN2, ENA, 's', 0);
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

void setMotor(int inA, int inB, int enablePin, char direction, int speed) {
  int motorSpeed = constrain(speed, 0, MAX_SPEED);

  switch (direction) {
    case 'f':
      digitalWrite(inA, HIGH);
      digitalWrite(inB, LOW);
      analogWrite(enablePin, motorSpeed);
      break;
    case 'b':
      digitalWrite(inA, LOW);
      digitalWrite(inB, HIGH);
      analogWrite(enablePin, motorSpeed);
      break;
    case 's':
      digitalWrite(inA, LOW);
      digitalWrite(inB, LOW);
      analogWrite(enablePin, 0);
      break;
  }
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

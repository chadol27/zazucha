#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// 핀 설정
const int IN1 = 2;   const int IN2 = 3;
const int IN3 = 4;   const int IN4 = 5;
const int ENA = 10;  const int ENB = 11;
const int LED_PIN = 9;
const int trigPin = 6;
const int echoPin = 7;
const int servoPin = 8;

// 객체 생성
Servo radarServo;
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// 환경 설정 변수
const int MAX_SPEED = 255;     
const int ACCEL_STEP = 25;     // 레이더 루프와 박자를 맞추기 위해 가속 단위를 높임
const int ACCEL_DELAY = 15;    
const int SAFE_DISTANCE = 20;  // 장애물 제동 거리 (20cm)

// 레이더 제어 변수
int servoAngle = 90;
int servoStep = 5;
unsigned long lastRadarTime = 0;
const int radarInterval = 40;  // 레이더 갱신 주기 (40ms)

void setup() {
  Serial.begin(9600);
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  radarServo.attach(servoPin);
  pixels.begin();
  
  stopMotors(); // 초기 상태 정지 및 노란색 LED
  
  Serial.println("--- 풀 패키지 통합 테스트 (레이더 감지 포함) ---");
  Serial.println("w: 전진 | s: 후진 | a: 좌회전 | d: 우회전 | x: 즉시정지");
  Serial.println("※ 전방 20cm 이내 장애물 감지 시 자동 강제 제동 작동");
  Serial.println("-----------------------------------------------------");
}

void loop() {
  // 1. 레이더 상시 가동 (비동기 처리로 키보드 입력 대기 중에도 계속 회전)
  if (millis() - lastRadarTime >= radarInterval) {
    lastRadarTime = millis();
    runRadar();
  }

  // 2. 키보드 명령 제어
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'w': case 'W':
        Serial.println(">> 전진 명령달솜 (Slow Start...)");
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        slowStartWithLED(0, 150, 0); // 초록색 가속
        break;
        
      case 's': case 'S':
        Serial.println(">> 후진 명령달솜 (Slow Start...)");
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
        slowStartWithLED(150, 0, 0); // 빨간색 가속
        break;
        
      case 'a': case 'A':
        Serial.println(">> 좌회전 명령달솜 (Slow Start...)");
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        slowStartWithLED(0, 0, 150); // 파란색 가속
        break;
        
      case 'd': case 'D':
        Serial.println(">> 우회전 명령달솜 (Slow Start...)");
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
        slowStartWithLED(0, 0, 150); // 파란색 가속
        break;
        
      case 'x': case 'X':
        Serial.println(">> 사용자가 즉시 정지시킴");
        stopMotors();
        break;
    }
  }
}

// --- 레이더 구동 및 장애물 감지 함수 ---
void runRadar() {
  servoAngle += servoStep;
  if (servoAngle >= 165 || servoAngle <= 15) {
    servoStep = -servoStep;
  }
  radarServo.write(servoAngle);
  
  // 초음파 거리 측정
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 20000); // 타임아웃 제한
  int distance = (duration == 0) ? 0 : duration * 0.034 / 2;
  
  // PC 시리얼 모니터로 [각도, 거리] 데이터 전송
  Serial.print("Radar -> Angle: "); Serial.print(servoAngle);
  Serial.print(" | Dist: "); Serial.print(distance);
  Serial.println(" cm");
  
  // 비상 자동 제동 시스템 작동 조건
  if (distance > 0 && distance < SAFE_DISTANCE) {
    Serial.println("⚠️⚠️ [경고] 전방 장애물 감지! 비상 강제 제동 수행! ⚠️⚠️");
    stopMotors();
    
    // LED 경고등 연출 (빨간색 깜빡임)
    for(int blink=0; blink<2; blink++) {
      setLEDAll(150, 0, 0); delay(60);
      setLEDAll(0, 0, 0); delay(60);
    }
    stopMotors(); // 정지등(노란색) 상태 복귀
  }
}

// --- 모터 슬로우 스타트 가속 함수 ---
void slowStartWithLED(int r, int g, int b) {
  for (int speed = 0; speed <= MAX_SPEED; speed += ACCEL_STEP) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    
    int ledCount = map(speed, 0, MAX_SPEED, 0, NUMPIXELS);
    pixels.clear();
    for (int i = 0; i < ledCount; i++) {
      pixels.setPixelColor(i, pixels.Color(r, g, b));
    }
    pixels.show();
    delay(ACCEL_DELAY);
  }
  analogWrite(ENA, MAX_SPEED);
  analogWrite(ENB, MAX_SPEED);
  setLEDAll(r, g, b);
}

// --- 즉시 정지 및 LED 기본 설정 함수 ---
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  setLEDAll(100, 70, 0); // 대기 및 정지 표시 (노란색)
}

void setLEDAll(int r, int g, int b) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}
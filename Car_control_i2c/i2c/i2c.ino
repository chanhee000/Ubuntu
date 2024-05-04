#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x05

#define ENC_PIN_A 2
#define ENC_PIN_B 3
#define MOTOR_DIR_PIN 4
#define MOTOR_PWM_PIN 5

#define DEFAULT_STEERING_ANGLE 90
#define RC_SERVO_PIN 8

union DataUnion {
  short data;
  byte bytes[2];
} carSpeed, carServo;

Servo steeringServo;

void motorControl(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_DIR_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, speed);
  } else {
    digitalWrite(MOTOR_DIR_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, -speed);
  }
 
  Serial.print("Speed: ");
  Serial.println(speed);
}

void receiveData(int byteCount) {
  if (Wire.available() >= 9) {
    byte receivedData[9];
    for (int i = 0; i < 9; i++) {
      receivedData[i] = Wire.read(); 
    }

    if (receivedData[0] == '#' && receivedData[1] == 'C' && receivedData[8] == '*') {
      carServo.bytes[0] = receivedData[2];
      carServo.bytes[1] = receivedData[3];
      short angle = carServo.data;

      carSpeed.bytes[0] = receivedData[4];
      carSpeed.bytes[1] = receivedData[5];
      float speed = carSpeed.data;
      
      steeringServo.write(DEFAULT_STEERING_ANGLE + angle);
  
      Serial.print("Angle Offset: ");
      Serial.println(angle);
      Serial.print("Steering Angle: ");
      Serial.println(DEFAULT_STEERING_ANGLE + angle);
      
      motorControl(speed);
    } else {
      Serial.println("Error: Invalid data received");
    }
  }
}

void setup() {
  // 핀 모드 설정
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  
  // I2C 통신 및 시리얼 통신 시작
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Serial.begin(115200);
  
  // 서보 모터 연결
  steeringServo.attach(RC_SERVO_PIN);   
}

// 메인 루프
void loop() {
  // 여기에 추가적인 작업이 필요하다면 작성
}

#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x05

#define NEUTRAL_ANGLE 90
#define RC_SERVO_PIN   8

#define encodPinA1   2                       // Quadrature encoder A pin
#define encodPinB1   3                       // Quadrature encoder B pin

#define MOTOR_DIR    4
#define MOTOR_PWM    5

int Steering_Angle = 0;
volatile long encoderPos = 0;
unsigned long lastTime, now, h, k;
unsigned char encoderPos_long[6] = {h, 0, 0, 0, 0, k};
Servo Steeringservo;

union
{
  int data;
  byte bytes[2];
} car_speed, car_servo;

union
{
  int encoder_data;
  byte encoder_bytes[2];
} encoder_union;

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoderB, FALLING);    // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;         // To prevent Motor Noise
}

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == HIGH)
  {
    encoderPos++;
  }
  else
  {
    encoderPos--;
  }
}

void reset_encoder(void)
{
  encoderPos = 0;
}

void motor_control(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, speed);
  }
  else
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, -speed);
  }
}

void receiveData(int byteCount)
{
  if (Wire.available() >= 9)
  {
    byte receivedData[9];
    for (int i = 0; i < 9; i++)
    {
      receivedData[i] = Wire.read();
    }

    if (receivedData[0] == '#' && receivedData[1] == 'C' && receivedData[8] == '*')
    {
      car_servo.bytes[0] = receivedData[2];
      car_servo.bytes[1] = receivedData[3];
      short angle = car_servo.data;

      car_speed.bytes[0] = receivedData[4];
      car_speed.bytes[1] = receivedData[5];
      float speed = car_speed.data;

      Steeringservo.write(NEUTRAL_ANGLE + angle);

      motor_control(speed);
    }
    else
    {
      Serial.println("Invalid protocol");
    }
  }
}

void setup()
{
#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(requestData);

  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);
  interrupt_setup();
}

void encoder_serial_print(void)
{
  Serial.print("encoderPos: ");
  Serial.println(encoderPos);
}

void loop()
{
  encoder_serial_print();
  delay(100);
}

void requestData()
{
  encoderPos_long[0] = 'h'; // Setting the first byte to 'h' to indicate valid data
  encoder_union.encoder_data = encoderPos;
  encoderPos_long[1] = encoder_union.encoder_bytes[0];
  encoderPos_long[2] = encoder_union.encoder_bytes[1];
  encoderPos_long[3] = 0;
  encoderPos_long[4] = 0;
  encoderPos_long[5] = 'k'; // Setting the last byte to 'k' to indicate valid data
  Wire.write(encoderPos_long, sizeof(encoderPos_long));
}

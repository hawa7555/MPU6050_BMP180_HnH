#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <avr/wdt.h>

#define g 9.80665f
#define accelRange 16384.0
#define gyroRange 65.5

MPU6050 mpu;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
uint8_t devStatus;
Quaternion q;
VectorFloat gravity;

float ypr[3];
float y, p , r;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

float accelX_off, accelY_off, accelZ_off;
float gyroX_off, gyroY_off, gyroZ_off;

char selection = '0';

void setup() {
  pinMode(7, OUTPUT);
  
  digitalWrite(7, HIGH);
  delay(1000);
  digitalWrite(7, LOW);
    
  Wire.begin();
  //Wire.setClock(400000);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);        //500dps, full range
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus) {
    while (1);
  }
  mpu.CalibrateAccel(100);
  mpu.CalibrateGyro(100);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();     //packetsize is 42

  gyroX_off = mpu.getXGyroOffset();
  gyroY_off = mpu.getYGyroOffset();
  gyroZ_off = mpu.getZGyroOffset();
  accelX_off = mpu.getXAccelOffset();
  accelY_off = mpu.getYAccelOffset();
  accelZ_off = mpu.getZAccelOffset();

  wdt_disable();
  delay(3000);        //to avoid infinite resetting
  selection = 0;      //initial value
}

void loop() {
  switch(selection) {
    case '0': getOrientation(); break;
    case '1': getAccelReadings(); break;
    case '2': getGyroReadings(); break;
    case '3': sendString("97000"); break;
  }
}

void getOrientation() {
  mpu.resetFIFO();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024 ) {
    return;
  }
  else if (fifoCount % packetSize != 0) {
    return;
  }
  wdt_enable(WDTO_4S);                 //watchDog timer for 4sec
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();    //wait for correct data length, very short duration

  while (fifoCount >= packetSize) {                    //to read latest packet
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  y = ypr[0] * 180 / M_PI;
  p = ypr[1] * 180 / M_PI;
  r = ypr[2] * 180 / M_PI;
  delay(100);
  wdt_reset();
}

void getAccelReadings() {
  wdt_enable(WDTO_4S);               //watchDog timer for 4sec
  mpu.getAcceleration(&ax, &ay, &az);
  accelX = (ax - accelX_off) / accelRange * g;
  accelY = (ay - accelY_off) / accelRange * g;
  accelZ = (az - accelZ_off) / accelRange * g;
  delay(100);
  wdt_reset();
}

void getGyroReadings() {
  wdt_enable(WDTO_4S);               //watchDog timer for 4sec
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = (gx - gyroX_off) / gyroRange;
  gyroY = (gy - gyroY_off) / gyroRange;
  gyroZ = (gz - gyroZ_off) / gyroRange;
  delay(100);
  wdt_reset();
}

void uartInit() {
  //baud rate, 9600
  UBRR0H = 0x00;
  UBRR0L = 0x67;

  UCSR0B |= (1 << 4) | (1 << 3); //receiver and transmitter enable
  UCSR0C |= (1 << 2) | (1 << 1); //8-bit data size
  UCSR0B |= (1 << RXCIE0);      //receiver interrupt enable
  sei();               //global interrupt enable
}

void sendChar(char data) {
  UDR0 = data;
  while (!(UCSR0A & (1 << UDRE0)) ); //wait to send data
}

char receiveData() {
  while (!(UCSR0A & (1 << 7)) ); //wait to receive data
  char data = UDR0;
  return data;
}

void sendString(char* data) {
  while (*data != '\0') sendChar(*data++);
}

ISR(USART_RX_vect) {
  selection = UDR0;
}

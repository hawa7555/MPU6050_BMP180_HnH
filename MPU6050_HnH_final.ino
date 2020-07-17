#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "SFE_BMP180.h"
#include "avr/wdt.h"
#include "SoftwareSerial.h"

#define g 9.80665f
#define accelRange 16384.0
#define gyroRange 65.5
#define seaLevelPressure_hPa 1013.25

SoftwareSerial bluetooth(7, 8);  //rx-tx

MPU6050 mpu;
SFE_BMP180 bmp180;

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

double Temp, Pressure, alti;

char selection;
String str;
char data[10];

void setup() {
  Wire.begin();
  //Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);        //500dps, full range
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus) {
    Serial.println("DMP Init Failed");
    while (1);
  }
  mpu.CalibrateAccel(100);
  mpu.CalibrateGyro(100);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();     //packetsize is 42
  Serial.println(packetSize);

  gyroX_off = mpu.getXGyroOffset();
  gyroY_off = mpu.getYGyroOffset();
  gyroZ_off = mpu.getZGyroOffset();
  accelX_off = mpu.getXAccelOffset();
  accelY_off = mpu.getYAccelOffset();
  accelZ_off = mpu.getZAccelOffset();

  bluetooth.begin(9600);
  wdt_disable();
  delay(3000);        //to avoid infinite resetting
  selection = '0';   //initial value
}

void loop() {
  if ( bluetooth.available() > 0) {
    selection = bluetooth.read();
  }
  switch (selection) {
    case '0': getOrientation(); break;
    case '1': getAccelReadings(); break;
    case '2': getGyroReadings(); break;
    case '3': getBmpReadings(); break;
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
  wdt_enable(WDTO_8S);                 //watchDog timer for 8sec
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
  //Serial.println(p);
  str = "*" + String(p);
  str.toCharArray(data, 10);
  bluetooth.write(data);
  delay(100);
  wdt_reset();
}

void getAccelReadings() {
  wdt_enable(WDTO_8S);               //watchDog timer for 8sec
  mpu.getAcceleration(&ax, &ay, &az);
  accelX = (ax - accelX_off) / accelRange * g;
  accelY = (ay - accelY_off) / accelRange * g;
  accelZ = (az - accelZ_off) / accelRange * g;
  //Serial.println(accelY);
  str = "*" + String(accelZ);
  str.toCharArray(data, 10);
  bluetooth.write(data);
  delay(100);
  wdt_reset();
}

void getGyroReadings() {
  wdt_enable(WDTO_8S);               //watchDog timer for 8sec
  mpu.getRotation(&gx, &gy, &gz);
  gyroX = (gx - gyroX_off) / gyroRange;
  gyroY = (gy - gyroY_off) / gyroRange;
  gyroZ = (gz - gyroZ_off) / gyroRange;
  //Serial.println(gyroZ);
  str = "*" + String(gyroZ);
  str.toCharArray(data, 10);
  bluetooth.write(data);
  delay(100);
  wdt_reset();
}

void getBmpReadings() {
  if ( !bmp180.begin() ) {
    Serial.println("BMP180 Problem");
    return;
  }
  int flag = bmp180.startTemperature();
  if (flag) {
    delay(300);
    wdt_enable(WDTO_8S);               //watchDog timer for 8sec
    flag = bmp180.getTemperature(Temp);

    if (flag) {
      int delayN = bmp180.startPressure(2);

      if (delayN > 0) {
        delay(delayN);
        flag = bmp180.getPressure(Pressure, Temp);
        alti = bmp180.altitude(Pressure, seaLevelPressure_hPa);

        if (flag) {
          str = "*" + String(Pressure * 100);
          str.toCharArray(data, 10);
          bluetooth.write(data);
        }
      }
    }
  }
  wdt_reset();
}

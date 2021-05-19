#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#define INT_PIN 8

MPU6050 mpu6050;
bool dmpReady = false;
uint16_t  packetSize = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

unsigned long getCurrentTimeMS() {
  return ESP.getCycleCount() / 80000L;
}

void setup() {

  Wire.begin();  
  Wire.setClock(400000);
  Serial.begin(115200);
  pinMode(INT_PIN, INPUT);

  if (mpu6050.testConnection()) {
    Serial.println("MPU6050 OK");
    attachInterrupt(INT_PIN, dmpDataReady, RISING);
  } else {
    Serial.println("MPU6050 NG");
    return;
  }
  
  mpu6050.initialize();
  mpu6050.setXGyroOffset(220);
  mpu6050.setYGyroOffset(76);
  mpu6050.setZGyroOffset(-85);
  mpu6050.setZAccelOffset(1688);

  if (mpu6050.dmpInitialize() == 0) {
    Serial.println("MPU6050DMP OK");
    mpu6050.setDMPEnabled(true); 
    packetSize = mpu6050.dmpGetFIFOPacketSize();
    dmpReady = true;    
  } else {
    Serial.println("MPU6050DMP NG");
  }
}

uint16_t fifoCount  = 0;
uint8_t  fifoBuffer[64];
unsigned long preTime = 0;

void loop() {
  
  if (!dmpReady) return;
  
  while (!mpuInterrupt);
  mpuInterrupt = false;

  while (fifoCount < packetSize) {
    fifoCount = mpu6050.getFIFOCount();
  }
  
  uint8_t mpuIntStatus = mpu6050.getIntStatus();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu6050.resetFIFO();
    return;
  } else if ((mpuIntStatus & 0x02) == false) {
    return;
  }
  
  mpu6050.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;

  unsigned long curTime = getCurrentTimeMS();
  unsigned long duration = curTime - preTime;

  Quaternion q;
  VectorInt16 aa;
  VectorInt16 aaReal;
  VectorFloat gravity;
  
  mpu6050.dmpGetQuaternion(&q, fifoBuffer);
  mpu6050.dmpGetAccel(&aa, fifoBuffer);
  mpu6050.dmpGetGravity(&gravity, &q);
  mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.print(aaReal.z);
  Serial.print("\t");
  Serial.print(duration);
  Serial.println("");
  preTime = curTime;
}

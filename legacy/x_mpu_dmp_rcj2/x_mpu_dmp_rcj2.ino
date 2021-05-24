#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <BleMouse.h>

#define Gyro_X -1529
#define Gyro_Y -981
#define Gyro_Z -131
#define Accel_Z -1458
BleMouse bleMouse;

MPU6050 mpu;
static uint8_t mpuIntStatus;
static bool dmpReady = false;  // set true if DMP init was successful
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

int16_t  Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0, y, z;
uint16_t fifoCount;
uint8_t fifoBuffer[64]; // FIFO storage buffer                 // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float x_kand=1;
float y_kand=1;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  bleMouse.begin();

  pinMode(16,INPUT);
  pinMode(17,INPUT);
  
  Gryo_Start();
}

void loop() {
  
    GyroGet();

    Serial.print(Gyro);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.println(z);

    if(bleMouse.isConnected()) {     //接続状況確認
      if(touchRead(T0)<30){
         bleMouse.move(-(Gyro)*x_kand, (y)*y_kand, 0);
      }
      if(digitalRead(16)==0){
         bleMouse.click() ;
         delay(100);
      }
      if(digitalRead(17)==0){
         bleMouse.click(MOUSE_RIGHT) ;
         delay(100);
      }   
     }
}

void GyroGet() {
  mpuIntStatus = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Gyro_Now = degrees(ypr[0]) + 180;
    Gyro = Gyro_Now + Gyro_Offset;
    if (Gyro < 0) Gyro += 360;
    if (Gyro > 359) Gyro -= 360;
  }
  y=ypr[1] * 180 / M_PI;
  z=ypr[2] * 180 / M_PI;
}

void Gryo_Start() {
  mpu.initialize();
  if (mpu.testConnection() != true) {
    Serial.println("MPU disconection");
    while (true) {}
  }
  if (mpu.dmpInitialize() != 0) {
    Serial.println("MPU break");
    while (true) {}
  }
  mpu.setXGyroOffset(Gyro_X);
  mpu.setYGyroOffset(Gyro_Y);
  mpu.setZGyroOffset(Gyro_Z);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(Accel_Z);
  
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
}

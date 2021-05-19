#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Wire.begin();
  Serial.begin(38400);

  // 初期化
  accelgyro.initialize();
  delay(10);

  // 計測範囲を2000 deg/secに設定、16.4 LSB/deg/s
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  delay(10);
  
  accelgyro.CalibrateGyro();
  accelgyro.CalibrateAccel();
}

void loop() {

  // 時間を計っとく
  static float timer = 0;

  // 各軸加速度と角速度を取得
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //X、Y軸まわりの角度を計算
  float degY = atan2(ax, az) * RAD_TO_DEG;
  float degX = atan2(ay, az) * RAD_TO_DEG;

  // Z軸まわりの角度を計算
  static float degZ = 0;
  static int last_time = micros();
  float omega = float(gz)/16.4; //omega[deg/s] = gz[LSB] / 16.4[LSB/deg/s]
  int current_time = micros();  //time[us]
  float dt = current_time - last_time;
  // 角度は角速度の積分
  degZ += omega *dt/1000000.0;  //degZ[deg] = omega[deg/s] * dt[us] / 1000000[us/s]
  last_time = current_time;
  
  Serial.print(timer+=dt/1000000); Serial.print("\t");
  Serial.print(degX); Serial.print("\t");
  Serial.print(degY);Serial.print("\t");
  Serial.println(degZ);
}

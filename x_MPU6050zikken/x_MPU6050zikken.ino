// MPU-6050 Accelerometer + Gyro
  #include <Wire.h>
 
  #define MPU6050_ACCEL_XOUT_H       0x3B   // R
  #define MPU6050_WHO_AM_I           0x75   // R
  #define MPU6050_PWR_MGMT_1         0x6B   // R/W
  #define MPU6050_I2C_ADDRESS 0x68
 
typedef union accel_t_gyro_union{
  struct{
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  }
  reg;
  struct{
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  }
  value;
};
 
void setup(){
  Wire.begin();
  int error;
  uint8_t c;
  Serial.begin(9600);
  Serial.print("InvenSense MPU-6050");
  Serial.print("June 2012");
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print("WHO_AM_I : ");
  Serial.print(c,HEX);
  Serial.print(", error = ");
  Serial.println(error,DEC);
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  Serial.print("PWR_MGMT_1 : ");
  Serial.print(c,HEX);
  Serial.print(", error = ");
  Serial.println(error,DEC);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}
 
  void loop(){
  int error;
  float dT;
  accel_t_gyro_union accel_t_gyro;
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  Serial.print(error,DEC);
  Serial.print("\t");
 
    uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
 
  dT = ( (float) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  Serial.print(dT, 1);
  Serial.print("\t");
 
  float acc_x = accel_t_gyro.value.x_accel / 16384.0; //FS_SEL_0 16,384 LSB / g
  float acc_y = accel_t_gyro.value.y_accel / 16384.0;
  float acc_z = accel_t_gyro.value.z_accel / 16384.0;
 
  Serial.print(acc_x, 2);
  Serial.print("\t");
  Serial.print(acc_y, 2);
  Serial.print("\t");
  Serial.print(acc_z, 2);
  Serial.print("\t");
 
  float acc_angle_x = atan2(acc_x, acc_z) * 360 / 2.0 / PI;
  float acc_angle_y = atan2(acc_y, acc_z) * 360 / 2.0 / PI;
  float acc_angle_z = atan2(acc_x, acc_y) * 360 / 2.0 / PI;
 
  Serial.print(acc_angle_x, 2);
  Serial.print("\t");
  Serial.print(acc_angle_y, 2);
  Serial.print("\t");
  Serial.print(acc_angle_z, 2);
  Serial.print("\t");
 
  float gyro_x = accel_t_gyro.value.x_gyro / 131.0;  //FS_SEL_0 131 LSB / (°/s)
  float gyro_y = accel_t_gyro.value.y_gyro / 131.0;
  float gyro_z = accel_t_gyro.value.z_gyro / 131.0;
 
  Serial.print(gyro_x, 2);
  Serial.print("\t");
  Serial.print(gyro_y, 2);
  Serial.print("\t");
  Serial.print(gyro_z, 2);
  Serial.println("");
}
 
// MPU6050_read
int MPU6050_read(int start, uint8_t *buffer, int size){
  int i, n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  n = Wire.endTransmission(false);   // hold the I2C-bus
  if (n != 0)
    return (n);
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size){
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0);  // return : no error
}
 
// MPU6050_write
int MPU6050_write(int start, const uint8_t *pData, int size){
  int n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
  return (0);         // return : no error
}
 
// MPU6050_write_reg
int MPU6050_write_reg(int reg, uint8_t data){
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}

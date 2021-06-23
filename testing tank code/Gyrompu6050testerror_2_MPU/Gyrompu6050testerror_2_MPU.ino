/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

//---------------------------------------------------------------
const int MPU2 = 0x69; // MPU6050 I2C address
float AccX2, AccY2, AccZ2;
float GyroX2, GyroY2, GyroZ2;
float accAngleX2, accAngleY2, gyroAngleX2, gyroAngleY2, gyroAngleZ2;
float roll2, pitch2, yaw2;
float AccErrorX2, AccErrorY2, GyroErrorX2, GyroErrorY2, GyroErrorZ2;
float elapsedTime2, currentTime2, previousTime2;
int c2 = 0;
//---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  //--------------------------------------------------------------
  
  //Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU2);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  //-------------------------------------------------------------
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU2);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);

  
  
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU2);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  //---------------------------------------------------------------
  // Call this function if you need to get the IMU error values for your module
  
  /*calculate_IMU_error();
  delay(20);
  calculate_IMU_error2();
  delay(20);
*/
}
void loop() {
    //Serial.println("LOOP mulai");
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Serial.println("errorx kelar");
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  while (c2 < 200) {
    Wire.beginTransmission(MPU2);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true);
    AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX2 = AccErrorX2 + ((atan((AccY2) / sqrt(pow((AccX2), 2) + pow((AccZ2), 2))) * 180 / PI));
    AccErrorY2 = AccErrorY2 + ((atan(-1 * (AccX2) / sqrt(pow((AccY2), 2) + pow((AccZ2), 2))) * 180 / PI));
    c2++;
  }
  //Serial.println("errorx2 kelar");
  //Divide the sum by 200 to get the error value
  AccErrorX2 = AccErrorX2 / 200;
  AccErrorY2 = AccErrorY2 / 200;
  c2 = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Serial.println("gyroerror kelar");
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
//Wire.endTransmission();
//delay(20);
  //---------------------------------------------------------------
  c2=0;
  // Read gyro values 200 times
  while (c2 < 200) {
    Wire.beginTransmission(MPU2);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true);
    GyroX2 = Wire.read() << 8 | Wire.read();
    GyroY2 = Wire.read() << 8 | Wire.read();
    GyroZ2 = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX2 = GyroErrorX2 + (GyroX2 / 131.0);
    GyroErrorY2 = GyroErrorY2 + (GyroY2 / 131.0);
    GyroErrorZ2 = GyroErrorZ2 + (GyroZ2 / 131.0);
    c2++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX2 = GyroErrorX2 / 200;
  GyroErrorY2 = GyroErrorY2 / 200;
  GyroErrorZ2 = GyroErrorZ2 / 200;
  //Serial.println("gyroerror2 kelar");
  //Wire.endTransmission();
  //---------------------------------------------------------------
  // Print the error values on the Serial Monitor
  /*
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);*/
//----------------------------------------------------------------
  Serial.print("AccErrorX: ");
  Serial.print(AccErrorX);
  Serial.print("    AccErrorX2: ");
  Serial.println(AccErrorX2);
  Serial.print("AccErrorY: ");
  Serial.print(AccErrorY);
  Serial.print("    AccErrorY2: ");
  Serial.println(AccErrorY2);
  Serial.print("GyroErrorX: ");
  Serial.print(GyroErrorX);
  Serial.print("    GyroErrorX2: ");
  Serial.println(GyroErrorX2);
  Serial.print("GyroErrorY: ");
  Serial.print(GyroErrorY);
  Serial.print("    GyroErrorY2: ");
  Serial.println(GyroErrorY2);
  Serial.print("GyroErrorZ: ");
  Serial.print(GyroErrorZ);
  Serial.print("    GyroErrorZ2: ");
  Serial.println(GyroErrorZ2);
//----------------------------------------------------------------
Serial.println("LOOP abis");
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

//-----------------------------------------------------------
void calculate_IMU_error2() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c2 < 200) {
    Wire.beginTransmission(MPU2);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true);
    AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX2 = AccErrorX2 + ((atan((AccY2) / sqrt(pow((AccX2), 2) + pow((AccZ2), 2))) * 180 / PI));
    AccErrorY2 = AccErrorY2 + ((atan(-1 * (AccX2) / sqrt(pow((AccY2), 2) + pow((AccZ2), 2))) * 180 / PI));
    c2++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX2 = AccErrorX2 / 200;
  AccErrorY2 = AccErrorY2 / 200;
  c2 = 0;
  // Read gyro values 200 times
  while (c2 < 200) {
    Wire.beginTransmission(MPU2);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true);
    GyroX2 = Wire.read() << 8 | Wire.read();
    GyroY2 = Wire.read() << 8 | Wire.read();
    GyroZ2 = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX2 = GyroErrorX2 + (GyroX2 / 131.0);
    GyroErrorY2 = GyroErrorY2 + (GyroY2 / 131.0);
    GyroErrorZ2 = GyroErrorZ2 + (GyroZ2 / 131.0);
    c2++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX2 = GyroErrorX2 / 200;
  GyroErrorY2 = GyroErrorY2 / 200;
  GyroErrorZ2 = GyroErrorZ2 / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX2: ");
  Serial.println(AccErrorX2);
  Serial.print("AccErrorY2: ");
  Serial.println(AccErrorY2);
  Serial.print("GyroErrorX2: ");
  Serial.println(GyroErrorX2);
  Serial.print("GyroErrorY2: ");
  Serial.println(GyroErrorY2);
  Serial.print("GyroErrorZ2: ");
  Serial.println(GyroErrorZ2);
}
//---------------------------------------------------------------

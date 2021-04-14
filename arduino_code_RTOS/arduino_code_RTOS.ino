#include <Arduino_FreeRTOS.h>
void SerialCom(void *pvParameters);
void Gerak(void *pvParameters);
void GerakX(void *pvParameters);
void GerakY(void *pvParameters);
void GerakZ(void *pvParameters);

//=====================================================================
#include <Servo.h>
Servo servo_x;
Servo servo_y;
Servo servo_z;
int j = 0;
float correct_x;
float correct_y;
float correct_z;

//=======================================================================

#include <Wire.h>
const int MPU = 0x68; // MPU1 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

//---------------------------------------------------------------
const int MPU2 = 0x69; // MPU2 I2C address (AD0 == HIGH)
float AccX2, AccY2, AccZ2;
float GyroX2, GyroY2, GyroZ2;
float accAngleX2, accAngleY2, gyroAngleX2, gyroAngleY2, gyroAngleZ2;
float roll2, pitch2, yaw2;
float AccErrorX2, AccErrorY2, GyroErrorX2, GyroErrorY2, GyroErrorZ2;
float elapsedTime2, currentTime2, previousTime2;
int c2 = 0;
//---------------------------------------------------------------

float px, ix, dx, pidx;
float errorx, prev_errorx = 0;

float py, iy, dy, pidy;
float errory, prev_errory = 0;

float pz, iz, dz, pidz;
float errorz, prev_errorz = 0;

float spX = 90;
int pX = 180;
int qX = 0;

float spY = 90;
int pY = 180;
int qY = 0;

float spZ = 90;
int pZ = 180;
int qZ = 0;

int RaspiX = 90;
int RaspiZ = 90;


//---------------------------------------------------------------

/***************************************
*                                      *
    Observer Feedback Control System
*                                      *
****************************************/
float Ts = 1;
float current_time, previous_time, elapsed_time;

long count = 0;

float x1_hat_x = 0;
float x2_hat_x = 0;
float x3_hat_x = 0;

float x1_hat_y = 0;
float x2_hat_y = 0;
float x3_hat_y = 0;

float x1_hat_z = 0;
float x2_hat_z = 0;
float x3_hat_z = 0;

float u1 = 0;
float u2 = 0;
float u3 = 0;
float u1_x = 0;
float u2_y = 0;
float u3_z = 0;
float y1 = 0;
float y2 = 0;
float y3 = 0;

int dutyCycle  = 0;

float a11_x = 0;
float a12_x = 0;
float a13_x = 0;
float a21_x = 5420000;
float a22_x = -14700;
float a23_x = 0;
float a31_x = 300;
float a32_x = 0;
float a33_x = -900;

float a11_y = 100;
float a12_y = 1600;
float a13_y = 5071400;
float a21_y = 0;
float a22_y = 0;
float a23_y = 0;
float a31_y = 0;
float a32_y = 0;
float a33_y = -600;

float a11_z = 140;
float a12_z = 2300;
float a13_z = 140507;
float a21_z = 0;
float a22_z = -670;
float a23_z = 0;
float a31_z = 0;
float a32_z = 0;
float a33_z = 0;

float b11_x = 1;
float b12_x = 1.0003253;
float b21_x = 0;
float b22_x = 3.1257;
float b31_x = -115.2096;
float b32_x = 0.0198;

float b11_y = 4.8964;
float b12_y = 0;
float b21_y = -125.678;
float b22_y = 1.0000028573;
float b31_y = 0.0134;
float b32_y = 0;

float b11_z = 1;
float b12_z = 1;
float b21_z = -110.3138;
float b22_z = 0;
float b31_z = 1;
float b32_z = 1.00000017864;

float k1_x = 1.0000000032461;
float k2_x = 0;
float k3_x = 0;
float g_x = 0.000001549;

float k1_y = 0;
float k2_y = 1.00000871893691;
float k3_y = 0;
float g_y = 0.00001304;

float k1_z = 0;
float k2_z = 0;
float k3_z = 1.0000000035277;
float g_z = 0.000001274;



//---------------------------------------------------------------


void setup() {

  Serial.begin(9600);
  //========================================================
  servo_x.attach(9);                  // Set pin 9 untuk servo x
  servo_y.attach(8);                  // Set pin 8 untuk servo y
  servo_z.attach(10);                 // Set pin 10 untuk servo z

  servo_x.write(90);                  // Set posisi servo x ke 90 derajat
  servo_y.write(90);                  // Set posisi servo y ke 90 derajat
  servo_z.write(90);                  // Set posisi servo z ke 90 derajat
  delay(1000);                        // Memberikan delay agar servo sampai ke posisi set point

  Wire.begin();                      // Inisialisasi komunikasi
  Wire.beginTransmission(MPU);       // Memulai komunikasi dengan MPU1
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - memasukan nilai 0 ke register 6B
  Wire.endTransmission(true);        // Mengakhiri transmisi

  // Mengkonfigurasi sensitivitas Accelerometer - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  // Komunikasi dengan register ACCEL_CONFIG (1C hex)
  Wire.write(0x10);                  // Men-Set nilai register bits menjadi 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Mengkonfigurasi sensitivitas Gyro - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Komunikasi dengan register GYRO_CONFIG (1B hex)
  Wire.write(0x10);                   // Men-Set nilai register bits menjadi 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);


  //--------------------------------------------------------------

  Wire.beginTransmission(MPU2);      // Memulai komunikasi dengan MPU2
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - memasukan nilai 0 ke register 6B
  Wire.endTransmission(true);        // Mengakhiri transmisi

  // Mengkonfigurasi sensitivitas Accelerometer - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU2);
  Wire.write(0x1C);                  // Komunikasi dengan register ACCEL_CONFIG (1C hex)
  Wire.write(0x10);                  // Men-Set nilai register bits menjadi 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Mengkonfigurasi sensitivitas Gyro - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU2);
  Wire.write(0x1B);                   // Komunikasi dengan register GYRO_CONFIG (1B hex)
  Wire.write(0x10);                   // Men-Set nilai register bits menjadi 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  //---------------------------------------------------------------


  // Panggil fungsi ini untuk melihat nilai IMU error
  calculate_IMU_error();
  delay(20);
  calculate_IMU_error2();
  delay(200);
  xTaskCreate(Gerak, "gerak", 512, NULL, 1, NULL);
  xTaskCreate(SerialCom, "SerialCom", 128, NULL, 1, NULL);
  xTaskCreate(GerakX, "gerakx", 512, NULL, 1, NULL);
  xTaskCreate(GerakY, "geraky", 512, NULL, 1, NULL);
  xTaskCreate(GerakZ, "gerakz", 512, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {

}

void SerialCom(void *pvParameters)
{
  Serial.println("Komunikasi Serial OK");
  for(;;){
    if (Serial.available()>0){
      RaspiX = Serial.parseInt();
      RaspiZ = Serial.parseInt();
      modify_Xsetpoint(RaspiX, RaspiX+90, RaspiX-90);
      modify_Zsetpoint(RaspiZ, RaspiZ+90, RaspiZ-90);
    }
  }
}

void modify_Xsetpoint(float RaspiX, float RaspiX1, float RaspiX2)
{
  spX = RaspiX;
  pX = RaspiX1;
  qX = RaspiX2;
}

void modify_Zsetpoint(float RaspiZ, float RaspiZ1, float RaspiZ2)
{
  spZ = RaspiZ;
  pZ = RaspiZ1;
  qZ = RaspiZ2;
}

void modify_xhat(float x1, float x2, float x3)
{
  x1_hat_x = x1;
  x2_hat_y = x2;
  x3_hat_z = x3;
}

void GerakX(void *pvParameters)
{
  Serial.println("Modul GerakX OK");
  for (;;)
  {
    u1 = (k1_x * x1_hat_x) + (k2_x * x2_hat_x) + (k3_x * x3_hat_x) + g_x * spX;
    u1_x = spX - u1;
    if (u1_x < 70) {
      u1_x = 70;
    }
    else if (u1_x > 130) {
      u1_x = 130;
    }
    servo_x.write(u1_x);
    //servo_y.write(u2_y);
    //servo_z.write(u3_z);
    Serial.print("           U1 : ");
    Serial.print(u1_x);
    //Serial.print("           U2 : ");
    //Serial.print(u2_y);
    //Serial.print("           U3: ");
    //Serial.print(u3_z);
  }

}

void GerakY(void *pvParameters)
{
  Serial.println("Modul GerakY OK");
  for (;;)
  {
    u2 = (k1_y * x1_hat_y) + (k2_y * x2_hat_y) + (k3_y * x3_hat_y) + g_y * spY;
    u2_y = spY - u2;
    if (u2_y < 60) {
      u2_y = 60;
    }
    else if (u2_y > 130) {
      u2_y = 130;
    }
    //servo_x.write(u1_x);
    servo_y.write(u2_y);
    //servo_z.write(u3_z);
    //Serial.print("           U1 : ");
    //Serial.print(u1_x);
    Serial.print("           U2 : ");
    Serial.print(u2_y);
    //Serial.print("           U3: ");
    //Serial.print(u3_z);
  }

}
void GerakZ(void *pvParameters)
{
  Serial.println("Modul GerakZ OK");
  for (;;)
  {
    u3 = (k1_z * x1_hat_z) + (k2_z * x2_hat_z) + (k3_z * x3_hat_z) + g_z * spZ;
    u3_z = spZ - u3;
    //  if (u3_z < 70) {
    //    u3_z = 70;
    //  }
    //  else if (u3_z > 130) {
    //    u3_z = 130;
    //  }
    //servo_x.write(u1_x);
    //servo_y.write(u2_y);
    servo_z.write(u3_z);
    // Serial.print("           U1 : ");
    //Serial.print(u1_x);
    //Serial.print("           U2 : ");
    //Serial.print(u2_y);
    Serial.print("           U3: ");
    Serial.print(u3_z);
  }

}


void Gerak(void *pvParameters)
{
  Serial.println("Modul Gerak OK");
  for (;;)
  {
    //====================================================================================================================
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, tiap nilai sumbu disimpan tiap 2 registers

    // Untuk nilai akselero +-2g, nilainya harus dibagi dengan 16384, sesuai dengan datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    // Menghitung data Roll dan Pitch dari accelerometer
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) ; // Masih dalam bentuk radian, dikali 180/pi untuk menjadi derajat
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI); // Masih dalam bentuk radian, dikali 180/pi untuk menjadi derajat

    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time merupakan waktu yg disimpan sebelum waktu aktual dibaca
    currentTime = millis();            // Current time merupakan waktu aktua;
    elapsedTime = (currentTime - previousTime) / 1000; // Dibagi 1000 agar nilainya menjadi seconds

    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, tiap nilai sumbu disimpan tiap 2 registers
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // Untuk nilai gyro 250deg/s, nilainya harus dibagi 131.0, sesuai dengan datasheet
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Nilai Output dikoreksi dengan dijumlahkan dengan nilai yg didapat dari void calculate_IMU_error()
    GyroX = GyroX - 1.37;          //0.74;
    GyroY = GyroY + 0.46;          //0.66;
    GyroZ = GyroZ + 0.18;          //0.03;

    // Karena hasilnya masih berupa degrees per seconds (deg/s), Jadi harus dikalikan dengan satuan waktu (seconds) untuk mendapatkan nilai sudut dalam degrees
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw =  yaw + GyroZ * elapsedTime;
    // Mengkombinasikan nilai acceleromter dan gyro untuk mendapatkan posisi sudut yg akurat
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;


    //-------------------------------------------------------------------
    // === Read acceleromter data for MPU2 === //
    Wire.beginTransmission(MPU2);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true); // Read 6 registers total, tiap nilai sumbu disimpan tiap 2 registers

    // Untuk nilai akselero +-2g, nilainya harus dibagi dengan 16384, sesuai dengan datasheet
    AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    // Menghitung data Roll dan Pitch dari accelerometer
    accAngleX2 = (atan(AccY2 / sqrt(pow(AccX2, 2) + pow(AccZ2, 2))) * 180 / PI) ; // Masih dalam bentuk radian, dikali 180/pi untuk menjadi derajat
    accAngleY2 = (atan(-1 * AccX2 / sqrt(pow(AccY2, 2) + pow(AccZ2, 2))) * 180 / PI); // Masih dalam bentuk radian, dikali 180/pi untuk menjadi derajat

    // === Read gyroscope data === //
    previousTime2 = currentTime2;        // Previous time merupakan waktu yg disimpan sebelum waktu aktual dibaca
    currentTime2 = millis();            // Current time merupakan waktu aktua;
    elapsedTime2 = (currentTime2 - previousTime2) / 1000; // Dibagi 1000 agar nilainya menjadi seconds

    Wire.beginTransmission(MPU2);
    Wire.write(0x43); // Gyro data first register address 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true); // Read 6 registers total, tiap nilai sumbu disimpan tiap 2 registers
    GyroX2 = (Wire.read() << 8 | Wire.read()) / 131.0; // Untuk nilai gyro 250deg/s, nilainya harus dibagi 131.0, sesuai dengan datasheet
    GyroY2 = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ2 = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Nilai Output dikoreksi dengan dijumlahkan dengan nilai yg didapat dari void calculate_IMU_error()
    GyroX2 = GyroX2 + 0.77;//5.44;          //
    GyroY2 = GyroY2 + 0.66;//0.11;          //
    GyroZ2 = GyroZ2 - 0.03;//1.69;          //

    // Karena hasilnya masih berupa degrees per seconds (deg/s), Jadi harus dikalikan dengan satuan waktu (seconds) untuk mendapatkan nilai sudut dalam degrees
    gyroAngleX2 = gyroAngleX2 + GyroX2 * elapsedTime2; // deg/s * s = deg
    gyroAngleY2 = gyroAngleY2 + GyroY2 * elapsedTime2;
    yaw2 =  yaw2 + GyroZ2 * elapsedTime2;
    // Mengkombinasikan nilai acceleromter dan gyro untuk mendapatkan posisi sudut yg akurat
    roll2 = 0.96 * gyroAngleX2 + 0.04 * accAngleX2;
    pitch2 = 0.96 * gyroAngleY2 + 0.04 * accAngleY2;

    //--------------------------------------------------------------------

    int servo0Value = roll * 4; // dikali 4 untuk mendapatkan posisi derajat sesungguhnya
    int servo1Value = pitch * 4; //
    int servo2Value = yaw * 4; //


    int servo0Value1 = map(servo0Value, -90, 90, pX, qX); // dimaping untuk mulai pada sudut 90 derajat
    int servo1Value1 = map(servo1Value, -90, 90, pY, qY);
    int servo2Value1 = map(servo2Value, -90, 90, pZ, qZ);
    //  servo0Value1 =servo0Value1 +30;
    //if(servo0Value1<70){
    //  servo0Value1 =70;
    // }


    //----------------------------------------------------------

    int gyro0Value1 = roll2 * 4;
    int gyro1Value1 = pitch2 * 4;
    int gyro2Value1 = yaw2 * 4;

    int gyro0Value2 = map(gyro0Value1, -90, 90, pX, qX); //90-->22 == 4x lipat
    int gyro1Value2 = map(gyro1Value1, -90, 90, pY, qY);
    int gyro2Value2 = map(gyro2Value1, -90, 90, pZ, qZ);
    //----------------------------------------------------------

    // Servo dikontrol berdasarkan output dari MPU6050
    //  servo_x.write(servo0Value1);
    //  servo_y.write(servo1Value1);
    //  servo_z.write(servo2Value1);




    //---------------------------------------------------------------
    //                          State Observer
    //---------------------------------------------------------------

    y1 = spX - servo0Value1;
    y2 = spY - servo1Value1;
    y3 = spZ - servo2Value1;

    x1_hat_x = (a11_x * x1_hat_x) + (a12_x * x2_hat_x) + (a13_x * x3_hat_x) + b12_x * y1;
    //  x2_hat_x = (a21_x * x1_hat_x) + (a22_x * x2_hat_x) + (a23_x * x3_hat_x) + b22_x * y1;
    //  x3_hat_x = (a31_x * x1_hat_x) + (a32_x * x2_hat_x) + (a33_x * x3_hat_x) + b32_x * y1;

    //  x1_hat_y = (a11_y * x1_hat_y) + (a12_y * x2_hat_y) + (a13_y * x3_hat_y) + b12_y * y2;
    x2_hat_y = (a21_y * x1_hat_y) + (a22_y * x2_hat_y) + (a23_y * x3_hat_y) + b22_y * y2;
    //  x3_hat_y = (a31_y * x1_hat_y) + (a32_y * x2_hat_y) + (a33_y * x3_hat_y) + b32_y * y2;

    //  x1_hat_z = (a11_z * x1_hat_z) + (a12_z * x2_hat_z) + (a13_z * x3_hat_z) + b12_z * y3;
    //  x2_hat_z = (a21_z * x1_hat_z) + (a22_z * x2_hat_z )+ (a23_z * x3_hat_z) + b22_z * y3;
    x3_hat_z = (a31_z * x1_hat_z) + (a32_z * x2_hat_z) + (a33_z * x3_hat_z) + b32_z * y3;

    modify_xhat(x1_hat_x, x2_hat_y, x3_hat_z);

    Serial.print("Body X : ");
    Serial.print(servo0Value1);
    Serial.print("           Body Y : ");
    Serial.print(servo1Value1);
    Serial.print("           Body Z : ");
    Serial.print(servo2Value1);
    Serial.print("            Canon X : ");
    Serial.print(gyro0Value2);
    Serial.print("            Canon Y : ");
    Serial.print(gyro1Value2);
    Serial.print("            Canon Z : ");
    Serial.println(gyro2Value2);
  }
}


void calculate_IMU_error() {
  // Fungsi ini digunakan untuk menghitung nilai error dari accelerometer dan gyroscope
  // Posisi IMU harus diletakan dibidang yang datar untuk mendapatkan nilai yang baik

  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    // Semua bacaan dijumlahkan
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI)); // Rumus matriks rotasi, ada di data sheet
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  // Hasil dari penjumlahan dibagi dengan 200 untuk mendapatkan nilai errornya
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

    // Semua bacaan dijumlahkan
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  // Hasil dari penjumlahan dibagi dengan 200 untuk mendapatkan nilai errornya
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print error values pada Serial Monitor
  // Apabila tidak terdapat error maka akan bernilai 0
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

void calculate_IMU_error2() {
  // Fungsi ini digunakan untuk menghitung nilai error dari accelerometer dan gyroscope
  // Posisi IMU harus diletakan dibidang yang datar untuk mendapatkan nilai yang baik

  // Read accelerometer values 200 times
  while (c2 < 200) {
    Wire.beginTransmission(MPU2);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2, 6, true);
    AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    // Semua bacaan dijumlahkan
    AccErrorX2 = AccErrorX2 + ((atan((AccY2) / sqrt(pow((AccX2), 2) + pow((AccZ2), 2))) * 180 / PI));
    AccErrorY2 = AccErrorY2 + ((atan(-1 * (AccX2) / sqrt(pow((AccY2), 2) + pow((AccZ2), 2))) * 180 / PI));
    c2++;
  }

  // Hasil dari penjumlahan dibagi dengan 200 untuk mendapatkan nilai errornya
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

    // Semua bacaan dijumlahkan
    GyroErrorX2 = GyroErrorX2 + (GyroX2 / 131.0);
    GyroErrorY2 = GyroErrorY2 + (GyroY2 / 131.0);
    GyroErrorZ2 = GyroErrorZ2 + (GyroZ2 / 131.0);
    c2++;
  }
  // Hasil dari penjumlahan dibagi dengan 200 untuk mendapatkan nilai errornya
  GyroErrorX2 = GyroErrorX2 / 200;
  GyroErrorY2 = GyroErrorY2 / 200;
  GyroErrorZ2 = GyroErrorZ2 / 200;

  // Print error values pada Serial Monitor
  // Apabila tidak terdapat error maka akan bernilai 0
  //  Serial.print("AccErrorX: ");
  //  Serial.println(AccErrorX2);
  //  Serial.print("AccErrorY: ");
  //  Serial.println(AccErrorY2);
  //  Serial.print("GyroErrorX: ");
  //  Serial.println(GyroErrorX2);
  //  Serial.print("GyroErrorY: ");
  //  Serial.println(GyroErrorY2);
  //  Serial.print("GyroErrorZ: ");
  //  Serial.println(GyroErrorZ2);
}
//---------------------------------------------------------------

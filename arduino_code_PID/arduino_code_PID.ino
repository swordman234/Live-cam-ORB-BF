#include <Servo.h>
Servo servo_x;
Servo servo_y;
Servo servo_z;
int j = 0;
float correct_x;
float correct_y;
float correct_z;

// how much serial data we expect before a newline/enter
const unsigned int MAX_INPUT = 7;

float timenow = 0;
float timeend = 0;
float timeelapsed = 0;

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

float spZ = 90;
int pZ = 180;
int qZ = 0;

int raspiX = 0;
int raspiY = 0;

float kpx = 1.063040;
float kix = 0;
float kdx = 0.010467;

float kpy = 1.017864;
float kiy = 0;
float kdy = 0.017981;

float kpz = 1.019039;
float kiz = 0;
float kdz = 0.012509;


//---------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  servo_x.attach(8);                  // Set pin 8 untuk servo x
  servo_y.attach(9);                  // Set pin 9 untuk servo y
  servo_z.attach(10);                 // Set pin 10 untuk servo z

  servo_x.write(90);                  // Set posisi servo x ke 90 derajat
  servo_y.write(90);                  // Set posisi servo y ke 90 derajat
  servo_z.write(90);                  // Set posisi servo z ke 90 derajat
  delay(1000);                        // Memberikan delay agar servo sampai ke posisi set point

  Wire.begin();                      // Inisialisasi komunikasi
  Wire.beginTransmission(MPU);       // Memulai komunikasi dengan MPU1
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x01);                  // Make reset - memasukan nilai 0 ke register 6B
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
}

// split the data into its parts
void parseData(const char * data) {
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(data, ","); // this continues where the previous call left off
  raspiX = atoi(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  raspiY = atoi(strtokIndx);     // convert this part to an integer

}

// here to process incoming serial data after a terminator received
void process_data (const char * data)
  {
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  parseData(data);
  
  }  // end of process_data

void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
    {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte
      
      // terminator reached! process input_line here ...
      process_data (input_line);
      
      // reset buffer for next time
      input_pos = 0; 
      Serial.begin(115200); 
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch
   
  } // end of processIncomingByte  


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
  /*Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);*/
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
  /*Serial.print("AccErrorX: ");
  Serial.println(AccErrorX2);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY2);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX2);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY2);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ2);*/
}
//---------------------------------------------------------------

void loop() {
  timenow = millis();
  
  // === Read serial input if available ===//

  if(Serial.available() > 0) {
    //delay(100);
    processIncomingByte (Serial.read ());
    spX = raspiY;
    pX = raspiY + 90;
    qX = raspiY - 90;
    
    spZ = raspiX;
    pZ = raspiX + 90;
    qZ = raspiX - 90;
    }
  //delay(50);
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
  GyroX = GyroX - 1.40;          //0.74;
  GyroY = GyroY + 0.50;          //0.66;
  GyroZ = GyroZ + 0.02;          //0.03;

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
  GyroY2 = GyroY2 + 0.63;//0.11;          //
  GyroZ2 = GyroZ2 - 0.00;//1.69;          //

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


  int servo0Value1 = map(servo0Value, -90, 90, qX, pX);
  int servo1Value1 = map(servo1Value, -90, 90, 0, 180);
  int servo2Value1 = map(servo2Value, -90, 90, pZ, qZ);
  //  servo0Value1 =servo0Value1 +30;
  //if(servo0Value1<70){
  //  servo0Value1 =70;
  // }


  //----------------------------------------------------------

  int gyro0Value1 = roll2 * 4;
  int gyro1Value1 = pitch2 * 4;
  int gyro2Value1 = yaw2 * 4;

  int gyro0Value2 = map(gyro0Value1, -90, 90, qX, pX); //90-->22 == 4x lipat
  int gyro1Value2 = map(gyro1Value1, -90, 90, 0, 180);
  int gyro2Value2 = map(gyro2Value1, -90, 90, pZ, qZ);
  //----------------------------------------------------------

  // Servo dikontrol berdasarkan output dari MPU6050
  //  servo_x.write(servo0Value1);
  //  servo_y.write(servo1Value1);
  //  servo_z.write(servo2Value1);

  //---------------------------------------------------------------
  //                              PID
  //---------------------------------------------------------------

  errorx = spX - servo0Value1;
  px = kpx * errorx;
  ix = kix * (prev_errorx + errorx);
  dx = kdx * (errorx - prev_errorx);
  pidx = px + ix + dx;
  prev_errorx = errorx;
  int pid_x = spX - pidx;
  if (pid_x < 90) {
    pid_x = 90;
  }
  else if (pid_x > 130) {
    pid_x = 130;
  }

  errory = spY - servo1Value1;
  py = kpy * errory;
  iy = kiy * (prev_errory + errory);
  dy = kdy * (errory - prev_errory);
  pidy = py + iy + dy;
  prev_errory = errory;
  int pid_y = spY - pidy;
  if (pid_y < 70) {
    pid_y = 70;
  }
  else if (pid_y > 110) {
    pid_y = 110;
  }

  errorz = spZ - servo2Value1;
  pz = kpz * errorz;
  iz = kiz * (prev_errorz + errorz);
  dz = kdz * (errorz - prev_errorz);
  pidz = pz + iz + dz;
  prev_errorz = errorz;
  int pid_z = spZ - pidz;

  /*Serial.print("Body X : ");
  Serial.print(servo0Value1);
  Serial.print("           Body Y : ");
  Serial.print(servo1Value1);
  Serial.print("           Body Z : ");
  Serial.print(servo2Value1);*/
  /*Serial.print("            PID X : ");
    Serial.print(pid_x);
    Serial.print("            PID Y : ");
    Serial.print(pid_y);
    Serial.print("            PID Z : ");
    Serial.print(pid_z);*/
  /*Serial.print("            Canon X : ");
  Serial.print(gyro0Value2);
  Serial.print("            Canon Y : ");
  Serial.print(gyro1Value2);
  Serial.print("            Canon Z : ");
  Serial.println(gyro2Value2);*/
  servo_x.write(pid_x);
  servo_y.write(pid_y);                  // Set posisi servo y ke 90 derajat
  servo_z.write(pid_z);                  // Set posisi servo z ke 90 derajat
  
  /*Serial.print(raspiX);
  Serial.print(",");
  Serial.println(raspiY);*/
  /*Serial.print(spZ);
  Serial.print(",");
  Serial.println(spX);*/
  timeend = millis();
  timeelapsed = (timeend-timenow)/1000;
  Serial.println(timeelapsed);
  
  //---------------------------------------------------------------
  /*
    // Untuk mengetahui posisi dari body tank
    Serial.print("servo x : ");
    Serial.print(servo0Value1);
    Serial.print("            servo y : ");
    Serial.print(servo1Value1);
    Serial.print("            servo z : ");
    Serial.println(servo2Value1);

    //----------------------------------------------------------
    // Untuk mengetahui posisi dari ujung meriam tank
    Serial.println("");
    Serial.print("gyro x : ");
    Serial.print(gyro0Value2);
    Serial.print("            gyro y : ");
    Serial.print(gyro1Value2);
    Serial.print("            gyro z : ");
    Serial.println(gyro2Value2);
    //----------------------------------------------------------
  */
  //Serial.flush();
}

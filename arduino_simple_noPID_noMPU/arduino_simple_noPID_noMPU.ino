#include <Servo.h>
Servo servo_x;
Servo servo_y;
Servo servo_z;
// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 7;
int nilaiX = map(0, -90, 90, 0, 180);
int nilaiY = map(0, -90, 90, 0, 180);
int raspiX = 90;
int raspiY = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  servo_x.attach(8);                  // Set pin 8 untuk servo x
  servo_y.attach(9);                  // Set pin 9 untuk servo y
  servo_z.attach(10);                 // Set pin 10 untuk servo z
  
  servo_x.write(90);                  // Set posisi servo x ke 90 derajat
  servo_y.write(90);                  // Set posisi servo y ke 90 derajat
  servo_z.write(90);                  // Set posisi servo z ke 90 derajat
  delay(1000);                        // Memberikan delay agar servo sampai ke posisi set point
  

}

// split the data into its parts
void parseData(const char * data) {
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(data, ","); // this continues where the previous call left off
  raspiX = atoi(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  raspiY = atoi(strtokIndx);     // convert this part to a float

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
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch
   
  } // end of processIncomingByte  

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    processIncomingByte (Serial.read ());
  }
  //if (RaspiZ > -20 && RaspiZ < 20 && RaspiX > -20 && RaspiX < 20) {
    nilaiX = map(raspiX,-90,90,0,180);
    nilaiY = map(raspiY,-90,90,0,180);
  //}

  servo_x.write(raspiY);                  // sets the servo position according to the scaled value
  servo_y.write(90);                  // sets the servo position according to the scaled value
  servo_z.write(raspiX);                  // sets the servo position according to the scaled value

  /*
  Serial.print("X = ");
  Serial.print(nilaiX);
  Serial.print("      Y = ");
  Serial.println(nilaiY);
  Serial.print("raspi X = ");
  Serial.print(raspiX);
  Serial.print("      raspi Y = ");
  Serial.println(raspiY);
  */

  
}

/*
Program: Receive Integers From Raspberry Pi
File: receive_ints_from_raspberrypi.ino
Description: Receive integers from a Raspberry Pi
Author: Addison Sears-Collins
Website: https://automaticaddison.com
Date: July 5, 2020
*/
 
// Initialize the integer variables
int x = 90;
int y = 90;
 
int sum = 0;
 
void setup(){
   
  // Set the baud rate  
  Serial.begin(9600);
   
}
 
void loop(){
 
  if(Serial.available() > 0) {
    x = Serial.parseInt();
    y = Serial.parseInt(); 
 
    // Compute a sum to prove we have integers
    sum = x + y;
 
    // We do println to add a new line character '\n' at the end
    // of the comma-separated stream of integers
    Serial.print(x); Serial.print(",");
    Serial.print(y); Serial.print(",");
    Serial.println(sum); 
  }
}

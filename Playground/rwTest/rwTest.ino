//Current sensor Pins
#define Sensor_IM1  28

//Reaction Wheels PWM Pins
#define pin_PWM_M1  4

//Reaction Wheels direction pines
#define pin_dir_M1  3

//Reaction Wheel ecoder Pines
#define pin_enc_M1  2

//Debug flag
#define DEBUG true

#include <SD.h>
#include <SPI.h>

const int chipSelect = 53;
bool UpDown = false;
int i=0;
int n=0;
int cuentas1=0;

void setup() {
// Define inputs and outputs
Serial.begin(9600);
Serial.print("Initializing SD card...");

// See if the card is present and can be initialized:
if (!SD.begin(chipSelect)) {
Serial.println("Card failed, or not present");
}
Serial.println("card initialized.");

// Initialize Inputs and outPuts
pinMode(pin_PWM_M1,OUTPUT);
pinMode(pin_dir_M1,OUTPUT);
pinMode(Sensor_IM1,INPUT);
pinMode(pin_enc_M1,INPUT);
attachInterrupt(digitalPinToInterrupt(pin_enc_M1), ISR, RISING)
}

void loop() {
  
// Performing trapezoidal wave form
  if(i<255 && UpDown == false){
    i++;
  } else if (i==255 && UpDown == false){
    i=255;
    n++;
    if (n==255){
      n=0;
      UpDown=true;
    }
  } else if (UpDown==true && i>0){
    i--;
  } else if (UpDown==true && i==0) {
    i=0;
    n++;
    if (n==255){
      n=0;
      UpDown=false;
    }
  }

// Read angular velocity

  
  Serial.println(i);

// open the file. note that only one file can be open at a time,
// so you have to close this one before opening another.
//File dataFile = SD.open("datalog.txt", FILE_WRITE);

// if the file is available, write to it:
//if (dataFile) {
//  dataFile.print(i);
//  dataFile.print(",");
//  dataFile.close();
  // print to the serial port too:
//  Serial.println(dataString);
//}
}

void ISR(){
if ((digitalRead(pin_enc_M1)))
       cuentas1++;                 //Increase counter  
}

//Current sensor Pins
#define Sensor_IM1  A0
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

//Define SD card pin
const int chipSelect = 53;

//Trapezoidal wave form constants
bool UpDown = false;
int i=0;
int n=0;

//Encoder variables
volatile int cuentas1=0;

//Reaction wheek angular rate
float Wrw;

//Read current constants
int I_M1;
const float  sensibi_ACS712=0.185;              // Sensibilidad de de ACS712 [V/A]


//Perform amount of time
unsigned long time1;
unsigned long dt;

void ReadEnc(){
if ((digitalRead(pin_enc_M1)))
       cuentas1++;                 //Increase counter  
}

int getSensorASC712(byte pin_sensor, byte samplesNumber)
{
   long corrienteSum = 0;
   int corriente;
   int voltaje;
   if((analogRead(pin_sensor))>3105){
   for (int j = 0; j < samplesNumber; j++)
   {
      voltaje = ((analogRead(pin_sensor))*3300)/ 4095;  // Convertir LSB a voltaje
      corrienteSum += (voltaje - 2500)/sensibi_ACS712;  // Corriente [mA]
   }
  corriente= corrienteSum/samplesNumber;
   }
   else corriente=0;
  return (corriente); 
}

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
attachInterrupt(digitalPinToInterrupt(pin_enc_M1), ReadEnc, RISING);

}

void loop() {
//Get the amount of time
time1=millis(); 

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
//Send PWM value
analogWrite(pin_PWM_M1, i);

//Read current measure
 // I_M1=getSensorASC712(Sensor_IM1,20);
//if((analogRead(Sensor_IM1))>3105){
 I_M1=analogRead(Sensor_IM1);
//}

// Read angular velocity (RPM)
//Wrw=cuentas1*60/(6*dt*0.0001);
Wrw=cuentas1;
cuentas1=0;

delay(100);

//Get dt
dt=millis()-time1;
time1=0;

#if DEBUG
  Serial.print(i);
  Serial.print(",");
  Serial.print(Wrw);
  Serial.print(",");
  Serial.print(I_M1);
  Serial.print(",");
  Serial.println(dt);
#endif

// open the file. note that only one file can be open at a time,
// so you have to close this one before opening another.
File dataFile = SD.open("RW3prueba1.txt", FILE_WRITE);

// if the file is available, write to it:
if (dataFile) {
  dataFile.print(i);
  dataFile.print(",");
  dataFile.print(Wrw);
  dataFile.print(",");
  dataFile.print(I_M1);
  dataFile.print(",");
  dataFile.println(dt);
  dataFile.close();
}
}

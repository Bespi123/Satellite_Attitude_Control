/************************************************************************************
 * PROGRAM NAME: Reaction wheels prove and caraxterization
 * DESCRIPTION: 
 * This program test the CubeSat reaction wheels with an trapezoidal wave form as input
 * the program measure reaction wheels angular rates, and total current aplied to it. The
 * data in measure and sending to the PC serial port each 200ms.
 * 
 * IMPORTANT: The program is based in the program developed by Ing. Hammerly 
 * Mamani Valencia 
 * 
 * PROGRAM NAME: Reaction wheel test and caracterization 
 * 
 * AUTHOR: Bach. Brayan Espinoza (Bespi123)
 * 
 * DEPENDENCIES:
 * - Wire
 * - stdint
 * - stdbool 
 * - inc/hw_ints.h, and others than can be obtained in:
 *   https://github.com/energia/Energia/tree/master/hardware/cc2600emt/cores/cc2600emt
 */

/***************************************************************************************
 **                              Libraries and dependencies                           ** 
 ***************************************************************************************
 */
#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

/***************************************************************************************
 **                              Program definitions                                  ** 
 ***************************************************************************************
 */
// MPU6050 VALUES
#define  MPU6050_ADDRESS   0x68                      //MPU6050 I2C address
#define  PWR_MGMT_1                 0x00             //Sleep off         

//GYRO SCALE VALUES
#define  GYRO_FULL_SCALE_250_DPS    0x00           
#define  GYRO_FULL_SCALE_500_DPS    0x08
#define  GYRO_FULL_SCALE_1000_DPS   0x10
#define  GYRO_FULL_SCALE_2000_DPS   0x18
//ACC SCALE VALUES
#define  ACC_FULL_SCALE_2_G         0x00  
#define  ACC_FULL_SCALE_4_G         0x08
#define  ACC_FULL_SCALE_8_G         0x10
#define  ACC_FULL_SCALE_16_G        0x18

//MPU6050 Registers address
#define  addr_PWR_MGMT_1      0x6B 
#define  addr_ACC_FULL_SCALE  0x1C
#define  addr_GYRO_FULL_SCALE 0x1B
#define  addr_ACC_XOUT        0x3B
#define  addr_ACC_YOUT        0x3D
#define  addr_ACC_ZOUT        0x3F
#define  addr_TEMP_OUT        0x41
#define  addr_GYR_XOUT        0x43
#define  addr_GYR_YOUT        0x45
#define  addr_GYR_ZOUT        0x47
 
//Current sensor Pins
#define Sensor_IM1  28
#define Sensor_IM2  27
#define Sensor_IM3  26

//Reaction Wheels PWM Pins
#define pin_PWM_M1  40
#define pin_PWM_M2  39
#define pin_PWM_M3  38

//Reaction Wheels direction pines
#define pin_dir_M1  13
#define pin_dir_M2  12
#define pin_dir_M3  11

//Reaction Wheel ecoder Pines
#define pin_enc_M1  36
#define pin_enc_M2  35
#define pin_enc_M3  33

//Debug flag
#define DEBUG true

/***************************************************************************************
 **                              Program variables                                    ** 
 ***************************************************************************************
 */
/* 
//Trapezoidal wave form constants
bool UpDown = false;
int i=255;
int n=0;
*/

//Timer interruption Flag 
bool sta_procs=0;               

//Direction flag
bool dir=false;

//MPU6050 Variables
int16_t acel_X,acel_Y,acel_Z;         //MPU6050 Accelerations
int16_t Temp;                         //MPU6050 Temperature
int16_t gyr_X,gyr_Y,gyr_Z;            //MPU6050 angular rates
int16_t roll,pitch,yaw;               //Calculated Euler Angles
uint16_t Data1[6];                     //Data to send (Local variable)  
  

/*
//Encoder variables
volatile int cuentas1, cuentas2, cuentas3;
volatile int vel_M1, vel_M2, vel_M3;
*/

//Read current constants
volatile int16_t I_M1, I_M2, I_M3;
const float  sensibi_ACS712=0.185;  //ACS712 [V/A]

//MPU6050 Programs
// Gravity 9.82 m/s^2
const float  acel_escala = (2.0*9.82)/32768.0;  // scale_acel*g / size_data; bits -->  [m/s^2]
const float  gyr_escala = 250.0/32768.0;        // scale_gyr/ size_data; bits -->  [º/s]
const float  temp_escala=340;                   // Temperature sensibility [LSB/ºC]
long tiempo_prev, ts;

/***************************************************************************************
 **                      Encoder external interruption                                ** 
 ***************************************************************************************
 */
 /*
void encoder_M1(){
if ((digitalRead(pin_enc_M1)))
       cuentas1++;                 //Increase counter  
}

void encoder_M2(){
     if (digitalRead(pin_enc_M2))   
       cuentas2++;                 //Increase counter
}

void encoder_M3(){
     if (digitalRead(pin_enc_M3))
       cuentas3++;                 //Increase counter
}
/*
 * 
 */
/***************************************************************************************
 **                          Get current Function                                     ** 
 ***************************************************************************************
 */
 /*
int getSensorASC712(byte pin_sensor, byte samplesNumber)
{
  
   long corrienteSum = 0;
   int corriente;
   int voltaje;
 
   //if((analogRead(pin_sensor))>3105){
   for (int j = 0; j < samplesNumber; j++)
   {
      voltaje = ((analogRead(pin_sensor))*3300)/ 4095;  // Convertir LSB a voltaje
      corrienteSum += (voltaje - 2500)/sensibi_ACS712;  // Corriente [mA]
   }
  corriente= corrienteSum/samplesNumber;
  //} else corriente=0;
  return (corriente); 

}
 */
 /***************************************************************************************
 **                              Timer0 Interruption                                  ** 
 ***************************************************************************************
 */
void Timer0IntHandler() {
//Clean Overflow flag
MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       

// Measurements of current
//I_M1=getSensorASC712(Sensor_IM1,20);
//I_M2=getSensorASC712(Sensor_IM2,20);
//I_M3=getSensorASC712(Sensor_IM3,20);
   
// Read encoders
//vel_M1=cuentas1;
//vel_M2=cuentas2;
//vel_M3=cuentas3;

// Clear encoder varaibles
//cuentas1=0;
//cuentas2=0;
//cuentas3=0;

//Turn-on Interruption flag
sta_procs=1;
}

/***************************************************************************************
 **                             Write MPU6050 Subrutine                               ** 
 ***************************************************************************************
 */
void write_mpu6050(byte addr_device, byte addr_register,byte data)
 {
    Wire.beginTransmission(addr_device);          // Start comunication
    Wire.write(addr_register);                    // I2C direcction                               
    Wire.write(data);                             // Data to be writing            
    Wire.endTransmission(true);                   // End comunication
 }

/***************************************************************************************
 **                              Read MPU6050 Data registers                          ** 
 ***************************************************************************************
 */
 void Lectura_completa_MPU6050(){
// Read accelerometer 
read_mpu6050(MPU6050_ADDRESS,addr_ACC_XOUT,6);
// Single burst reading
acel_X=(Data1[0]<<8)| Data1[1];
acel_Y=(Data1[2]<<8)| Data1[3];
acel_Z=(Data1[4]<<8)| Data1[5];
   
// Read gyro  
read_mpu6050(MPU6050_ADDRESS,addr_GYR_XOUT,6);
//Single burst reading
gyr_X=(Data1[0]<<8)| Data1[1];
gyr_Y=(Data1[2]<<8)| Data1[3];
gyr_Z=(Data1[4]<<8)| Data1[5];

// TEMP Reading
read_mpu6050(MPU6050_ADDRESS,addr_TEMP_OUT,2);

//Single burst reading
Temp=(Data1[0]<<8)| Data1[1];

// Scale values:  acel -> [m/s^2]
acel_X=acel_X*acel_escala;
acel_Y=acel_Y*acel_escala;
acel_Z=acel_Z*acel_escala;
//  gyr -> [º/seg]
gyr_X=gyr_X*gyr_escala; 
gyr_Y=gyr_Y*gyr_escala;  
gyr_Z=gyr_Z*gyr_escala; 
//    temp -> [Celcius]
Temp=(Temp/temp_escala)+36.53;   // Ecuacion dada en Datasheet de MPU6050
}

/***************************************************************************************
 **                             Euler Angles calculation                               ** 
 ***************************************************************************************
 */
void Calculo_ang_euler()
{
  roll = atan(acel_Y / sqrt(pow(acel_X, 2) + pow(acel_Z, 2))) * (180.0 / 3.14);
  pitch = atan(acel_X / sqrt(pow(acel_Y, 2) + pow(acel_Z, 2))) * (180.0 / 3.14);
  yaw = yaw + (gyr_Z * ts) / 1000;
}

 /***************************************************************************************
 **                              Read MPU6050 Subrutine                               ** 
 ***************************************************************************************
 */
void read_mpu6050(byte addr_device, byte addr_register,byte cant)
 {
    Wire.beginTransmission(addr_device);          //Start comunication
    Wire.write(addr_register);                    //I2C direction
    Wire.endTransmission(false);                  

    //Request information
    Wire.requestFrom(addr_device, cant,true);     
    byte index = 0;
    while (Wire.available())
    {
      Data1[index++] = Wire.read();
    }   
 }
 


/***************************************************************************************
 **                                  MAIN PROGRAM                                     ** 
 ***************************************************************************************
 */
void setup() { 
//Initialize serial port
Serial.begin(9600);

// Initialize Inputs and outPuts

//Set pin PF2, PF3, PB3 as pwm outputs from motors
pinMode(pin_PWM_M1,OUTPUT);
pinMode(pin_PWM_M2,OUTPUT);
pinMode(pin_PWM_M3,OUTPUT);

//Set pin PA4, PA3, PA2 as outputs to control rotation direction
pinMode(pin_dir_M1,OUTPUT);
pinMode(pin_dir_M2,OUTPUT);
pinMode(pin_dir_M3,OUTPUT);

//Set pin PC5, PC6, PD6 as encoder inputs 
pinMode(pin_enc_M1,INPUT);     
pinMode(pin_enc_M2,INPUT);
pinMode(pin_enc_M3,INPUT);

//Attach external interrupt for the encoder
//attachInterrupt(digitalPinToInterrupt(pin_enc_M1), encoder_M1, RISING);
//attachInterrupt(digitalPinToInterrupt(pin_enc_M2), encoder_M2, RISING);
//attachInterrupt(digitalPinToInterrupt(pin_enc_M3), encoder_M3, RISING);

/*
// Set Timer 0
MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                    //Enable Timer 0 
MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);                 //Set TIMER 0, in periodic mode
MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet()/5);      //Set TIMER 0,overflow period: 200 mseg
TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer0IntHandler);           //Set Interruption function
MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                 //Enable timer 0 with overflow

//Set TIMER 0 interruption: counting start
MAP_TimerEnable(TIMER0_BASE, TIMER_A); 
*/

 // CONFIGURACION MPU9250
  // Configuracion  Registro de Administrador de energia: Oscilador 20Hz, NO en modo Sleep, 
  // No se resetea configuraciones Anteriores
  write_mpu6050(MPU6050_ADDRESS,addr_PWR_MGMT_1,PWR_MGMT_1);                   
  // Configuracion de Acelerometro +-2g                                       
  write_mpu6050(MPU6050_ADDRESS,addr_ACC_FULL_SCALE,ACC_FULL_SCALE_2_G);         
  // Configuracion de giroscopio   +-250 º/s   
  write_mpu6050(MPU6050_ADDRESS,addr_GYRO_FULL_SCALE,GYRO_FULL_SCALE_250_DPS);  
  
}

void loop() {
  ts = millis()-tiempo_prev;
  tiempo_prev=millis();
  
  //Read MPU data
  Lectura_completa_MPU6050();
    
 //Get Euler angles from MPU data
 // Calculo_ang_euler();

//Send data throuth Bluetooth
 Serial.print('r');
  Serial.print(roll);     
 Serial.print(',');
  Serial.print(pitch);    
 Serial.print(',');
  Serial.print(yaw);     
 Serial.print(',');
  Serial.print(acel_X);    
 Serial.print(',');
  Serial.print(acel_Y);     
 Serial.print(',');      
  Serial.print(acel_Z);     
 Serial.print(',');
  Serial.print(gyr_X);    
 Serial.print(',');
  Serial.print(gyr_Y);      
 Serial.print(',');    
  Serial.println(gyr_Z);     

   /*
if(sta_procs==1){

  /*
  // Performing trapezoidal wave form
  if(i<255 && UpDown == false){
    i++;
  } else if (i==255 && UpDown == false){
    i=255;
    n++;
    if (n==10){
      n=0;
      UpDown=true;
    }
  } else if (UpDown==true && i>-255){
    i--;
  } else if (UpDown==true && i==-255) {
    i=-255;
    n++;
    if (n==10){
      n=0;
      UpDown=false;
    }
  }

  //Get sign
  if(i<0)
    dir=true;
  else
    dir=false;
      
  /*
  //Performing square signal
  n++;
  if(n<100){
    i=255;
  }else if(n<355){
    i=0;
  }else{
    dir=!dir;
    n=0;
  }
  */
  //Send direction
  //digitalWrite(pin_dir_M1,dir);
  //digitalWrite(pin_dir_M2,dir);
  //digitalWrite(pin_dir_M3,dir);
  
  //Send PWM value
  //analogWrite(pin_PWM_M1, abs(i));
  //analogWrite(pin_PWM_M2, abs(i));
  //analogWrite(pin_PWM_M3, abs(i));
/*
  #if DEBUG
    //Serial.print(i);
    //Serial.print(",");
    //Serial.println(vel_M1);
    //Serial.print(",");
    //Serial.print(vel_M2);
    //Serial.print(",");
    //Serial.println(vel_M3);
    
  #endif

  //Clean Interruption flag
  sta_procs=0; 
}
*/
//delay(500);
}

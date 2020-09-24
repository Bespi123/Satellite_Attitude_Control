/************************************************************************************
 * TESIS: Design and Implementation of test platform for control and determination of 
 * picosatelite attitude CubeSat
 * PROGRAM NAME: Cubesat monitoring v7
 * DESCRIPTION: 
 * Procesamiento de datos de sensores aprox cada 200ms
 * 
 * AUTHOR: Ing. Hammerly Mamani Valencia
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
#define  MPU6050_ADDRESS   0x68                      //MPU6050 I2C address

// MPU6050 VALUES
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
 **                              Program Variables                                    ** 
 ***************************************************************************************
 */
int16_t acel_X,acel_Y,acel_Z;         //MPU6050 Accelerations
int16_t Temp;                         //MPU6050 Temperature
int16_t gyr_X,gyr_Y,gyr_Z;            //MPU6050 angular rates
int16_t roll,pitch,yaw;               //Calculated Euler Angles
int16_t Data1[6];                     //Data to send (Local variable)  
int16_t I_M1,I_M2,I_M3;               //Current sensor values (volatile)
bool sta_procs=0;                     //Timer interruption Flag  
byte dataofPC[10];                    //Comming data from PC  
int cuentas1,cuentas2,cuentas3;       //Encoder counter (volatile)
int pwm_M1,pwm_M2,pwm_M3;             //PWM scale
int vel_M1,vel_M2,vel_M3;             //Encoder Angular velocity 
int dir_M1,dir_M2,dir_M3;             //Direction Rotation

/***************************************************************************************
 **                              Program constants                                    ** 
 ***************************************************************************************
 */
// Gravity 9.82 m/s^2
const float  acel_escala = (2.0*9.82)/32768.0;  // scale_acel*g / size_data; bits -->  [m/s^2]
const float  gyr_escala = 250.0/32768.0;        // scale_gyr/ size_data; bits -->  [º/s]
const float  temp_escala=340;                   // Temperature sensibility [LSB/ºC]
const float  sensibi_ACS712=0.185;              // ACS712 sensibility [V/A]
const int    ts=200;                            // Sample time of Ecoder, MPU6050, and current sensor


/***************************************************************************************
 **                              Main Program                                         ** 
 ***************************************************************************************
 */
void setup() {
//Initialize serial and i2c ports 
Serial5.begin(9600);         //bluetooth module 
Wire.begin();                //MPU605 port

//Set pin PC5, PC6, PD6 as encoder inputs 
pinMode(pin_enc_M1,INPUT);     
pinMode(pin_enc_M2,INPUT);
pinMode(pin_enc_M3,INPUT);

//Set pin PF2, PF3, PB3 as pwm outputs from motors
pinMode(pin_PWM_M1,OUTPUT);
pinMode(pin_PWM_M2,OUTPUT);
pinMode(pin_PWM_M3,OUTPUT);
  
//Set pin PA4, PA3, PA2 as outputs to control rotation direction
pinMode(pin_dir_M1,OUTPUT);
pinMode(pin_dir_M2,OUTPUT);
pinMode(pin_dir_M3,OUTPUT);
  
//Set external int in PC5, PC6, PD6 to read encoder pulses
attachInterrupt(pin_enc_M1,encoder_M1,RISING);
attachInterrupt(pin_enc_M2,encoder_M2,RISING);
attachInterrupt(pin_enc_M3,encoder_M3,RISING);
  
// Set Timer 2
MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                    //Enable Timer 0 
MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);                 //Set TIMER 0, in periodic mode
MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet()/5);      //Set TIMER 0,overflow period: 200 mseg
TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer0IntHandler);           //Set Interruption function
MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                 //Enable timer 0 with overflow
  
// SET MPU9250
//Set powerManagment register: 20Hz Oscillator , NO Sleep mode, 
//No reset in before settings
write_mpu6050(MPU6050_ADDRESS,addr_PWR_MGMT_1,PWR_MGMT_1);                   
//Acclerometer range +-2g                                       
write_mpu6050(MPU6050_ADDRESS,addr_ACC_FULL_SCALE,ACC_FULL_SCALE_2_G);         
//Gyro config range   +-250 º/s   
write_mpu6050(MPU6050_ADDRESS,addr_GYRO_FULL_SCALE,GYRO_FULL_SCALE_250_DPS);      

//Set TIMER 0 interruption: counting start
MAP_TimerEnable(TIMER0_BASE, TIMER_A); 

//Turn off Reaction Wheels
analogWrite(pin_PWM_M1, 255);
analogWrite(pin_PWM_M2, 255);
analogWrite(pin_PWM_M3, 255);
}

void loop() {
// Check Timer0 interruption flag
if(sta_procs==1){  
  //Get Euler angles from MPU data
  Calculo_ang_euler();
  //Send data throuth Bluetooth
  Serial5.write('i');
  send_data1_toPC(roll);     
  Serial5.write('j');
  send_data1_toPC(pitch);    
  Serial5.write('k');
  send_data1_toPC(yaw);     
  Serial5.write('l');
  send_data1_toPC(acel_X);    
  Serial5.write('m');
  send_data1_toPC(acel_Y);     
  Serial5.write('n');      
  send_data1_toPC(acel_Z);     
  Serial5.write('o');
  send_data1_toPC(gyr_X);    
  Serial5.write('p');
  send_data1_toPC(gyr_Y);      
  Serial5.write('q');    
  send_data1_toPC(gyr_Z);      
  Serial5.write('r');
  send_data2_toPC(vel_M1);      
  Serial5.write('s');
  send_data2_toPC(I_M1);      
  Serial5.write('t');      
  send_data2_toPC(vel_M2);      
  Serial5.write('u');
  send_data2_toPC(I_M2);     
  Serial5.write('v');
  send_data2_toPC(vel_M3);      
  Serial5.write('w');
  send_data2_toPC(I_M3);     
  
  //Clean Interruption flag
  sta_procs=0;    
}
   
//Read data from PC
if(Serial5.available()>0){ 
  Serial5.readBytes(dataofPC,10);
  // DECODIFICATION FRAME
  // Data direction to Motor 1
  if(dataofPC[3]==16) digitalWrite(pin_dir_M1,HIGH);
  else  digitalWrite(pin_dir_M1,LOW); 
      
  // Data de direction to Motor 2
  if(dataofPC[6]==16) digitalWrite(pin_dir_M1,HIGH);
  else  digitalWrite(pin_dir_M1,LOW);

  // Data direction to Motor 3
  if(dataofPC[9]==16) digitalWrite(pin_dir_M1,HIGH);
  else  digitalWrite(pin_dir_M1,LOW);

   // Recived PWM from reaction wheels
   if((dataofPC[0])== 224){
      pwm_M1= dataofPC[2]*100+dataofPC[1];
      analogWrite(pin_PWM_M1, pwm_M1);
      pwm_M2= dataofPC[5]*100+dataofPC[4];
      analogWrite(pin_PWM_M2, pwm_M2);
      pwm_M3= dataofPC[8]*100+dataofPC[7];
      analogWrite(pin_PWM_M3, pwm_M3);                
    }

    //ON/OFF Reaction wheels
    if((dataofPC[0])== 239){
      // Turn off motors
      analogWrite(pin_PWM_M1,255);
      analogWrite(pin_PWM_M2,255);
      analogWrite(pin_PWM_M3,255);
    }
}
}

/***************************************************************************************
 **                              Timer0 Interruption                                  ** 
 ***************************************************************************************
 */
void Timer0IntHandler() {
//Clean Overflow flag
MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       

//Read MPU data
Lectura_completa_MPU6050();
    
// Measurements of current
I_M1=getSensorASC712(Sensor_IM1,20);
I_M2=getSensorASC712(Sensor_IM2,20);
I_M3=getSensorASC712(Sensor_IM3,20);
   
// Read encoders
vel_M1=cuentas1;
vel_M2=cuentas2;
vel_M3=cuentas3;
// Clear encoder varaibles
cuentas1=0;
cuentas2=0;
cuentas3=0;

//Turn-on Interruption flag
sta_procs=1;
}


/***************************************************************************************
 **                         Encoder external Interruptions                            ** 
 ***************************************************************************************
 */
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
 **                              Calculation of motor currents                        ** 
 ***************************************************************************************
 */
int getSensorASC712(byte pin_sensor, byte samplesNumber){
//Varibles   
long corrienteSum = 0;
int corriente;
int voltaje;
//If read value es grater than 3105
if((analogRead(pin_sensor))>3105){
   for (int j = 0; j < samplesNumber; j++)
   {
      voltaje = ((analogRead(pin_sensor))*3300)/ 4095;  // Turn LSB to voltaje
      corrienteSum += (voltaje - 2500)/sensibi_ACS712;  // Current [mA]
   }
  corriente= corrienteSum/samplesNumber; //Mean current measure
}
   else corriente=0;
  return (corriente); 
}

/***************************************************************************************
 **                              Send data to PC function                             ** 
 ***************************************************************************************
 */
void send_data1_toPC(int16_t data_toPC){
//Variables
int num;
byte aux_data;
byte sign;
   
//Data frame
if (data_toPC>=0) sign=10;    
else sign=0;
data_toPC=abs(data_toPC);
num=data_toPC;
aux_data=(num)%100;

//Send data to PC
Serial5.write(aux_data);              // Envio de unidad y decenas
Serial5.write((num-aux_data)/100);    // Envio de centenas
Serial5.write(sign);                  // Envio de signo de numero
}

/***************************************************************************************
 **                 Send data without Sign and lower than 9999                        ** 
 ***************************************************************************************
 */
void send_data2_toPC(int data_toPC){
//Variables
int num;
byte aux_data;
   
//Data frame
data_toPC=abs(data_toPC);
num=data_toPC;
aux_data=(num)%100;

//Send data to PC
Serial5.write(aux_data);              // Envio de unidad y decenas
Serial5.write((num-aux_data)/100);    // Envio de centenas
}

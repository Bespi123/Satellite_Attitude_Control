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
//Trapezoidal wave form constants
bool UpDown = false;
int i=0;
int n=0;

//Timer interruption Flag 
bool sta_procs=0;                     

//Encoder variables
volatile int cuentas1, cuentas2, cuentas3;
volatile int vel_M1, vel_M2, vel_M3;

//Read current constants
volatile int16_t I_M1, I_M2, I_M3;
const float  sensibi_ACS712=0.185;  //ACS712 [V/A]

/***************************************************************************************
 **                      Encoder external interruption                                ** 
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
 **                          Get current Function                                     ** 
 ***************************************************************************************
 */
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
attachInterrupt(digitalPinToInterrupt(pin_enc_M1), encoder_M1, RISING);
attachInterrupt(digitalPinToInterrupt(pin_enc_M2), encoder_M2, RISING);
attachInterrupt(digitalPinToInterrupt(pin_enc_M3), encoder_M3, RISING);

// Set Timer 0
MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                    //Enable Timer 0 
MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);                 //Set TIMER 0, in periodic mode
MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet()/5);      //Set TIMER 0,overflow period: 200 mseg
TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer0IntHandler);           //Set Interruption function
MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                 //Enable timer 0 with overflow

//Set TIMER 0 interruption: counting start
MAP_TimerEnable(TIMER0_BASE, TIMER_A); 
}

void loop() {
  
if(sta_procs==1){
  /*
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
    
  */
  //Performing square signal
  n++;
  if(n<100){
    i=255;
  }else if(n<355){
    i=0;
  }else{
    n=0;
  }

  //Send PWM value
  analogWrite(pin_PWM_M1, i);
  analogWrite(pin_PWM_M2, i);
  analogWrite(pin_PWM_M3, i);

  #if DEBUG
    Serial.print(255-i);
    Serial.print(",");
    Serial.print(vel_M1);
    Serial.print(",");
    Serial.print(vel_M2);
    Serial.print(",");
    Serial.println(vel_M3);
    
  #endif

  //Clean Interruption flag
  sta_procs=0; 
}
}

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

///////////////////////////////////////////////////////////////////////////////////
//                                      TESIS                                    //
//             Diseño e Implementacion de plataforma de pruebas para             //
//             control y determinacion de actitud de picosatelite                //
//                                     CubeSat                                   //
//                       Autor: Hammerly Mamani Valencia                         //
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//                                Programa                                       //
//                           Monitoreo de CubeSat                                //
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//  LIBRERIAS  ////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>
//#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "driverlib/debug.h"
//#include "driverlib/gpio.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/rom.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/timer.h"
///////////////////////////////////////////////////////////////////////////////////

//  DEFINICION DE IDENTIFICADORES  ////////////////////////////////////////////////
#define  MPU6050_ADDRESS   0x68
#define  MAGNET_ADDRESS    0x0C

#define  PWR_MGMT_1    0x00 
#define  INT_PIN_CFG   0x02
#define  GYRO_FULL_SCALE_250_DPS    0x00  
#define  GYRO_FULL_SCALE_500_DPS    0x08
#define  GYRO_FULL_SCALE_1000_DPS   0x10
#define  GYRO_FULL_SCALE_2000_DPS   0x18
#define  ACC_FULL_SCALE_2_G         0x00  
#define  ACC_FULL_SCALE_4_G         0x08
#define  ACC_FULL_SCALE_8_G         0x10
#define  ACC_FULL_SCALE_16_G        0x18
#define  CNTL1         0x16

#define  addr_PWR_MGMT_1      0x6B 
#define  addr_INT_PIN_CFG     0x37
#define  addr_ACC_FULL_SCALE  0x1C
#define  addr_GYRO_FULL_SCALE 0x1B
#define  addr_ACC_XOUT        0x3B
#define  addr_ACC_YOUT        0x3D
#define  addr_ACC_ZOUT        0x3F

#define  addr_TEMP_OUT        0x41

#define  addr_GYR_XOUT        0x43
#define  addr_GYR_YOUT        0x45
#define  addr_GYR_ZOUT        0x47
#define  addr_CNTL1           0x0A    // Configuracion: medicion continua Modo 2, 16 bits de salida
#define  addr_ST1   0x02
#define  addr_MAGNET_XOUT   0X03
#define  addr_MAGNET_YOUT   0X05
#define  addr_MAGNET_ZOUT   0X07

// Pines de sensores de corriente
#define Sensor_IM1  28
#define Sensor_IM2  27
#define Sensor_IM3  26

// Pines de control de velocidad de Ruedas de Reaccion
#define pin_PWM_M1  40
#define pin_PWM_M2  39
#define pin_PWM_M3  38

// Pines para control de direccion de Ruedas de Reaccion
#define pin_dir_M1  13
#define pin_dir_M2  12
#define pin_dir_M3  11

// Pines de encoder de  Ruedas de Reaccion
#define pin_enc_M1  36
#define pin_enc_M2  35
#define pin_enc_M3  33
///////////////////////////////////////////////////////////////////////////////////

//  DEFINICION DE VARIALBES  //////////////////////////////////////////////////////
int16_t acel_X,acel_Y,acel_Z;
int16_t Temp;
int16_t gyr_X,gyr_Y,gyr_Z;
int16_t roll,pitch,yaw;
int16_t Data1[6];
int16_t I_M1,I_M2,I_M3;
bool sta_procs=0;   
byte dataofPC[10];
int cuentas1,cuentas2,cuentas3;
int pwm_M1,pwm_M2,pwm_M3;
int vel_M1,vel_M2,vel_M3;
int dir_M1,dir_M2,dir_M3;

///////////////////////////////////////////////////////////////////////////////////
//  CONSTANTES: FACTORES DE CONVERSION  ///////////////////////////////////////////

// Gravedad 9.82 m/s^2
const float  acel_escala = (2.0*9.82)/32768.0;  // scale_acel*g / size_data; bits -->  [m/s^2]
const float  gyr_escala = 250.0/32768.0;        // scale_gyr/ size_data; bits -->  [º/s]
const float  temp_escala=340;                   // Sensibilidad de tempoeratura [LSB/ºC]
const float  sensibi_ACS712=0.185;              // Sensibilidad de de ACS712 [V/A]
const int    ts=200;                            // Periodo de muestreo de Encoder, MPU6050 y sensor de Corriente
///////////////////////////////////////////////////////////////////////////////////

//  CONFIGURACION PRINCIPAL  //////////////////////////////////////////////////////
void setup() {

  Serial5.begin(9600);          // Inicializamos el puerto serie para Modulo Bluetooth 
  Wire.begin();                 // Inicializamos  el puerto de comunicacion I2C
  
  // Configurar pin PC5, PC6, PD6 como entradas para encoder 
  pinMode(pin_enc_M1,INPUT);     
  pinMode(pin_enc_M2,INPUT);
  pinMode(pin_enc_M3,INPUT);
  
 // Configurar pin PF2, PF3, PB3 como salidas de control de pwm para motores
  pinMode(pin_PWM_M1,OUTPUT);
  pinMode(pin_PWM_M2,OUTPUT);
  pinMode(pin_PWM_M3,OUTPUT);
  
  // Configurar pin PA4, PA3, PA2 como salidas de control de sentido de Giro
  pinMode(pin_dir_M1,OUTPUT);
  pinMode(pin_dir_M2,OUTPUT);
  pinMode(pin_dir_M3,OUTPUT);
  
  // Configurar interrupcion externa usando PC5, PC6, PD6 para lectura de pulsos de encoder
  attachInterrupt(pin_enc_M1,encoder_M1,RISING);
  attachInterrupt(pin_enc_M2,encoder_M2,RISING);
  attachInterrupt(pin_enc_M3,encoder_M3,RISING);
  
  // CONFIGURACION DE TIMER 2
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                    //Habilitacion de TIMER 0
  MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);                 //Configuracion de TIMER 0,modo periodico
  MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet()/5);      //Configuracion de TIMER 0, periodo de desbordamiento: configurado a 200 mseg
  TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer0IntHandler);           //Configuracion de funcion a ejecutar cuando se habilite la interrupcion
  MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                 //Habilitacion de timer 0 en funcion al desbordamiento
  
  // CONFIGURACION MPU9250
  // Configuracion  Registro de Administrador de energia: Oscilador 20Hz, NO en modo Sleep, 
  // No se resetea configuraciones Anteriores
  write_mpu6050(MPU6050_ADDRESS,addr_PWR_MGMT_1,PWR_MGMT_1);                   
  // Configuracion de Acelerometro +-2g                                       
  write_mpu6050(MPU6050_ADDRESS,addr_ACC_FULL_SCALE,ACC_FULL_SCALE_2_G);         
  // Configuracion de giroscopio   +-250 º/s   
  write_mpu6050(MPU6050_ADDRESS,addr_GYRO_FULL_SCALE,GYRO_FULL_SCALE_250_DPS);      

  //Habilita interrupcion de TIMER 0: comienza a contar
  MAP_TimerEnable(TIMER0_BASE, TIMER_A); 

  // Inicializamos Motores apagados
  analogWrite(pin_PWM_M1, 255);
  analogWrite(pin_PWM_M2, 255);
  analogWrite(pin_PWM_M3, 255);

}

///////////////////////////////////////////////////////////////////////////////////

//  PROGRAMA PRINCIPAL  ///////////////////////////////////////////////////////////
void loop() {

   // Procesamiento de datos de sensores aprox  cada 200ms
   // Procesamiento y envio de datos por bluetooth *************
   if(sta_procs==1)
   {
      // Procesamiento de datos de IMU, Encoder
      Calculo_ang_euler();
  
      // Envio de datos por Bluetooth
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

      sta_procs=0; 
   }
   
   // Lectura de datos de PC ********************************** 
   if(Serial5.available()>0)
   {
     // Recepcion de datos 
     Serial5.readBytes(dataofPC,10);

     // DECODIFICACION DE TRAMA DE DATOS
     // Datos de dirección para Motor 1
     if(dataofPC[3]==16) digitalWrite(pin_dir_M1,HIGH);
      else  digitalWrite(pin_dir_M1,LOW); 
      
     // Dato de dirección para Motor 2
     if(dataofPC[6]==16) digitalWrite(pin_dir_M1,HIGH);
      else  digitalWrite(pin_dir_M1,LOW);

     // Datos de dirección para Motor 3
     if(dataofPC[9]==16) digitalWrite(pin_dir_M1,HIGH);
      else  digitalWrite(pin_dir_M1,LOW);

     // Dato de ON/OFF de motores y PWM de motores
     if((dataofPC[0])== 224){
        pwm_M1= dataofPC[2]*100+dataofPC[1];
        analogWrite(pin_PWM_M1, pwm_M1);
        pwm_M2= dataofPC[5]*100+dataofPC[4];
        analogWrite(pin_PWM_M2, pwm_M2);
        pwm_M3= dataofPC[8]*100+dataofPC[7];
        analogWrite(pin_PWM_M3, pwm_M3);                
        }
      if((dataofPC[0])== 239){
        // Se apaga motores
        analogWrite(pin_PWM_M1,255);
        analogWrite(pin_PWM_M2,255);
        analogWrite(pin_PWM_M3,255);
      }
    }
}
///////////////////////////////////////////////////////////////////////////////////

// SUBRUTINA DE INTERRUPCION POR TIMER 0  /////////////////////////////////////////
void Timer0IntHandler() {
  MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       //Limpia Bandera de desbordamiento de TIMER 0

  // Lectura de datos de Sensor IMU
  Lectura_completa_MPU6050();
  
  // Procesamiento de datos de Corriente
  I_M1=getSensorASC712(Sensor_IM1,20);
  I_M2=getSensorASC712(Sensor_IM2,20);
  I_M3=getSensorASC712(Sensor_IM3,20);
   
  // Lectura de datos de Encoder de Motores
  vel_M1=cuentas1;
  vel_M2=cuentas2;
  vel_M3=cuentas3;
  cuentas1=0;
  cuentas2=0;
  cuentas3=0;
  sta_procs=1;
}
///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA DE INTERRUPCION EXTERNA PARA LECTURA DE ENCODER DE MOTOR1 ///////////
void encoder_M1(){
     if ((digitalRead(pin_enc_M1))==1)   // Si PC_5 se ha puesto a 1 (flanco de subida),
   {  
       cuentas1++;                 // entonces incrementar una unidad el valor de X.
   }
}
///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA DE INTERRUPCION EXTERNA PARA LECTURA DE ENCODER DE MOTOR2 ///////////
void encoder_M2(){
     if ((digitalRead(pin_enc_M2))==1)   // Si PC_6 se ha puesto a 1 (flanco de subida),
   {  
       cuentas2++;                 // entonces incrementar una unidad el valor de X.
   }
}
///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA DE INTERRUPCION EXTERNA PARA LECTURA DE ENCODER DE MOTOR 3 //////////
void encoder_M3(){
     if ((digitalRead(pin_enc_M3))==1)   // Si PD_6 se ha puesto a 1 (flanco de subida),
   {  
       cuentas3++;                 // entonces incrementar una unidad el valor de X.
   }
}

///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA DE LECTURA COMPLETA DE MPU9250  /////////////////////////////////////
 void Lectura_completa_MPU6050()
 {
    // Lectura de ACELEROMETRO 
   read_mpu6050(MPU6050_ADDRESS,addr_ACC_XOUT,6);
   // Concatenar Datos
   acel_X=(Data1[0]<<8)| Data1[1];
   acel_Y=(Data1[2]<<8)| Data1[3];
   acel_Z=(Data1[4]<<8)| Data1[5];
   
   // Lectura de GIROSCOPIO 
   read_mpu6050(MPU6050_ADDRESS,addr_GYR_XOUT,6);
   // Concatenar Datos
   gyr_X=(Data1[0]<<8)| Data1[1];
   gyr_Y=(Data1[2]<<8)| Data1[3];
   gyr_Z=(Data1[4]<<8)| Data1[5];

   // Lectura de TEMPERATURA 
   read_mpu6050(MPU6050_ADDRESS,addr_TEMP_OUT,2);
   // Concatenar datos
   Temp=(Data1[0]<<8)| Data1[1];

   ///// Escalado de valores  ////////////////
   //  acel -> [m/s^2]
   acel_X=acel_X*acel_escala;
   acel_Y=acel_Y*acel_escala;
   acel_Z=acel_Z*acel_escala;

   //  gyr -> [º/seg]
   gyr_X=gyr_X*gyr_escala; 
   gyr_Y=gyr_Y*gyr_escala;  
   gyr_Z=gyr_Z*gyr_escala; 

   //  gyr -> [Celcius]
   Temp=(Temp/temp_escala)+36.53;   // Ecuacion dada en Datasheet de MPU6050
 }

///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA CALCULO DE ANGULOS DE EULER  ////////////////////////////////////////
void Calculo_ang_euler()
{
  roll = atan(acel_Y / sqrt(pow(acel_X, 2) + pow(acel_Z, 2))) * (180.0 / 3.14);
  pitch = atan(acel_X / sqrt(pow(acel_Y, 2) + pow(acel_Z, 2))) * (180.0 / 3.14);
  yaw = yaw + (gyr_Z * ts) / 1000;
}
///////////////////////////////////////////////////////////////////////////////////

//   SUBRUTINA DE ESCRITURA EN MPU9250 - COMUNICACION I2C  ////////////////////////
void write_mpu6050(byte addr_device, byte addr_register,byte data)
 {
    Wire.beginTransmission(addr_device);          // Inicializa la comunicacion 
    Wire.write(addr_register);                    // Direccion                               
    Wire.write(data);                             // Dato                      
    Wire.endTransmission(true);                   // Finaliza la comunicacion
 }

///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA DE LECTURA EN MPU9250 - COMUNICACION I2C  ///////////////////////////
void read_mpu6050(byte addr_device, byte addr_register,byte cant)
 {
    Wire.beginTransmission(addr_device);
    Wire.write(addr_register);
    Wire.endTransmission(false);

    Wire.requestFrom(addr_device, cant,true);
    byte index = 0;
    while (Wire.available())
    {
      Data1[index++] = Wire.read();
    }   
 }
///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA CALCULO DE CORRIENTE DE MOTORES  ////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////////


//  SUBRUTINA DE ENVIO DE DATOS A PC: CON SIGNO SIN DECIMAL CON  TAMAÑO MENOR A 9999 
 void send_data1_toPC(int16_t data_toPC)
 {
   int num;
   byte aux_data;
   int sign;
   
   // Ordenamiento de trama de datos y envio a PC

    if (data_toPC>=0) sign=10;
    else sign=0;
    data_toPC=abs(data_toPC);
    num=data_toPC;
    aux_data=(num)%100;
    Serial5.write(aux_data);              // Envio de unidad y decenas
    Serial5.write((num-aux_data)/100);    // Envio de centenas
    Serial5.write(sign);                  // Envio de signo de numero
}
///////////////////////////////////////////////////////////////////////////////////

//  SUBRUTINA DE ENVIO DE DATOS A PC: SIN SIGNO SIN DECIMAL CON  TAMAÑO MENOR A 9999 
 void send_data2_toPC(int data_toPC)
 {
   int num;
   byte aux_data;
   
   // Ordenamiento de trama de datos y envio a PC
    data_toPC=abs(data_toPC);
    num=data_toPC;
    aux_data=(num)%100;
    Serial5.write(aux_data);              // Envio de unidad y decenas
    Serial5.write((num-aux_data)/100);    // Envio de centenas
}
///////////////////////////////////////////////////////////////////////////////////

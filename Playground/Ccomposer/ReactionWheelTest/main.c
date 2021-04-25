/***********************
 * IAAPP - UNSA
 * Autor: Brayan Espinoza (Bespi123)
 * Description:
 *
 * IMPORTANT: Change stack size into 2k and compiled into c99 dialect.
 * Modify: 11/03/2021
 ***********************/

//--------------------------Program Libraries-------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sensorlib/quaternion.h"
#include "sensorlib/quaternion.c"
#include "sensorlib/vector.h"
#include "inc/tm4c123gh6pm.h"

//-------------------------Program Definitions------------------------
//  MPU 6050 DEFINITIONS
#define MPU_6050_ADDR           0x68    //MPU 6050 ADDRESS

// MPU 6050 REGISTERS
#define  Reg_PWR_MGMT_1         0x6B    // Power management 1
#define  Reg_INT_PIN_CFG        0x37    // Interrupt pin configuration
#define  Reg_CONFIG             0x1A    // Configuration
#define  Reg_ACC_CONFIG         0x1C    // Accel configuration
#define  Reg_GYRO_CONFIG        0x1B    // Gyro configurations
#define  Reg_INT_PIN_CFG        0x37    // Int pin enable configuration
#define  Reg_INT_ENABLE         0x38    // Enables interrupt generation
#define  Reg_INT_STATUS         0x3A    // Shows interrupt status
#define  Reg_ACC_XOUT_H         0x3B    // AccX data MSB
#define  Reg_ACC_YOUT_H         0x3D    // AccY data MSB
#define  Reg_ACC_ZOUT_H         0x3F    // AccZ data MSB
#define  Reg_TEMP_OUT           0x41    // Temp data MSB
#define  Reg_GYRO_XOUT_H        0x43    // GyroX data MSB
#define  Reg_GYRO_YOUT_H        0x45    // GyroY data MSB
#define  Reg_GYRO_ZOUT_H        0x47    // GyroZ data MSB
#define  Reg_WHO_I_AM           0x75    // Read Address

#define PI  3.141592        // Pi definition
/*
 *------------------PROGRAM FUNCTIONS-------------------
 */
//  I2C Functions
void I2c1_begin(void);
static int I2C_wait_till_done(void);
char I2C1_writeByte(int slaveAddr, char memAddr, char data);
char I2C1_readBytes(int slaveAddr, char memAddr, int byteCount, char* data);

// Interrupt functions
void timerA1Handler(void);
void initDirectionPins (void);

//  UART Functions
void Uart5_begin(void);
void UART5_printString(char *str);
void UART5_Transmitter(unsigned char data);
void UART5_sendBuffer(char *data, uint8_t bufferSize);
void sendEncodedData(char *data, uint8_t dataSize);

//  MPU6050 Functions
bool MPU6050_Init(void);
void MPU6050_getData(int16_t *MPURawData);

// Counter Timer Functions
void countersTimeInit(void);
void countersCaptureInit(void);
int timerA2Capture(void);
int timerA0Capture(void);
int timerB0Capture(void);
void EnableCountersInt(void);
void DisableCountersInt(void);

// Timer interruption
void initTimer1Aint(void);
void Timer2A_Handler(void);

// ADC Initialization
void adcInitialization(void);
void adcTimerTriggerInit(uint32_t period);

//PWM Functions
void enable_PWM(void);

//SysTick function
void SysTickInit(void);

// Controller and attitude functions
void getRwAngularRates(bool *sign, unsigned int *rawRateRPM, float *rwRatesRadPerSecconds);
void GetEulerAngles(float *Acc, float *gyro, float *Euler);
void attitudeControlLaw(float *SetPoint, float *Euler, float *EulerAnt, float *eEulerI, float *ControlTorque);
void torqueCalculation(float *torque, float *rwRates, float *rwRatesAnt);
void torqueController(float *torque, float *ControlTorque, float *rawCommand, float *errorI);
void sendRwCommands(uint16_t *dutyCycle, bool *sign,float *rawCommand);
void sendRwComOpenLoop(uint16_t *dutyCycle, bool *sign);

// Other functions
void Delay(unsigned long counter);

//----------------PROGRAM GLOBAL VARIABLES--------------------
//char m_cMesg[100];              // Buffer to send
char m_cMesg[49];              // Buffer to send
unsigned char m_cMode = 'M';    // Mode variable
//char m_fPrueba[] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','W','X','Y','Z'};   //Prueba
bool m_bSent = false;           // Flag to send data

//----------------------RW Controller Variables---------------------
#define Icoeficient = 15673894481;          // I controller coeficient
#define ReactioWheelInertia 9.4e-7;      // Reaction wheel inertia
#define DT 0.100;                           // Period of sample
float m_itorque[3];         // Current calculated Torque
float m_fTorqueI[3];        // Calculate torque integral
float m_fRawcommand[3];     // Calculate PWM velocity

//---------------CURRENT SENSORS VARIABLES---------------------
//Filter variables (Mobile Average Filter)
const int m_iWindowSize = 20;            // Window size
volatile unsigned int m_ix[20];          // Buffer to store past outputs
volatile unsigned int m_iy[20];          // Buffer to store past outputs
volatile unsigned int m_iz[20];          // Buffer to store past outputs
//Variables to store current sensors
volatile unsigned int m_iRw1Current;     // ADC Current
volatile unsigned int m_iRw2Current;     // ADC Current
volatile unsigned int m_iRw3Current;     // ADC Current

//--------------REACTION WHEELS COMMAND VARIABLES--------------
uint16_t m_iDutyCycle[3];                       // RW PWM duty cycle
bool m_bDirection[] =  {true, true, false};     // RW Direction

//-------------REACTION WHEELS ENCODERS VARIABLES---------------
//Speed measure from encoders
//uint32_t m_uiPeriodX, m_uiPeriodY, m_uiPeriodZ;        // 24-bit, 65.5 ns units
//uint32_t static m_uiFirstX, m_uiFirstY, m_uiFirstZ;    // Timer0A first edge
//volatile unsigned char m_cCounter=0;                   // Counter to control encoder flow
const int m_iRwWindowSize = 20;            // Window size
volatile unsigned int m_iRwX[20];          // Buffer to store past outputs
volatile unsigned int m_iRwY[20];          // Buffer to store past outputs
volatile unsigned int m_iRwZ[20];          // Buffer to store past outputs
float m_uiRwRates[3];             // Filtered reaction wheels
float m_uiRwRatesAnt[3];          // Filtered reaction wheels past sample

//-------------MPU AND KINETICS VARIABLES-----------------------
float m_fq[4];                             // Attitude quaternion
float m_fEulerAngles[3];                   // Euler Angles
float m_fAcc[3], m_fGyro[3];               // MPU6050 data
float m_fEulerAngles_ant[3];               // Euler Angles past samples

//------------ATTITUDE CONTROLLER VARIABLES-----------------------
float m_ftorqueCommand[3];                 // Commands Torques
float m_fsetPoint[] = {0,0,0};            // Set Points in Euler Angles (rad)
float m_feEulerI[3];                       // Euler error integer
//float m_fPrueb[3]={5.2,3.0,5.2};
//--------------------MAIN PROGRAM---------------------
int main(void){
  //initTimer1Aint();                   // Initialize Timer1 interruption
  //adcInitialization();                // Initialize ADC to read currents
  I2c1_begin();                         // Initialize I2C port
  Delay(1000);
  Uart5_begin();                        // Initialize UART port
  Delay(1000);
  MPU6050_Init();                       // Initialize MPU6050
  Delay(1000);
  initDirectionPins();                  // Initialize CW/CCW Pines
  enable_PWM();                         // Enable PWM channels
  countersCaptureInit();                // Initialize timers capture mode for encoders
  adcTimerTriggerInit(5000000);         // Initialize ACD0 in timer trigger mode each 300ms
  SysTickInit();                        // Initialize Systick interrupt
  while(1){
      if(m_bSent){
          // Send Data o PC in strings
          sprintf(m_cMesg, "i%.2f\t %.2f\t%.2f\n",
                  m_fEulerAngles[0], m_fEulerAngles[1], m_fEulerAngles[2]);
          UART5_printString(m_cMesg);
          sprintf(m_cMesg, "j%.2f\t %.2f\t%.2f\n",
                  m_fAcc[0], m_fAcc[1], m_fAcc[2]);
          UART5_printString(m_cMesg);
          sprintf(m_cMesg, "k%.2f\t %.2f\t%.2f\n",
                  m_fGyro[0], m_fGyro[1], m_fGyro[2]);
          UART5_printString(m_cMesg);
          sprintf(m_cMesg, "l%.2f\t %.2f\t%.2f\n",
                  m_uiRwRates[0], m_uiRwRates[1], m_uiRwRates[2]);
          UART5_printString(m_cMesg);
          sprintf(m_cMesg, "m%d\t %d\t%d\n",
                  m_iRw2Current, m_iRw3Current,m_iRw1Current);
          UART5_printString(m_cMesg);

          // Refresh flag
          m_bSent=false;
      }
  }
}

//--------------------------MPU6050 FUNCTIONS----------------------------
/* bool MPU6050_Init(void)
 * Description:
 * This function initialize the MPU6050 and performs its configuration.
 * Return:
 * True if the device is initialize properly and false when device is not
 * finded.
 */
bool MPU6050_Init(void){
    //  Local variables
    char Ack;

    //  Check connection and initialize MPU6050
    I2C1_readBytes(MPU_6050_ADDR, Reg_WHO_I_AM, 1, &Ack);
    if(MPU_6050_ADDR==Ack){
        //Configure MPU6050
        I2C1_writeByte(MPU_6050_ADDR,Reg_PWR_MGMT_1, 0x00);      // Wake-up, 8MHz Oscillator
        I2C1_writeByte(MPU_6050_ADDR,Reg_ACC_CONFIG, 0x00);      // Accel range +/- 2G
        I2C1_writeByte(MPU_6050_ADDR,Reg_GYRO_CONFIG,0x00);      // Gyro range +/- 250 Deg/s
        I2C1_writeByte(MPU_6050_ADDR,Reg_CONFIG, 0x00);          // Set DLPF 256Hz
        //Enable dataReady Interrupt
        I2C1_writeByte(MPU_6050_ADDR,Reg_INT_PIN_CFG,0x70);      // Low active/Open drain/Latch until read/Clear on any read
        I2C1_writeByte(MPU_6050_ADDR,Reg_INT_ENABLE, 0x01);      // Raw_ready_enable int active
        return true;
    }
    return false;
}

/* void MPU6050_getData(int16_t *MPURawData)
 * Description:
 * This function reads MPU6050 Registers in a single burst.
 * Return:
 * Puts MPU6050 data into MPURawData buffer.
 */
void MPU6050_getData(int16_t *MPURawData){
    //Local variables
    char sensordata[14];                  // Char array

    //Read 14 Registers in a single burst
    I2C1_readBytes(MPU_6050_ADDR,Reg_ACC_XOUT_H, 14, sensordata);
    //Turn into 16bits data
    for(char i = 0; i < 7; i++){
        MPURawData[i] = ((int16_t)sensordata[2*i] << 8) | sensordata[2*i+1];
    }
}

//--------------------------UART FUNCTIONS--------------------------
/* void Uart5_begin(void)
 * Description:
 * This function initialize the UART5 Serial Port and starts UART 5 interrupt.
 * Serial 5 is initialized with data length 8-bit, not parity bit, FIFO no enable.
 */
void Uart5_begin(void){
    // Enable clock to UART5 and wait for the clock
    SYSCTL_RCGCUART_R |= 0x20;
    while((SYSCTL_PRUART_R & 0x20)==0);

    // Enable clock to PORTE for PE4/Rx and RE5/Tx and wait
    SYSCTL_RCGCGPIO_R |= 0x10;
    while((SYSCTL_PRGPIO_R & 0x10)==0);

    // UART5 initialization
    UART5_CTL_R &= ~0x01;                               // Disable UART5
    UART5_IBRD_R = (UART5_IBRD_R & ~0XFFFF)+104;        // for 9600 baud rate, integer = 104
    UART5_FBRD_R = (UART5_FBRD_R & ~0X3F)+11;           // for 9600 baud rate, fractional = 11

    UART5_CC_R = 0x00;                                  // Select system clock (based on clock source and divisor factor)
    UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x70;        // Data length 8-bit, not parity bit, FIFO enable
    //UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x60;      // Data length 8-bit, not parity bit, FIFO disable
    UART5_IFLS_R &= ~0x3F;                              // TX FIFO <= 1/2 empty, RX FIFO >= 1/2 full
    UART5_IFLS_R += (UART_IFLS_TX4_8|UART_IFLS_RX4_8);  // and RX time-out
    //UART5_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);

    UART5_CTL_R &= ~0X20;                           // HSE = 0
    UART5_CTL_R |= 0X301;                           // UARTEN = 1, RXE = 1, TXE =1

    // UART5 TX5 and RX5 use PE4 and PE5.
    // Configure them as digital and enable alternate function.
    GPIO_PORTE_DEN_R = 0x30;             // Set PE4 and PE5 as digital.
    GPIO_PORTE_AFSEL_R = 0x30;           // Use PE4,PE5 alternate function.
    GPIO_PORTE_AMSEL_R = 0X00;           // Turn off analog function.
    GPIO_PORTE_PCTL_R = 0x00110000;      // Configure PE4 and PE5 for UART.

    // Enable UART5 interrupt
    UART5_ICR_R &= ~(0x0780);                                          // Clear receive interrupt
    UART5_IM_R  = 0x0010;                                              // Enable UART5 Receive interrupt
    NVIC_PRI15_R = (NVIC_PRI15_R&0x00FFFFFF)|(0x07<<13);               //Timer0A=priority 1
    NVIC_EN1_R  |= (1<<29);                                            // Enable IRQ61 for UART5
}

/* void UART5_Transmitter(unsigned char data)
 * Description:
 * This function sends a char throught UART5.
 */
void UART5_Transmitter(unsigned char data){
    while((UART5_FR_R & (1<<5)) != 0);      // Wait until Tx buffer is not full
    UART5_DR_R = data;                      // Before giving it another byte
}

/* void UART5_printString(char *str)
 * Description:
 * This function sends a char array str througth UART5.
 */
void UART5_printString(char *str){
  while(*str){
        UART5_Transmitter(*(str++));
    }
}

/* void UART5_sendBuffer(char *data, uint8_t bufferSize){
 * Description:
 * Function to send a buffer through UART5
 */
void UART5_sendBuffer(char *data, uint8_t bufferSize){
    for(char i=0; i< bufferSize; i++){
        UART5_DR_R = data[i];
    }
}

/*void sendEncodedData(char *data, uint8_t dataSize)
 * Description:
 * Function to encode and send a char array through UART5
 * the encode algorithm is based in the API2 mode used by
 * the Xbees.
 */
void sendEncodedData(char *data, uint8_t dataSize){
    //Local variables
    uint8_t add=0;
    char message[50];

    //Encode incoming data
    //Check for special cases
    for(uint8_t i = 1; i < dataSize; i++){
        if(data[i] == 'i' || data[i] == 125)
            add++;
    }
    //Create message and copy frame
    uint8_t size = dataSize+add;
    memcpy(message,data,dataSize);
    //Replace special cases
    for(uint8_t j = 1; j < size; j++){
      if (message[j] == 'i' ||  message[j] == 125){
        for(uint8_t k = size-1 ; k > j ; k--)
            message[k+1] = message[k];
        message[j+1] = message[j] ^ 32;
        message[j] = 125;
      }
    }

    //Send data through UART5
    UART5_sendBuffer(message, size);
}

/* void UART5_Handler(void){
 * Description:
 * This function manage the UART5 interruption.
 */
void UART5_Handler(void){
    //Local variables
    unsigned char i = 0;
    char begingChar = 'o';
    char endChar    = 'p';
    char *ret;
    char rx_rawData[16];          // Incoming buffer data
    unsigned char rx_data[10];         //data
    UART5_ICR_R &= ~(0x010);           // Clear receive interrupt
    Delay(1);
    // Read until Rx buffer is empty
    while((UART5_FR_R & (1<<4)) == 0){
        rx_rawData[i] = UART5_DR_R;
         i++;
    }

    ret = memchr(rx_rawData, begingChar, 16);
    memcpy(rx_data,ret+1,10);


     // Get reaction wheel mode
    if((rx_data[0])== 224){
        //Change to manual mode
        m_cMode = 'M';
        //RW1, RW2, RM3 directions
        if(rx_data[3]==16)
            m_bDirection[0] = true;
        else
            m_bDirection[0] = false;
        if(rx_data[6]==16)
            m_bDirection[1] = true;
        else
            m_bDirection[1] = false;
        if(rx_data[9]==16)
            m_bDirection[2] = true;
        else
            m_bDirection[2] = false;

        // RW1, RW2, RW3 duty cycles
        m_iDutyCycle[2] = ((uint16_t)rx_data[7] << 8) | rx_data[8];
        m_iDutyCycle[0] = ((uint16_t)rx_data[1] << 8) | rx_data[2];
        m_iDutyCycle[1] = ((uint16_t)rx_data[4] << 8) | rx_data[5];

    }else if((rx_data[0])== 239){
        // Change to automatic Mode
        m_cMode = 'A';
        // Set setpoint
        m_fEulerAngles[0]=0;
        m_fEulerAngles[1]=0;
        m_fEulerAngles[2]=0;
    }
}

//---------------------I2C Functions------------------------------------
/* void I2c1_begin(void)
 *
 * Description:
 * This function initialize the I2C1 Port in 100kHz.
 *
 */
void I2c1_begin(void){
    SYSCTL_RCGCGPIO_R |= 0x01;              // Enable the clock for port A
    while((SYSCTL_PRGPIO_R & 0x01)==0);     // Wait until clock is initialized
    SYSCTL_RCGCI2C_R   |= 0X02;             // Enable the clock for I2C 1
    while((SYSCTL_PRI2C_R & 0x02)==0);      // Wait until clock is initialized
    GPIO_PORTA_DEN_R |= 0xC0;               // PA6 and PA7 as digital

    // Configure Port A pins 6 and 7 as I2C 1
    GPIO_PORTA_AFSEL_R |= 0xC0;             // Use PA6, PA7 alternate function
    GPIO_PORTA_PCTL_R  |= 0X33000000;       // Configure PA6 and PA7 as I2C
    GPIO_PORTA_ODR_R   |= 0x80;             // SDA (PA7) pin as open Drain
    I2C1_MCR_R = 0x0010;                    // Enable I2C 1 master function

    /* Configure I2C 1 clock frequency
    (1 + TIME_PERIOD ) = SYS_CLK /(2*
    ( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
    TIME_PERIOD = 16 ,000 ,000/(2(6+4) *100000) - 1 = 7 */
    I2C1_MTPR_R = 0x07;
}

/* char I2C1_writeByte( @slaveAddr, @memAddr, @data)
 * int slaveAddr: I2C Slave Address
 * char memAddr:  Register number
 * char data:   Data to send
 *
 * Returns:
 *  0 if all it's Ok and 1 if there is an error.
 *
 * Description:
 * This function write only one byte with the following frame.
 * byte write: S-(saddr+w)-ACK-maddr-ACK-data-ACK-P
 *
 */
char I2C1_writeByte(int slaveAddr, char memAddr, char data){
    char error;
    // Send slave address and starting address
    I2C1_MSA_R = (slaveAddr << 1);      // Assign Master as writing mode
    I2C1_MDR_R = memAddr;               // Assign Starting address
    error = I2C_wait_till_done();       // Wait until write is complete
    if (error) return error;

    I2C1_MCS_R = 0x03;                  // S-(saddr+w)-ACK-maddr-ACK (Start/ Transmit)
    error = I2C_wait_till_done();       // Wait until write is complete
    if (error) return error;

    // Send data to write
    I2C1_MDR_R = data;                   // Assign data to send
    I2C1_MCS_R = 5;                      // -data-ACK-P (Transmit/ Stop)
    error = I2C_wait_till_done();        // Wait until write is complete
    while(I2C1_MCS_R & 0x40);            // Wait until bus is not busy
    error = I2C1_MCS_R & 0xE;            // Check error
    if (error) return error;
    return 0;
}

/* char I2C1_readBytes(@slaveAddr, @memAddr, @byteCount, @* data)
 * int slaveAddr: I2C Slave Address
 * char memAddr:  Register number
 * int byteCount: Bytes number to Read.
 * char* data:   Data to send
 *
 * Returns:
 *
 * Description:
 * This function reads the specified number of bytes in a single burst.
 *
 */
char I2C1_readBytes(int slaveAddr, char memAddr, int byteCount, char* data){
    // Local variables
    char error;

    // Check for incorrect length
    if (byteCount <= 0)
        return 1;

    // Send slave address and starting address
    I2C1_MSA_R = slaveAddr << 1;            // Assign Slave Address as writing mode
    I2C1_MDR_R = memAddr;                   // Send memory Address
    I2C1_MCS_R = 3;                         // S-(saddr+w)-ACK-maddr-ACK (Start/ Run)
    error = I2C_wait_till_done();           // Wait until write is complete
    if (error) return error;                // Check for error

    // Change bus to read mode
    I2C1_MSA_R = (slaveAddr << 1) + 1;      // Assign Slave Address as reading mode
                                            // Restart: -R-(saddr+r)-ACK */
    // If it's the last Byte NonACK
    if (byteCount == 1)
        I2C1_MCS_R = 7;                     // -data-NACK-P (Start/Transmit/Stop)
    else
        I2C1_MCS_R = 0x0B;                  // -data-ACK- (Ack/Start/Transmit)
    error = I2C_wait_till_done();           // Wait until write is complete
    if (error) return error;                // Check for error

    // Store the received data
    *data++ = I2C1_MDR_R;                   // Coming data from Slave

    // Single byte read
    if (--byteCount == 0){
        while(I2C1_MCS_R & 0x40);           // Wait until bus is not busy
        return 0;                           // No error
    }

    // Read the rest of the bytes
    while (byteCount > 1){
        I2C1_MCS_R = 9;                     // -data-ACK-
        error = I2C_wait_till_done();
        if (error) return error;
        byteCount--;
        *data++ = I2C1_MDR_R;               // store data received
    }

    I2C1_MCS_R = 5;                         // -data-NACK-P
    error = I2C_wait_till_done();
    *data = I2C1_MDR_R;                     // Store data received
    while(I2C1_MCS_R & 0x40);               // Wait until bus is not busy

    return 0;                               // No error
}

/* static int I2C_wait_till_done(void)
 * Returns:
 *  If there is no error, return 0.
 *
 * Description:
 * Wait until I2C master is not busy and return error code
 */
static int I2C_wait_till_done(void){
    while(I2C1_MCS_R & (1<<0) != 0); // Wait until I2C master is not busy
    return I2C1_MCS_R & 0xE;         // Check for errors
}

/* void Delay(unsigned long counter)
 * Description:
 * Single delay code.
 */
void Delay(unsigned long counter){
    unsigned long i = 0;
    for(i=0; i< counter*10000; i++);
}

//---------------------------PWM Funtions-------------------------------
/* void enable_PWM(void)
 * Description:
 * This function initialize the PWM ports for every reaction wheel
 * the pines are distributed in the following array:
 * PF0(M1PWM5) ---> RW1 (Z axis)
 * PF2(M1PWM6) ---> RW2 (X axis)
 * PF3(M1PWM7) ---> RW3 (Y axis)
 */
void enable_PWM(void){
    // Clock setting for PWM and GPIO PORT
    SYSCTL_RCGCPWM_R |= (1<<1);            // Enable clock to PWM1 module
    while((SYSCTL_PRPWM_R & (1<<1))==0);   // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<5);           // Enable system clock to PORTF
    while((SYSCTL_PRGPIO_R & (1<<5))==0);  // Wait until clock is initialized

    SYSCTL_RCC_R |= (1<<20);               // Enable System Clock Divisor function
    SYSCTL_RCC_R |= 0x000E0000;            // Use pre-divider value of 64

    // PF0 has special function, need to unlock to modify
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;     // Unlock commit register
    GPIO_PORTF_CR_R = 0X1F;                // Make PORTF0 configurable
    GPIO_PORTF_LOCK_R  = 0;                // Lock commit register

    // Setting of PF0, PF2 and PF3 as M1PWM5, M1PWM6, M1PWM7 channels
    GPIO_PORTF_AFSEL_R |= (1<<0)|(1<<2)|(1<<3);     // Sets an alternate function
    GPIO_PORTF_PCTL_R &= ~0x0000FF0F;               // Set PF0, PF2 and PF3 as output pin
    GPIO_PORTF_PCTL_R |= 0x00005505;                // Make PF0, PF2 and PF3 PWM output pin
    GPIO_PORTF_DEN_R |= (1<<0)|(1<<2)|(1<<3);       // set PF0, PF2 and PF3 digital pines

    // Enable PWM Generator 3 A and B
    PWM1_3_CTL_R &= ~(1<<0);            // Disable Generator 3 counter
    PWM1_3_CTL_R &= ~(1<<1);            // Select down count mode of counter 3
    PWM1_3_GENA_R = 0x0000008C;         // Set PWM output when counter reloaded and clear when matches PWMCMPA
    PWM1_3_GENB_R = 0x0000080C;         // Set PWM output when counter reloaded and clear when matches PWMCMPB
    PWM1_3_LOAD_R = 5000;               // Set load value for 50Hz (16MHz/65 = 250kHz/5000 = 50Hz).
    PWM1_3_CMPA_R = 4999;               // Set duty cycle to to minimum value
    PWM1_3_CMPB_R = 200;                // Set duty cycle to to minimum value
    PWM1_3_CTL_R = 1;                   // Enable Generator 3 counter

    // Enable PWM Generator2 A
    PWM1_2_CTL_R &= ~(1<<0);            // Disable Generator 2 counter
    PWM1_2_CTL_R &= ~(1<<1);            // Select down count mode of counter 2
    PWM1_2_GENA_R = 0x0000008C;         // Set PWM output when counter reloaded and clear when matches PWMCMPA
    PWM1_2_LOAD_R = 5000;               // Set load value for 50Hz (16MHz/65 = 250kHz/5000 = 50Hz).
    PWM1_2_CMPA_R = 4999;               // Set duty cycle to to minimum value
    PWM1_2_CTL_R = 1;                   // Enable Generator 3 counter

    // Enable PWM channels
    PWM1_ENABLE_R = (1<<4)|(1<<6)|(1<<7);  // Enable PWM1 channels 4,6,7 output
}

//----------------------- Direction pins functions--------------------------------
/* void initDirectionPins(void)
 * Description:
 * This function initialize the CCW/CW ports for every reaction wheel
 * the pines are distributed in the following array:
 * PA4 ---> RW1 (Z axis)
 * PA5 ---> RW2 (X axis)
 * PA3 ---> RW3 (Y axis)
 */
void initDirectionPins(void){
    // Clock setting for PortA
    SYSCTL_RCGCGPIO_R |= (1<<0);            // Enable clock
    while((SYSCTL_PRGPIO_R & (1<<0))==0);   // Wait until clock is enable

    //Initialize PA4, PA5, PA6  as a digital outputs
    GPIO_PORTA_DIR_R |= (1<<4)|(1<<5)|(1<<3);    // Set PA4, PA5, PA3 as outputs
    GPIO_PORTA_DEN_R |= (1<<4)|(1<<5)|(1<<3);    // make PORTA3-5 digital pins
}


//------------------------ Timer based interruptions functions ----------------------
void initTimer1Aint(void){
    //Enable timer1
    SYSCTL_RCGCTIMER_R |= (1<<1);  // Enable clock Timer1 subtimer A in run mode
    TIMER1_CTL_R = 0;              // Disable timer1 output
    TIMER1_CFG_R = 0x4;            // Select 16-bit configuration option
    TIMER1_TAMR_R = 0x02;          // Select periodic down counter mode of timer1
    //TIMER1_TAPR_R = 250-1;         // TimerA prescaler value 250 (64KHz)
    //TIMER1_TAILR_R = 32-1 ;     // TimerA counter starting count down value from 1ms
    //TIMER1_TAILR_R = 16000-1 ;     // TimerA counter starting count down value from 200ms
    TIMER1_TAILR_R = 64000-1 ;     // TimerA counter starting count down value from 1s
    TIMER1_ICR_R = 0x1;            // TimerA timeout flag bit clears

    // Enable timer 1 interrupt
    TIMER1_IMR_R |=(1<<0);         // Enables TimerA time-out  interrupt mask
    TIMER1_CTL_R |= 0x01;          // Enable TimerA module
    NVIC_PRI4_R |= (NVIC_PRI4_R&0x00FFFFFF)|(0x04<<13);      //Timer1A=priority 4
    NVIC_EN0_R |= (1<<21);         // Enable IRQ21
}

void timerA1Handler(void){
    if(TIMER1_MIS_R & 0x01){
/*
        // REACTION WHEELS RATES - MOBILE AVERAGE FILTER
        // Update past samples buffer
        for(int i = m_iRwWindowSize-2; i >= 0; i--){
            // Shift data
            m_iRwX[i+1]=m_iRwX[i];
            m_iRwY[i+1]=m_iRwY[i];
            m_iRwZ[i+1]=m_iRwZ[i];
        }

            // Get new sample and restart m_iRwnCurrent variables
            m_iRwX[0]=m_uiPeriodX;
            m_iRwY[0]=m_uiPeriodY;
            m_iRwZ[0]=m_uiPeriodZ;
            m_uiRwRateX = 0;
            m_uiRwRateY = 0;
            m_uiRwRateZ = 0;

            // Perform the mobile average
            for(int i = 0; i < m_iRwWindowSize; i++){
                m_uiRwRateX += m_iRwX[i]/m_iRwWindowSize;
                m_uiRwRateY += m_iRwY[i]/m_iRwWindowSize;
                m_uiRwRateZ += m_iRwZ[i]/m_iRwWindowSize;
            }

        sprintf(m_cMesg, "%d, %d, %d \n", m_uiPeriodX, m_uiPeriodY, m_uiPeriodZ);
        UART5_printString(m_cMesg);
*/
    }
    TIMER1_ICR_R = 0X01;                // TA1 Timeout flag
}

//-------------------------------Timers used for RPM Measurements functions-----------------------------
/* void countersTimeInit(void)
 * Description:
 * This function initialize timers as counter timer time mode to read reaction
 * wheels rates.
 * the pines and timers are distributed in the following array:
 * PF4 (TIMER2A) ---> RW1 (Z axis)
 * PB6 (TIMER0A) ---> RW2 (X axis)
 * PB7 (TIMER0B) ---> RW3 (Y axis)
 */
void countersTimeInit(void){
/*
    //Enable timer2A and Port F
    SYSCTL_RCGCTIMER_R |= (1<<2);               // Enable clock to Timer 2
    while((SYSCTL_PRTIMER_R & (1<<2))==0);      // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<4);                // Enable clock to PORTF
    while((SYSCTL_PRGPIO_R & (1<<4))==0);       // Wait until clock is initialized

    //Enable PF4
    GPIO_PORTF_DIR_R &= ~(1<<4);                // Make PF4 an input pin
    GPIO_PORTF_DEN_R |= (1<<4);                 // Make PF4 a digital pin
    GPIO_PORTF_AFSEL_R |= (1<<4);               // Enable alternate function on PF4
    GPIO_PORTF_PCTL_R &= ~0x000F0000;           // Configure PF4 as T2CCP0 pin
    GPIO_PORTF_PCTL_R |= 0x000070000;           // Configure PF4 as T2CCP0 pin

    //Set timer as input-edge time mode
    TIMER2_CTL_R &= ~(1<<0);                    // Disable TIMER2A in setup
    TIMER2_CFG_R |= (1<<2);                     // Configure as 16-bit timer mode
    TIMER2_TAMR_R = 0x07;                       // Count down, edge time, capture mode, TIMER2A
    TIMER2_CTL_R &= ~((1<<3)|(1<<2));           // Capture rising edge
    TIMER2_TAILR_R = 0x0000FFFF;                // Start value
    TIMER2_TAPR_R = 0xFF;                       // Activate Pre-scaler, creating 24-bit
    TIMER2_IMR_R |= (1<<2);                     // Enable capture match interrupt
    TIMER2_ICR_R = (1<<2);                      // Clear timer2A capture match flag
    TIMER2_CTL_R |= 0x00000001;                 // Enable Timer2A 24-b, +edge, interrupts

    //Enable interrupt
    //NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x40000000;      // Timer2A=priority 2
    //NVIC_EN0_R |= 1<<23;                                    // Enable interrupt 23 in NVIC

    //Enable Timer0 and portB
    SYSCTL_RCGCTIMER_R |= (1<<0);               // Enable clock to Timer 0
    while((SYSCTL_PRTIMER_R & (1<<0))==0);      // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<1);                // Enable clock to PORTB
    while((SYSCTL_PRGPIO_R & (1<<1))==0);       // Wait until clock is initialized

    //Enable PB6 - PB7
    GPIO_PORTB_DIR_R &= ~((1<<6) | (1<<7));     // Make PB6, PB7 an input pin
    GPIO_PORTB_DEN_R |= (1<<6) | (1<<7);        // Make PB6, PB7 a digital pin
    GPIO_PORTB_AFSEL_R |= (1<<6) | (1<<7);      // Enable alternate function on PB6, PB7
    GPIO_PORTB_PCTL_R &= ~0xFF000000;           // Configure PB6 as T0CCP0
    GPIO_PORTB_PCTL_R |= 0x77000000;

    //Set timer as input-edge counter mode
    TIMER0_CTL_R &= ~((1<<0)|(1<<8));           // Disable TIMER0A and TIMER0B in setup
    TIMER0_CFG_R |= (1<<2);                     // Configure as 16-bit timer mode
    TIMER0_TAMR_R = 0x07;                       // Count up, edge-time, capture mode TIMER0A
    TIMER0_TBMR_R = 0x07;                       // Count up, edge-time, capture mode TIMER0B
    TIMER0_CTL_R &= ~((1<<3)|(1<<2));           // Capture the rising edge TIMER0A
    TIMER0_CTL_R &= ~((1<<11)|(1<<10));         // Capture the rising edge TIMER0B
    TIMER0_TAILR_R = 0x0000FFFF;                // Start value TAMR0A
    TIMER0_TBILR_R = 0x0000FFFF;                // Start value TAMR0B
    TIMER0_TAPR_R = 0xFF;                       // Activate Pre-scaler, creating 24-bit TIMER0A
    TIMER0_TBPR_R = 0xFF;                       // Activate Pre-scaler, creating 24-bit TIMER0B
    TIMER0_IMR_R |= ((1<<2)|(1<<10));           // Enable capture match interrupt
    TIMER0_ICR_R = ((1<<2)|(1<<10));            // Clear TIMER0A and TIMER0B capture match flag
    //TIMER0_CTL_R |= ((1<<0)|(1<<8));            // Enable Timer2A 24-b, +edge, interrupts
    TIMER0_CTL_R |= (1<<8);            // Enable Timer2A 24-b, +edge, interrupts

    //Enable priority
    NVIC_PRI4_R |= (NVIC_PRI4_R&0x00FFFFFF)|(0x01<<29);               //Timer0A=priority 1
    NVIC_PRI5_R |= (NVIC_PRI5_R&0x00FFFFFF)|(0x02<<5)|(0x03<<29);     //Timer0B=priority 2 and Timer2A=priority 3
*/
}

void Timer2A_Handler(void){
/*
    //----Local variables----
    volatile unsigned int PeriodZ;   // Period with 62.5ns resolution

    // Acknowledge timer2A
    TIMER2_ICR_R |= (1<<2);

    //----Calculate amount of time----
    PeriodZ = ((m_uiFirstZ - TIMER2_TAR_R)&0x00FFFFFF);  // Period in 62.5ns resolution
    m_uiFirstZ = TIMER2_TAR_R;                           // Update time

    //----Update Buffer----
    // Update past samples buffer
    for(int i = m_iRwWindowSize-2; i >= 0; i--){
        // Shift data
        m_iRwZ[i+1]=m_iRwZ[i];
    }
    // Get new sample and restart m_iRwnCurrent variables
    m_iRwZ[0]=PeriodZ;

    //----Update Counter----
    m_cCounter++;

    //----Get average----
    if(m_cCounter == (m_iRwWindowSize-1)){

        m_uiRwRateX = 0;
        // Perform the mobile average
        for(int i = 0; i < m_iRwWindowSize; i++){
            m_uiRwRateX += (53333333/m_iRwX[i])/m_iRwWindowSize;
        }
        m_uiPeriodX = 53333333*m_uiRwRateX;
    */
/*
        // Reset counter
        m_cCounter = 0;

        //----Disable and Reset timer 0A----
        TIMER2_CTL_R &= ~(1<<0);
        TIMER2_TAV_R = 0X00;
        TIMER2_TAR_R = 0X00;

        //----Enable next thread----
        NVIC_DIS0_R |= (1<<23);       // Disable GPT2A interrupts
    }
*/
}

void Timer0A_Handler(void){
/*
    //----Local variables----
    volatile unsigned int PeriodX;   // Period with 62.5ns resolution

    // Acknowledge timer0A capture
    TIMER0_ICR_R |= (1<<2);

    //----Calculate amount of time----
    PeriodX = ((m_uiFirstX - TIMER0_TAR_R)&0x00FFFFFF);  // Period in 62.5ns resolution
    m_uiFirstX = TIMER0_TAR_R;                           // Update time

    //----Update Buffer----
    // Update past samples buffer
    for(int i = m_iRwWindowSize-2; i >= 0; i--){
        // Shift data
        m_iRwX[i+1]=m_iRwX[i];
    }
    // Get new sample and restart m_iRwnCurrent variables
    m_iRwX[0]=PeriodX;

    //----Update Counter----
    m_cCounter++;

    //----Get average----
    if(m_cCounter == (m_iRwWindowSize-1)){

        m_uiRwRateX = 0;
        // Perform the mobile average
        for(int i = 0; i < m_iRwWindowSize; i++){
            m_uiRwRateX += (53333333/m_iRwX[i])/m_iRwWindowSize;
        }
        m_uiPeriodX = 53333333*m_uiRwRateX;
        */
/*
        // Reset counter
        m_cCounter = 0;

        //----Disable and Reset timer 0A----
        TIMER0_CTL_R &= ~(1<<0);
        TIMER0_TAV_R = 0X00;
        TIMER0_TAR_R = 0X00;

        //----Enable next thread----
        NVIC_DIS0_R |= (1<<19);       // Disable GPT0A interrupts
        NVIC_EN0_R |= (1<<20);        // Enable GPT0B interrupts
        TIMER0_CTL_R |= (1<<8);       // Enable timer0B
    }
*/
}

void Timer0B_Handler(void){
/*
    //----Local variables----
    volatile unsigned int PeriodY;   // Period with 62.5ns resolution

    // Acknowledge timer0B capture
    TIMER0_ICR_R |= (1<<10);

    //----Calculate amount of time----
    PeriodY = ((m_uiFirstY - TIMER0_TBR_R)&0x00FFFFFF);  // Period in 62.5ns resolution
    m_uiFirstY = TIMER0_TBR_R;                           // Update time

    //----Update Buffer----
    // Update past samples buffer
    for(int i = m_iRwWindowSize-2; i >= 0; i--){
        // Shift data
        m_iRwY[i+1]=m_iRwY[i];
    }
    // Get new sample and restart m_iRwnCurrent variables
    m_iRwY[0]=PeriodY;

    //----Update Counter----
    m_cCounter++;

    //----Get average----
    if(m_cCounter == (m_iRwWindowSize-1)){

        m_uiRwRateX = 0;
        // Perform the mobile average
        for(int i = 0; i < m_iRwWindowSize; i++){
            m_uiRwRateX += (53333333/m_iRwX[i])/m_iRwWindowSize;
        }
        m_uiPeriodX = 53333333*m_uiRwRateX;
        */
/*
        // Reset counter
        m_cCounter = 0;

        //----Disable and Reset timer 0B----
        TIMER0_CTL_R &= ~(1<<8);
        TIMER0_TBV_R = 0X00;
        TIMER0_TBR_R = 0X00;

        //----Enable next thread----
        NVIC_DIS0_R |= (1<<20);       // Disable GPT0B interrupts
        NVIC_EN0_R |= (1<<23);        // Enable GPT2A interrupts
        TIMER2_CTL_R |= (1<<8);       // Enable GPT2A
    }
*/
}

/* void countersCaptureInit(void)
 * Description:
 * This function initialize timers as counter timer mode to read reaction
 * wheels rates.
 * the pines and timers are distributed in the following array:
 * PF4 (TIMER2A) ---> RW1 (Z axis)
 * PB6 (TIMER0A) ---> RW2 (X axis)
 * PB7 (TIMER0B) ---> RW3 (Y axis)
 */
void countersCaptureInit(void){
    // Enable timer2A and portF
    SYSCTL_RCGCTIMER_R |= (1<<2);           // Enable clock to Timer 2
    while((SYSCTL_PRTIMER_R & (1<<2))==0);  // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<4);            // Enable clock to PORTF
    while((SYSCTL_PRGPIO_R & (1<<4))==0);   // Wait until clock is initialized

    //Enable PF4
    GPIO_PORTF_DIR_R &= ~(1<<4);       // Make PF4 an input pin
    GPIO_PORTF_DEN_R |= (1<<4);        // Make PF4 a digital pin
    GPIO_PORTF_AFSEL_R |= (1<<4);      // Enable alternate function on PF4
    GPIO_PORTF_PCTL_R &= ~0x000F0000;  // Configure PF4 as T2CCP0 pin
    GPIO_PORTF_PCTL_R |= 0x000070000;

    //Set timer as input-edge counter mode
    TIMER2_CTL_R &= ~(1<<0);        // Disable TIMER2A in setup
    TIMER2_CFG_R |= (1<<2);         // Configure as 16-bit timer mode
    TIMER2_TAMR_R = 0x13;           // Up-count, edge-count, capture mode
    TIMER2_TAMATCHR_R = 0xFFFF;     // Set the count limit
    TIMER2_TAPMR_R = 0xFF;          // To 0xFFFFFF with prescaler
    TIMER2_CTL_R &= ~(1<<3);        // Capture the rising edge
    TIMER2_CTL_R &= ~(1<<2);        // Capture the rising edge

    //Enable timer0 and portB
    SYSCTL_RCGCTIMER_R |= (1<<0);           // Enable clock to Timer 0
    while((SYSCTL_PRTIMER_R & (1<<0))==0);  // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<1);            // Enable clock to PORTB
    while((SYSCTL_PRGPIO_R & (1<<1))==0);   // Wait until clock is initialized

    //Enable PB6 - PB7
    GPIO_PORTB_DIR_R &= ~((1<<6) | (1<<7));   // Make PB6, PB7 an input pin
    GPIO_PORTB_DEN_R |= (1<<6) | (1<<7);      // Make PB6, PB7 a digital pin
    GPIO_PORTB_AFSEL_R |= (1<<6) | (1<<7);    // Enable alternate function on PB6, PB7
    GPIO_PORTB_PCTL_R &= ~0xFF000000;         // Configure PB6 as T0CCP0
    GPIO_PORTB_PCTL_R |= 0x77000000;

    //Set timer as input-edge counter mode
    TIMER0_CTL_R &= ~(1<<0);        // Disable TIMER0A in setup0
    TIMER0_CTL_R &= ~(1<<8);        // Disable TIMER0B in setup
    TIMER0_CFG_R |= (1<<2);         // Configure as 16-bit timer mode
    TIMER0_TAMR_R = 0x13;           // Up-count, edge-count, capture mode TIMER A
    TIMER0_TBMR_R = 0x13;           // Up-count, edge-count, capture mode TIMER B
    TIMER0_TAMATCHR_R = 0xFFFF;     // Set the count limit TIMER A
    TIMER0_TBMATCHR_R = 0xFFFF;     // Set the count limit TIMER B
    TIMER0_TAPMR_R = 0xFF;          // To 0xFFFFFF with prescaler TIMER A
    TIMER0_TBPMR_R = 0xFF;          // To 0xFFFFFF with prescaler TIMER B
    TIMER0_CTL_R &= ~((1<<3)|(1<<2));          // Capture the rising edge TIMER A
    TIMER0_CTL_R &= ~((1<<11)|(1<<10));        // Capture the rising edge TIMER B
    TIMER0_CTL_R |= (1<<0);         // Enable Timer0A
    TIMER0_CTL_R |= (1<<8);         // Enable Timer0B
    TIMER2_CTL_R |= 0x01;           // Enable Timer2A
}

int timerA2Capture(void){
    return TIMER2_TAR_R;
}

int timerA0Capture(void){
    return TIMER0_TAR_R;
}

int timerB0Capture(void){
    return TIMER0_TBR_R;
}

void EnableCountersInt(void){
    //Enable interrupts
    NVIC_EN0_R |= ((1<<19)|(1<<20));                                 // Enable interrupt 19 and 20 in NVIC
    NVIC_EN0_R |= 1<<23;                                             // Enable interrupt 23 in NVIC
}

void DisableCountersInt(void){
    //Disable interrupts
    NVIC_DIS0_R |= ((1<<19)|(1<<20));                                 // Disable interrupt 19 and 20 in NVIC
    NVIC_DIS0_R |= 1<<23;                                             // Disable interrupt 23 in NVIC
}

//----------------------------ADC Functions-----------------------------------
/* void adcInitialization(void)
 * Description:
 * This function initialize adc in sequence 2 to read current consumption of
 * wheels rates.
 * the pines and timers are distributed in the following array:
 * PD3 (AIN4) ---> RW1 (Z axis)
 * PE1 (AIN2) ---> RW2 (X axis)
 * PE2 (AIN1) ---> RW3 (Y axis)
 */
void adcInitialization(void){
    // Enable Clock to ADC0 and GPIO pins
    SYSCTL_RCGCGPIO_R |= (1<<3);             // Enable Clock to GPIOD or PD3
    while((SYSCTL_PRGPIO_R & (1<<3))==0);    // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<4);             // Enable Clock to GPIOE for PE1, PE2
    while((SYSCTL_PRGPIO_R & (1<<4))==0);    // Wait until clock is initialized
    SYSCTL_RCGCADC_R  |= (1<<0);             // Module ADC0 clock enable
    while((SYSCTL_PRADC_R & (1<<0))==0);     // Wait until clock is initialized

    // Initialize PD3 for AIN4 input
    GPIO_PORTD_DIR_R &= ~(1<<3);            // Make PD3 input
    GPIO_PORTD_AFSEL_R |= (1<<3);           // Enable alternate function
    GPIO_PORTD_DEN_R &= ~(1<<3);            // Disable digital function
    GPIO_PORTD_AMSEL_R |= (1<<3);           // Enable analog function

    // Initialize PE1, PE2 for AIN1, AIN2 input
    GPIO_PORTE_DIR_R &= ~((1<<1)|(1<<2));   // Make PE1, PE2 input
    GPIO_PORTE_AFSEL_R |= (1<<1) | (1<<2);  // Enable alternate function
    GPIO_PORTE_DEN_R &= ~((1<<1)|(1<<2));   // Disable digital function
    GPIO_PORTE_AMSEL_R |= (1<<1) | (1<<2);  // Enable analog function

    // Initialize sample sequencer2
    //ADC0_PC_R = (1<<0);                   // Configure for 125k samples/sec
    ADC0_SSPRI_R = 0X3210;                  // Seq0 is highest, seq3 lowest priority
    ADC0_ACTSS_R &= ~(1<<2);                // Disable SS2 during configuration
    ADC0_EMUX_R &= ~0x0F00;                 // Software trigger conversion
    ADC0_SSMUX2_R = 0X000214;               // Get input from channel 1,2, and 4
    ADC0_SSCTL2_R |= (1<<9)|(1<<10);        // Take three samples at a time, set flag at 3rd sample

    // Enable ADC Interrupt
    ADC0_IM_R |= (1<<2);                    // Unmask sequence 2 interrupt
    NVIC_EN0_R |= (1<<16);                  // Enable IRQ17 for ADC0SS2
    ADC0_ACTSS_R |= (1<<2);                 // Enable ADC0 sequencer 2
    ADC0_PSSI_R |= (1<<2);                  // Enable SS2 conversion or start sampling data from AN0
}

//----------------------------ADC Functions-----------------------------------
/* void adcTimerTriggerInit(uint32_t period)
 * Description:
 * This function initialize adc in sequence 2 to read current consumption of
 * wheels rates.
 * the pines and timers are distributed in the following array:
 * PD3 (AIN4) ---> RW1 (Z axis)
 * PE1 (AIN2) ---> RW2 (X axis)
 * PE2 (AIN1) ---> RW3 (Y axis)
 */
void adcTimerTriggerInit(uint32_t period){
    // Enable ADC0 and GPIO pins
    SYSCTL_RCGCGPIO_R |= (1<<3);             // Enable Clock to GPIOD or PD3
    while((SYSCTL_PRGPIO_R & (1<<3))==0);    // Wait until clock is initialized
    SYSCTL_RCGCGPIO_R |= (1<<4);             // Enable Clock to GPIOE for PE1, PE2
    while((SYSCTL_PRGPIO_R & (1<<4))==0);    // Wait until clock is initialized
    SYSCTL_RCGCADC_R  |= (1<<0);             // Module ADC0 clock enable
    while((SYSCTL_PRADC_R & (1<<0))==0);     // Wait until clock is initialized

    // Initialize PD3 for AIN4 input
    GPIO_PORTD_DIR_R &= ~(1<<3);            // Make PD3 input
    GPIO_PORTD_AFSEL_R |= (1<<3);           // Enable alternate function
    GPIO_PORTD_DEN_R &= ~(1<<3);            // Disable digital function
    GPIO_PORTD_AMSEL_R |= (1<<3);           // Enable analog function

    // Initialize PE1, PE2 for AIN1, AIN2 input
    GPIO_PORTE_DIR_R &= ~((1<<1)|(1<<2));   // Make PE1, PE2 input
    GPIO_PORTE_AFSEL_R |= (1<<1) | (1<<2);  // Enable alternate function
    GPIO_PORTE_DEN_R &= ~((1<<1)|(1<<2));   // Disable digital function
    GPIO_PORTE_AMSEL_R |= (1<<1) | (1<<2);  // Enable analog function

    // Set ADC0
    ADC0_PC_R = 0x01;                       // Configure for 125K samples/sec
    ADC0_SSPRI_R = 0x3210;                  // Seq 0 is highest, Seq 3 is lowest

    // Enable Timer3
    SYSCTL_RCGCTIMER_R |= (1<<3);           // Enable timer3
    while((SYSCTL_PRTIMER_R & (1<<3))==0);  // Wait until clock is initialized
    // Timer3 configurations
    TIMER3_CTL_R &= ~(1<<0);                // Disable timer3A during setup
    TIMER3_CTL_R |= (1<<5);                 // Enable timer3A trigger to ADC
    TIMER3_CFG_R = 0;                       // Configure for 32-bit timer mode
    TIMER3_TAMR_R = 0x00000002;             // Configure for periodic mode
    TIMER3_TAPR_R = 0;                      // Prescale value for trigger
    TIMER3_TAILR_R = period-1;              // Start value for trigger
    TIMER3_IMR_R = 0x00000000;              // Disable all interrupts
    TIMER3_CTL_R |= (1<<0);                 // Enable timer3A 32-b, periodic

    // Initialize sample sequencer2
    ADC0_ACTSS_R &= ~(1<<2);                        // Disable sample sequencer 2
    ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFFF0FF)+0x0500;  // Enable timer trigger
    ADC0_SSMUX2_R = 0X000214;                       // Get input from channel 1,2, and 4
    ADC0_SSCTL2_R |= (1<<9)|(1<<10);                // Take three samples at a time, set flag at 3rd sample

    // Enable ADC Interrupt
    ADC0_IM_R |= (1<<2);            // Enable SS2 interrupts
    NVIC_EN0_R |= (1<<16);          // Enable IRQ17 for ADC0SS2
    ADC0_ACTSS_R |= (1<<2);         // Enable ADC0 sequencer 2
    //NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFF00FF)|0x00004000; // 11)priority 2
}

void ADC0SS2_Handler(void){
    //Local variables
    uint32_t xRaw, yRaw, zRaw;      //Variables to store raw data
    xRaw=ADC0_SSFIFO2_R&0xFFF;
    yRaw=ADC0_SSFIFO2_R&0xFFF;
    zRaw=ADC0_SSFIFO2_R&0xFFF;

    //Reaction wheels mobile average filtering
    // Update past samples buffer
    for(int i = m_iWindowSize-2; i >= 0; i--){
        // Shift data
        m_ix[i+1]=m_ix[i];
        m_iy[i+1]=m_iy[i];
        m_iz[i+1]=m_iz[i];
    }
    // Get new sample and restart m_iRwnCurrent variables
    m_ix[0]=(xRaw*3300)/ 4095;
    m_iy[0]=(yRaw*3300)/ 4095;
    m_iz[0]=(zRaw*3300)/ 4095;
    m_iRw1Current = 0;
    m_iRw2Current = 0;
    m_iRw3Current = 0;

    //Perform the mobile average
    for(int i = 0; i < m_iWindowSize; i++){
        m_iRw2Current += m_ix[i]/m_iWindowSize;
        m_iRw3Current += m_iy[i]/m_iWindowSize;
        m_iRw1Current += m_iz[i]/m_iWindowSize;
    }

    //Clear flag bit and Enable SS2 conversion or start sampling data
    ADC0_ISC_R = (1<<2);
    ADC0_PSSI_R |= (1<<2);
}

//-------------------------------SysTick functions--------------------------------
void SysTickInit(void){
    //NVIC_ST_RELOAD_R = 15999999;   // One second delay 1s relaod value
    NVIC_ST_RELOAD_R = 1599999;   // One second delay 0.1s relaod value
    //NVIC_ST_RELOAD_R = 19999;   // One second delay 1ms relaod value
    NVIC_ST_CTRL_R = 0x07;      // Enable counter, interrupt and select system bus clock
    NVIC_ST_CURRENT_R = 0;      // Initialize current value register
}

void SysTick_Handler(void){
    // Local variables
    unsigned int RwRate[3];

    // Get measured reaction wheels rates
    RwRate[0]=timerA0Capture()*33.333;
    RwRate[1]=timerB0Capture()*33.333;
    RwRate[2]=timerA2Capture()*33.333;

    // Reset counters
    TIMER0_CTL_R &= ~((1<<0)|(1<<8));   // Disable GPTM0A & GPTM0B
    TIMER2_CTL_R &= ~(1<<0);            // Disable GPTM0A & GPTM0B
    TIMER0_TAV_R = 0x00;
    TIMER0_TBV_R = 0x00;
    TIMER2_TAV_R = 0x00;
    TIMER0_TAR_R = 0x00;
    TIMER0_TBR_R = 0x00;
    TIMER2_TAR_R = 0x00;
/*
    //----Update Buffers----
    // Update past samples buffer
    for(int i = m_iRwWindowSize-2; i >= 0; i--){
        // Shift data
        m_iRwX[i+1]=m_iRwX[i];
        m_iRwY[i+1]=m_iRwY[i];
        m_iRwZ[i+1]=m_iRwZ[i];
    }
    // Get new sample in RPM
    m_iRwX[0]=33.33*RwRateX;
    m_iRwY[0]=33.33*RwRateY;
    m_iRwZ[0]=33.33*RwRateZ;
*/
    // Get Reaction Wheels in rad/s
    getRwAngularRates(m_bDirection, RwRate, m_uiRwRates);

    // GET MPU6050 DATA
   GetEulerAngles(m_fAcc, m_fGyro, m_fEulerAngles);

   if(m_cMode == 'A'){
       // Perform attitude controller and torque controller
       attitudeControlLaw(m_fsetPoint, m_fEulerAngles, m_fEulerAngles_ant, m_feEulerI, m_ftorqueCommand);
       torqueCalculation(m_itorque, m_uiRwRates, m_uiRwRatesAnt);
       torqueController(m_itorque, m_ftorqueCommand, m_fRawcommand, m_fTorqueI);
       // Send calculated PWM to Brushless Motors
       sendRwCommands(m_iDutyCycle, m_bDirection, m_fRawcommand);
   }else if (m_cMode == 'M'){
       sendRwComOpenLoop(m_iDutyCycle, m_bDirection);
   }

   // Enable timers and enable interruption flag
   TIMER0_CTL_R |= (1<<0)|(1<<8);   // Disable GPTM0A & GPTM0B
   TIMER2_CTL_R |= 1<<0;            // Disable GPTM0A & GPTM0B
   m_bSent = true;                  // Interruption flag
}

void GetEulerAngles(float *Acc, float *gyro, float *Euler){
   // Local Variables
   int16_t MpuRawData[7];             // MPU6050 raw data

   // GET MPU6050 DATA
   MPU6050_getData(MpuRawData);
   // Convert Readings
   Acc[0]= (float)MpuRawData[0]/16384.0;    // X Accel GEES
   Acc[1]= (float)MpuRawData[1]/16384.0;    // Y Accel GEES
   Acc[2]= (float)MpuRawData[2]/16384.0;    // Z Accel GEES
   gyro[0]= (float)MpuRawData[4]/131.1;     // X Gyro deg/s
   gyro[1]= (float)MpuRawData[5]/131.1;     // Y Gyro deg/s
   gyro[2]= (float)MpuRawData[6]/131.1;     // Z Gyro deg/s
   //t = ((float)MPURawData[3]/340.00)+36.53;

   //Get Euler angles (rad)
   Euler[0]= atan(Acc[1] / sqrt(pow(Acc[0], 2) + pow(Acc[2], 2)));
   Euler[1]= atan(Acc[0] / sqrt(pow(Acc[1], 2) + pow(Acc[2], 2)));
   Euler[2] += gyro[2] *(PI/180) *DT;
}

void attitudeControlLaw(float *SetPoint, float *Euler, float *EulerAnt, float *eEulerI, float *ControlTorque){
    // Local variables
    volatile float eRoll,ePitch,eYaw;
    //volatile float m_feRollI,m_fePitchI,m_feYawI;
    volatile float eRollD,ePitchD,eYawD;

    // Euler angle error
    eRoll  = SetPoint[0] - Euler[0];
    ePitch = SetPoint[1] - Euler[1];
    eYaw   = SetPoint[2] - Euler[2];

    // Integrate Error
    eEulerI[0] += eRoll*DT;
    eEulerI[1] += ePitch*DT;
    eEulerI[2] += eYaw*DT;

    // Error rate
    eRollD  = (Euler[0]-EulerAnt[0])/DT;
    ePitchD = (Euler[1]-EulerAnt[1])/DT;
    eYawD   = (Euler[2]-EulerAnt[2])/DT;

    // Perform PID controller
    ControlTorque[0]=-3.24987577526797e-07*eRoll -2.1955547593358e-10*eEulerI[0]-8.46467148788469e-05*eRollD;
    ControlTorque[1]=-3.24987577526797e-07*ePitch-2.1955547593358e-10*eEulerI[1]-8.46467148788469e-05*ePitchD;
    ControlTorque[2]=-3.24987577526797e-07*eYaw  -2.1955547593358e-10*eEulerI[2]-8.46467148788469e-05*eYawD;

    // Update new variables
    EulerAnt[0]=Euler[0];
    EulerAnt[1]=Euler[1];
    EulerAnt[2]=Euler[2];
}

void torqueCalculation(float *torque, float *rwRates, float *rwRatesAnt){
    // Local Variables
    volatile float rwAccX,rwAccY,rwAccZ;

    // Get acceleration
    rwAccX = (rwRates[0]-rwRatesAnt[0])/DT;
    rwAccY = (rwRates[1]-rwRatesAnt[1])/DT;
    rwAccZ = (rwRates[2]-rwRatesAnt[2])/DT;

    // Get current torque
    torque[0] = rwAccX*ReactioWheelInertia;
    torque[1] = rwAccY*ReactioWheelInertia;
    torque[2] = rwAccZ*ReactioWheelInertia;

    // Update variables
    rwRatesAnt[0] = rwRates[0];
    rwRatesAnt[1] = rwRates[1];
    rwRatesAnt[2] = rwRates[2];
}

void torqueController(float *torque, float *ControlTorque, float *rawCommand, float *errorI){
    // Local Variables
    float errorX, errorY, errorZ;

    //Get Torque error
    errorX = ControlTorque[0]-torque[0];
    errorY = ControlTorque[1]-torque[1];
    errorZ = ControlTorque[2]-torque[2];

    //Integrate torque error
    errorI[0] += errorX*DT;
    errorI[1] += errorY*DT;
    errorI[2] += errorZ*DT;

    //Calculate speed command
    rawCommand[0] = errorI[0] * 15673894481;
    rawCommand[1] = errorI[1] * 15673894481;
    rawCommand[2] = errorI[2] * 15673894481;
}

void sendRwCommands(uint16_t *dutyCycle, bool *sign,float *rawCommand){
    // Send commands
    dutyCycle[0] = abs(rawCommand[0]);
    dutyCycle[1] = abs(rawCommand[1]);
    dutyCycle[2] = abs(rawCommand[2]);

    //Check signs of torques
    if (rawCommand[0]>0)
        sign[0] = true;
    else
        sign[0] = false;
    if (rawCommand[1]>0)
        sign[1] = true;
    else
        sign[1] = false;
    if (rawCommand[2]>0)
        sign[2] = false;
    else
        sign[2] = true;

    // Saturate outputs
   if(dutyCycle[0]>4999)
       dutyCycle[0] = 4999;
   if(dutyCycle[1]>4999)
       dutyCycle[1] = 4999;
   if(dutyCycle[2]>4999)
       dutyCycle[2] = 4999;

   // Send Speed Command
   PWM1_3_CMPA_R = dutyCycle[0];
   PWM1_3_CMPB_R = dutyCycle[1];
   PWM1_2_CMPA_R = dutyCycle[2];

   // Send Direction pin
   if (sign[0])
       GPIO_PORTA_DATA_R |= (1<<5);
   else
       GPIO_PORTA_DATA_R &= ~(1<<5);
   if (sign[1])
       GPIO_PORTA_DATA_R |= (1<<3);
   else
       GPIO_PORTA_DATA_R &= ~(1<<3);
   if (sign[2])
       GPIO_PORTA_DATA_R |= (1<<4);
   else
       GPIO_PORTA_DATA_R &= ~(1<<4);
}

void sendRwComOpenLoop(uint16_t *dutyCycle, bool *sign){
    // Saturate outputs
    if(dutyCycle[0]>4999)
       dutyCycle[0] = 4999;
    if(dutyCycle[1]>4999)
       dutyCycle[1] = 4999;
    if(dutyCycle[2]>4999)
       dutyCycle[2] = 4999;

    // Send Speed Command
    PWM1_3_CMPA_R = dutyCycle[0];
    PWM1_3_CMPB_R = dutyCycle[1];
    PWM1_2_CMPA_R = dutyCycle[2];

    // Send Direction pin
    if (sign[0])
        GPIO_PORTA_DATA_R |= (1<<5);
    else
        GPIO_PORTA_DATA_R &= ~(1<<5);
    if (sign[1])
        GPIO_PORTA_DATA_R |= (1<<3);
    else
        GPIO_PORTA_DATA_R &= ~(1<<3);
    if (sign[2])
        GPIO_PORTA_DATA_R |= (1<<4);
    else
        GPIO_PORTA_DATA_R &= ~(1<<4);
}

void getRwAngularRates(bool *sign, unsigned int *rawRateRPM, float *rwRatesRadPerSecconds){
    // Turn RPM into rad/seconds
    rwRatesRadPerSecconds[0]=rawRateRPM[0]*2*PI/60;
    rwRatesRadPerSecconds[1]=rawRateRPM[1]*2*PI/60;
    rwRatesRadPerSecconds[2]=rawRateRPM[2]*2*PI/60;

    // Get sign
    if (sign[0])
        rwRatesRadPerSecconds[0] = rwRatesRadPerSecconds[0];
    else
        rwRatesRadPerSecconds[0] = -1*rwRatesRadPerSecconds[0];
    if (sign[1])
        rwRatesRadPerSecconds[1] = rwRatesRadPerSecconds[1];
    else
        rwRatesRadPerSecconds[1] = -1*rwRatesRadPerSecconds[1];
    if (sign[2])
        rwRatesRadPerSecconds[2] = -1*rwRatesRadPerSecconds[2];
    else
        rwRatesRadPerSecconds[2] = rwRatesRadPerSecconds[2];
}

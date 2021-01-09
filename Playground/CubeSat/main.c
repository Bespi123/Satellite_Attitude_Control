/***********************
 * IAAPP - UNSA
 * Autor: Brayan Espinoza (Bespi123)
 * Description:
 * This program send MPU6050 conected to I2C1 througth Serial 5 port.
 * IMPORTANT: Change stack size into 2k and compiled into c99 dialect.
 ***********************/

//--------------------------Program Lybraries-------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
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

int32_t Count1,Count2 = 0;

//------------------PROGRAM FUNCTIONS-------------------
//  I2C Functions
void I2c1_begin(void);
char I2C1_writeByte(int slaveAddr, char memAddr, char data);
char I2C1_readBytes(int slaveAddr, char memAddr, int byteCount, char* data);
static int I2C_wait_till_done(void);

// Interrupt functions
void GPIO(void);
void timerA1Handler(void);
void initDirectionPins (void);
void PortF1_IntEnable(void);

//  UART Functions
void Uart5_begin(void);
void UART5_Transmitter(unsigned char data);
void UART5_printString(char *str);

//  MPU6050 Functions
bool MPU6050_Init(void);
void MPU6050_getData(int16_t *MPURawData);
void Delay(unsigned long counter);
void enable_PWM(void);

// Timer Functions
void initTimer1Aint(void);
void timerA2Init(void);
int timerA2Capture(void);

// ADC Initialization
void adc0Initialization(void);

//----------------PROGRAM GLOBAL VARIABLES--------------
int16_t MPURawData[7];
char m_sMsg[20];
volatile unsigned int counter = 0;
float AX, AY, AZ, t, GX, GY, GZ;      // MPU6050 Data
volatile unsigned int adc_value;
volatile unsigned int adc_value1;
char mesg[100];
int duty_cycle =  4999;

volatile float voltage;
volatile bool m_bInterrupt = false;
//Filter variables (Moving Average Filter)
const int windowSize = 20;
volatile unsigned int x[20];
volatile unsigned long int y = 0;

//---------------------MAIN PROGRAM---------------------
int main(void){
  //    Local variables
 // int duty_cycle =  4999;
  //    Initialize I2C1 and UART5
  I2c1_begin();
  Delay(1000);
  Uart5_begin();
  MPU6050_Init();
  Delay(1000);
  GPIO();
  initDirectionPins();
  enable_PWM();
  timerA2Init();
  initTimer1Aint();
  adc0Initialization();
  //PortF1_IntEnable();

  while(1){
      /*
      duty_cycle = duty_cycle - 10;
      if (duty_cycle <= 0)
         duty_cycle = 5000;
      PWM1_3_CMPA_R = duty_cycle;
*/
     // voltage = (adc_value * 0.0008);
     // sprintf(mesg, "%d /n", adc_value);
     // UART5_printString(mesg);

      //sprintf(mesg, "RPM: %d \n", counter);
/*
      if(m_bInterrupt){
          sprintf(mesg, "%d, %d, %d \n", counter, adc_value-2400, adc_value1-2400);
          //sprintf(mesg, "%d,%d, %d \n", counter, adc_value1-2400);
          UART5_printString(mesg);
          m_bInterrupt = false;
      }
*/
      Delay(10);
/*
      // Read MPU6050 in a burst of data
      MPU6050_getData(MPURawData);

      // Convert The Readings
      AX = (float)MPURawData[0]/16384.0;
      AY = (float)MPURawData[1]/16384.0;
      AZ = (float)MPURawData[2]/16384.0;
      GX = (float)MPURawData[4]/131.0;
      GY = (float)MPURawData[5]/131.0;
      GZ = (float)MPURawData[6]/131.0;
      t = ((float)MPURawData[3]/340.00)+36.53;

      //Turn into string and send trough Serial5
      sprintf(m_sMsg,"%.2f,%.2f,%.2f\n",AX,AY,AZ);
      UART5_printString(m_sMsg);
*/
    }
}

bool MPU6050_Init(void){
    //  Local variables
    char Ack;
    //  Check connection
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

void PortF1_IntEnable(void){
    //Enable GPIOF clock
    SYSCTL_RCGCGPIO_R |= (1<<5);
    while((SYSCTL_PRGPIO_R & (1<<5))==0);

    //Initialize PORTF1
    GPIO_PORTF_DIR_R &= ~(1<<1);        // Set as digital input
    GPIO_PORTF_DEN_R |= (1<<1);         // Set as digital pin
    GPIO_PORTF_PUR_R |= (1<<1);         // Enable PORTF0 pull up resistor

    //Configure PORTF1 for falling edge trigger interrupt
    GPIO_PORTF_IS_R &=~(1<<1);          // make bit 1 edge sensitive
    GPIO_PORTF_IBE_R &=~(1<<1);         // trigger controlled by IEV
    GPIO_PORTF_IEV_R &=~(1<<1);         // falling edge trigger
    GPIO_PORTF_ICR_R |= (1<<1);         // clear any prior interrupt
    GPIO_PORTF_IM_R |= (1<<1);          // unmask interrupt

    // enable interrupt in NVIC and set priority to 3 */
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF)|0x00A00000;  //Priority 5
    NVIC_EN0_R |= (1<<30);  // enable IRQ30 (D30 of ISER[0])
}

/* void Uart5_begin(void)
 * Description:
 * This function initialize the UART5 Serial Port in 9600 bps
 * as data length 8-bit, not parity bit, FIFO enable.
 *
 */
void Uart5_begin(void){
    // Enable clock to UART5 and wait for the clock
    SYSCTL_RCGCUART_R |= 0x20;
    while((SYSCTL_PRUART_R & 0x20)==0);

    // Enable clock to PORTE for PE4/Rx and RE5/Tx and wait
    SYSCTL_RCGCGPIO_R |= 0x10;
    while((SYSCTL_PRGPIO_R & 0x10)==0);

    // UART5 initialization
    UART5_CTL_R &= ~0x01;                           // Disable UART5
    UART5_IBRD_R = (UART5_IBRD_R & ~0XFFFF)+104;    // for 9600 baud rate, integer = 104
    UART5_FBRD_R = (UART5_FBRD_R & ~0X3F)+11;       // for 9600 baud rate, fractional = 11

    UART5_CC_R = 0x00;                              // Select system clock (based on clock source and divisor factor)*/
    //UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x70;    // Data length 8-bit, not parity bit, FIFO enable
    UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x60;    // Data length 8-bit, not parity bit, FIFO disable
    UART5_CTL_R &= ~0X20;                           // HSE = 0
    UART5_CTL_R |= 0X301;                           // UARTEN = 1, RXE = 1, TXE =1

    // UART5 TX5 and RX5 use PE4 and PE5.
    // Configure them as digital and enable alternate function.
    GPIO_PORTE_DEN_R = 0x30;             // Set PE4 and PE5 as digital.
    GPIO_PORTE_AFSEL_R = 0x30;           // Use PE4,PE5 alternate function.
    GPIO_PORTE_AMSEL_R = 0X00;           // Turn off analog function.
    GPIO_PORTE_PCTL_R = 0x00110000;      // Configure PE4 and PE5 for UART.

    // Enable UART5 interrupt
    UART5_ICR_R &= ~(0x0780);            // Clear receive interrupt
    UART5_IM_R  = 0x0010;                // Enable UART5 Receive interrupt
    NVIC_EN1_R  |= (1<<29);               // Enable IRQ61 for UART5
    //NVIC->ISER[1] |= 0x20000000;
}

void UART5_Transmitter(unsigned char data){
    while((UART5_FR_R & (1<<5)) != 0);      // Wait until Tx buffer is not full
    UART5_DR_R = data;                      // Before giving it another byte
}

void UART5_printString(char *str){
  while(*str){
        UART5_Transmitter(*(str++));
    }
}

void UART5_Handler(void){
    //Local variables
    unsigned char rx_data = 0;
    UART5_ICR_R &= ~(0x010);            // Clear receive interrupt
    rx_data = UART5_DR_R ;              // Get the received data byte
    if(rx_data == 'A'){
        GPIO_PORTF_DATA_R |= (1<<3);
        GPIO_PORTA_DATA_R |= (1<<4);
    }else if(rx_data == 'B'){
        GPIO_PORTF_DATA_R &= ~(1<<3);
        GPIO_PORTA_DATA_R &= ~(1<<4);
    }
    UART5_Transmitter(rx_data); // send data that is received
}

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

void Delay(unsigned long counter){
    unsigned long i = 0;
    for(i=0; i< counter*10000; i++);
}

void GPIO(void){
    SYSCTL_RCGCGPIO_R |= (1<<5); //Set bit5 to enable

    //PORTF0 has special function, need to unlock to modify
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;   //Unlock commit register
    GPIO_PORTF_CR_R = 0X1F;              //Make PORTF0 configurable
    //GPIO_PORTF_LOCK_R = 0;               //Lock commit register

    //Initialize PF3 as a digital output, PF0 and PF4 as digital input pins
    GPIO_PORTF_DIR_R  &= ~(1<<4)|~(1<<0);               //Set PF4 and PF0 as a digital input pins
    GPIO_PORTF_DIR_R |= (1<<3);                         // Set PF3 as digital output to control green LED
    GPIO_PORTF_DEN_R |= (1<<4)|(1<<3)|(1<<0);           // make PORTF4-0 digital pins
    GPIO_PORTF_PUR_R |= (1<<4)|(1<<0);                  // enable pull up for PORTF4, 0

    // configure PORTF4, 0 for falling edge trigger interrupt
    GPIO_PORTF_IS_R  &= ~(1<<4)|~(1<<0);        // make bit 4, 0 edge sensitive
    GPIO_PORTF_IBE_R &=~(1<<4)|~(1<<0);         // trigger is controlled by IEV
    GPIO_PORTF_IEV_R &= ~(1<<4)|~(1<<0);        // falling edge trigger
    GPIO_PORTF_ICR_R |= (1<<4)|(1<<0);          // clear any prior interrupt
    GPIO_PORTF_IM_R  |= (1<<4)|(1<<0);          // unmask interrupt

    // enable interrupt in NVIC and set priority to 5
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF)|0x00A00000;  //Priority 5
    NVIC_EN0_R |= (1<<30);  // enable IRQ30 (D30 of ISER[0])
}

void GPIOPortF_Handler(void){

    if(GPIO_PORTF_MIS_R & 0x10){        //check if interrupt causes by PF4/SW1
        GPIO_PORTF_DATA_R |= (1<<3);
        GPIO_PORTF_ICR_R |= 0x10;       //clear the interrupt flag
    }
    else if (GPIO_PORTF_MIS_R & 0x01){  //check if interrupt causes by PF0/SW2
        GPIO_PORTF_DATA_R &= ~0X08;
        GPIO_PORTF_ICR_R |= 0x01;       //clear the interrupt flag
    }
    else if (GPIO_PORTF_MIS_R & (1<<1)){ //check if interrupt causes by PF0/SW2
        //UART5_printString("A");
        /*
        GPIO_PORTF_ICR_R |= (1<<1);       //clear the interrupt flag
        // Read MPU6050 in a burst of data
        MPU6050_getData(MPURawData);

        // Convert The Readings
        AX = (float)MPURawData[0]/16384.0;
        AY = (float)MPURawData[1]/16384.0;
        AZ = (float)MPURawData[2]/16384.0;
        GX = (float)MPURawData[4]/131.0;
        GY = (float)MPURawData[5]/131.0;
        GZ = (float)MPURawData[6]/131.0;
        t = ((float)MPURawData[3]/340.00)+36.53;

        //Turn into string and send trough Serial5
        sprintf(m_sMsg,"%.2f,%.2f,%.2f\n",AX,AY,AZ);
        UART5_printString(m_sMsg);
        */
    }
}

void enable_PWM(void){
    // Clock setting for PWM and GPIO PORT
    SYSCTL_RCGCPWM_R |= (1<<1);         // Enable clock to PWM1 module
    SYSCTL_RCGCGPIO_R |= (1<<5);        // Enable system clock to PORTF
    SYSCTL_RCC_R |= (1<<20);            // Enable System Clock Divisor function
    SYSCTL_RCC_R |= (7<<17);            // Use pre-divider value of 64 and after that feed clock to PWM1 module

    // Setting of PF2 pin for M1PWM6 channel output pin
    GPIO_PORTF_AFSEL_R |= (1<<2);       // PF2 sets a alternate function
    GPIO_PORTF_PCTL_R &= ~0x00000F00;   // Set PF2 as output pin
    GPIO_PORTF_PCTL_R |= 0x00000500;    // Make PF2 PWM output pin
    GPIO_PORTF_DEN_R |= (1<<2);         // set PF2 as a digital pin

    PWM1_3_CTL_R &= ~(1<<0);            // Disable Generator 3 counter
    PWM1_3_CTL_R &= ~(1<<1);            // Select down count mode of counter 3
    PWM1_3_GENA_R = 0x0000008C;         // Set PWM output when counter reloaded and clear when matches PWMCMPA
    PWM1_3_LOAD_R = 5000;               // Set load value for 50Hz 16MHz/65 = 250kHz and (250KHz/5000)
    PWM1_3_CMPA_R = 4999;               // Set duty cyle to to minumum value
    PWM1_3_CTL_R = 1;                   // Enable Generator 3 counter
    PWM1_ENABLE_R = 0x40;               // Enable PWM1 channel 6 output
}

void initDirectionPins(void){
    // Clock setting for PortA
    SYSCTL_RCGCGPIO_R |= (1<<0);            // Enable clock
    while((SYSCTL_PRGPIO_R & (1<<0))==0);   // Wait until clock is enable

    //Initialize PA2, PA3, PA4  as a digital outputs
    GPIO_PORTA_DIR_R |= (1<<4)|(1<<3)|(1<<2);    // Set PA2, PA3, PA4 as outputs
    GPIO_PORTA_DEN_R |= (1<<4)|(1<<3)|(1<<2);    // make PORTF4-0 digital pins
}

void initTimer1Aint(void){

    //Enable timer1
    SYSCTL_RCGCTIMER_R |= (1<<1);  // Enable clock Timer1 subtimer A in run mode
    TIMER1_CTL_R = 0;              // Disable timer1 output
    TIMER1_CFG_R = 0x4;            // Select 16-bit configuration option
    TIMER1_TAMR_R = 0x02;          // Select periodic down counter mode of timer1
    TIMER1_TAPR_R = 250-1;         // TimerA prescaler value 250
    TIMER1_TAILR_R = 12800-1 ;     // TimerA counter starting count down value from 200ms
    TIMER1_ICR_R = 0x1;            // TimerA timeout flag bit clears

    // Enable timer 1 interrupt
    TIMER1_IMR_R |=(1<<0);         // Enables TimerA time-out  interrupt mask
    TIMER1_CTL_R |= 0x01;          // Enable TimerA module
    NVIC_EN0_R |= (1<<21);         // Enable IRQ21
}

void timerA1Handler(void){
    if(TIMER1_MIS_R & 0x01){
        GPIO_PORTF_DATA_R ^= (1<<3);    // Toggle blue led

        counter = timerA2Capture();
        TIMER2_CTL_R &= ~1;             // Disable TIMER2A
        TIMER2_TAV_R = 0;
        TIMER2_TAR_R = 0;

        duty_cycle = duty_cycle - 10;
        if (duty_cycle <= 0)
           duty_cycle = 5000;
        PWM1_3_CMPA_R = duty_cycle;

        sprintf(mesg, "%d, %d, %d \n", counter, adc_value/10, adc_value1/10);
        UART5_printString(mesg);

        m_bInterrupt = true;
        TIMER2_CTL_R |= 1;             // Enable TIMER3A
    }

    TIMER1_ICR_R = 0X01;                // TA1 Timeout flag
}


void timerA2Init(void){
    //Enable timer3A and portC
    SYSCTL_RCGCTIMER_R |= (1<<2);  // Enable clock to Timer 2
    SYSCTL_RCGCGPIO_R |= (1<<1);   // Enable clock to PORTB

    //Enable PB0
    GPIO_PORTB_DIR_R &= ~(1<<0);   // Make PB0 an input pin
    GPIO_PORTB_DEN_R |= (1<<0);    // Make PB0 a digital pin
    GPIO_PORTB_AFSEL_R |= (1<<0);  // Enable alternate function on PB0
    GPIO_PORTB_PCTL_R &= ~0x0000000F;  // Configure PB0 as T2CCP0 pin
    GPIO_PORTB_PCTL_R |= 0x000000007;

    //Set timer as input-edge counter mode
    TIMER2_CTL_R &= ~(1<<0);        // Disable TIMER2A in setup
    TIMER2_CFG_R |= (1<<2);         // Configure as 16-bit timer mode
    TIMER2_TAMR_R = 0x13;           // Up-count, edge-count, capture mode
    TIMER2_TAMATCHR_R = 0xFFFF;     // Set the count limit
    TIMER2_TAPMR_R = 0xFF;          // To 0xFFFFFF with prescaler
    TIMER2_CTL_R |= ~(1<<3)|~(1<<2);    // Capture the rising edge
    TIMER2_CTL_R |= 0x01;               // Enable Timer3A
}

int timerA2Capture(void){
    return TIMER2_TAR_R;
}

void adc0Initialization(void){
    // Enable Clock to ADC4 and GPIO pins
    SYSCTL_RCGCGPIO_R |= (1<<3);   // Enable Clock to GPIOD or PD3/AN4
    SYSCTL_RCGCADC_R  |= (1<<0);   // Module ADC0 clock enable

    // Initialize PD3 for \AIN4 input
    GPIO_PORTD_DIR_R &= ~(1<<3);   // Make PD3 input
    GPIO_PORTD_AFSEL_R |= (1<<3);  // Enable alternate function
    GPIO_PORTD_DEN_R &= ~(1<<3);   // Disable digital function
    GPIO_PORTD_AMSEL_R |= (1<<3);  // Enable analog function

    // Initialize sample sequencer3
    //ADC0_PC_R = (1<<0);           // Configure for 125k samples/sec
    ADC0_SSPRI_R = 0X3210;        // Seq0 is highest, seq3 lowest priority
    ADC0_ACTSS_R &= ~(1<<3);      // Disable SS3 during configuration
    ADC0_EMUX_R &= ~0xF000;       // Software trigger conversion
    //ADC0_SSMUX3_R = 4;          // Get input from channel 4
    ADC0_SSMUX3_R = 4;            // Get input from channel 4
    ADC0_SSCTL3_R |= (1<<1)|(1<<2);   // Take one sample at a time, set flag at 1st sample

    // Enable ADC Interrupt
    ADC0_IM_R |= (1<<3);            // Unmask ADC4 sequence 3 interrupt
    NVIC_EN0_R |= (1<<17);           // Enable IRQ17 for ADC0SS3
    ADC0_ACTSS_R |= (1<<3);         // Enable ADC0 sequencer 3
    ADC0_PSSI_R |= (1<<3);          // Enable SS3 conversion or start sampling data from AN0
}

void ADC0SS3_Handler(void){
    adc_value = (ADC0_SSFIFO3_R*3300)/ 4095;     // Read ADC coversion result from SS3 FIFO
    y=0;
    // Perform mobile average filter
    for(int i = windowSize-2; i >= 0; i--){          // Shift data
        x[i+1]=x[i];
    }
    x[0]=(ADC0_SSFIFO3_R*3300)/ 4095;               // Get new data

    adc_value1 = 0;

    for(int i = 0; i < windowSize; i++){
        adc_value1 += x[i]/windowSize;
    };

    ADC0_ISC_R = 8;                 // clear coversion clear flag bit
    //sprintf(mesg, "%d\n", adc_value1-2400);
    //UART5_printString(mesg);

    ADC0_PSSI_R |= (1<<3);          // Enable SS3 conversion or start sampling data from AN0
}

 /*******************************************************************
 * IAAPP - UNSA
 * Autor: Brayan Espinoza (Bespi123)
 * Description:
 * This program send MPU6050 conected to I2C1 througth Serial 5 port.
 * IMPORTANT: Change stack size into 2k and compiled into c99 dialect.
 *******************************************************************/

//--------------------------Program Lybraries-------------------------
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//-------------------------Program Definitions------------------------
//  MPU 6050 DEFINITIONS
#define MPU_6050_ADDR           0x68    //MPU 6050 ADDRESS

// MPU 6050 REGISTERS
#define  Reg_PWR_MGMT_1         0x6B    // Power management 1
#define  Reg_INT_PIN_CFG        0x37    // Interrupt pin configuration
#define  Reg_CONFIG             0x1A    // Configuration
#define  Reg_ACC_CONFIG         0x1C    // Accel configuration
#define  Reg_GYRO_CONFIG        0x1B    // Gyro configurations
#define  Reg_ACC_XOUT_H         0x3B    // AccX data MSB
#define  Reg_ACC_YOUT_H         0x3D    // AccY data MSB
#define  Reg_ACC_ZOUT_H         0x3F    // AccZ data MSB
#define  Reg_TEMP_OUT           0x41    // Temp data MSB
#define  Reg_GYRO_XOUT_H        0x43    // GyroX data MSB
#define  Reg_GYRO_YOUT_H        0x45    // GyroY data MSB
#define  Reg_GYRO_ZOUT_H        0x47    // GyroZ data MSB
#define  Reg_WHO_I_AM           0x75    // Read Address

//
#define XG_OFFS_TC          0x00
#define YG_OFFS_TC          0x01
#define ZG_OFFS_TC          0x02
#define X_FINE_GAIN         0x03
#define Y_FINE_GAIN         0x04
#define Z_FINE_GAIN         0x05
#define XA_OFFS_H           0x06
#define XA_OFFS_L_TC        0x07
#define YA_OFFS_H           0x08
#define YA_OFFS_L_TC        0x09
#define ZA_OFFS_H           0x0A
#define ZA_OFFS_L_TC        0x0B
#define XG_OFFS_USRH        0x13
#define XG_OFFS_USRL        0x14
#define YG_OFFS_USRH        0x15
#define YG_OFFS_USRL        0x16
#define ZG_OFFS_USRH        0x17
#define ZG_OFFS_USRL        0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define FF_THR              0x1D
#define FF_DUR              0x1E
#define MOT_THR             0x1F
#define MOT_DUR             0x20
#define ZRMOT_THR           0x21
#define ZRMOT_DUR           0x22
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define BANK_SEL            0x6D
#define MEM_START_ADDR      0x6E
#define MEM_R_W             0x6F
#define DMP_CFG_1           0x70
#define DMP_CFG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

//------------------PROGRAM FUNCTIONS-------------------
//  I2C Functions
void I2c1_begin(void);
char I2C1_writeByte(int slaveAddr, char memAddr, char data);
char I2C1_readBytes(int slaveAddr, char memAddr, int byteCount, char* data);
static int I2C_wait_till_done(void);

//  UART Functions
void Uart5_begin(void);
void UART5_Transmitter(unsigned char data);
void UART5_printString(char *str);

//  MPU6050 Functions
bool MPU6050_Init(void);
void MPU6050_getData(int16_t *MPURawData);
void Delay(unsigned long counter);

//----------------PROGRAM GLOBAL VARIABLES--------------


//---------------------MAIN PROGRAM---------------------
int main(void){
  //    Local variables
  int16_t MPURawData[7];
  char m_sMsg[20];
  float AX, AY, AZ, t, GX, GY, GZ;      // MPU6050 Data

  //    Initialize I2C1 and UART5
  I2c1_begin();
  Delay(1000);
  Uart5_begin();
  MPU6050_Init();
  Delay(1000);

  while(1){
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

      /*
      sprintf(m_sMsg,"Gx = %.2f \t",GX);
      UART5_printString(m_sMsg);
      sprintf(m_sMsg,"Gy = %.2f \t",GY);
      UART5_printString(m_sMsg);
      sprintf(m_sMsg,"Gz  = %.2f \t",GZ);
      UART5_printString(m_sMsg);
      sprintf(m_sMsg,"Ax  = %.2f \t",AX);
      UART5_printString(m_sMsg);
      sprintf(m_sMsg,"Ay  = %.2f \t",AY);
      UART5_printString(m_sMsg);
      sprintf(m_sMsg,"Az  = %.2f \t",AZ);
      UART5_printString(m_sMsg);
      sprintf(m_sMsg,"Temp  = %.2f \r\n",t);
      UART5_printString(m_sMsg);
      Delay(1000);
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

        //I2C1_writeByte(MPU_6050_ADDR,INT_ENABLE, 0x01);
        return true;
    }
    return false;
}

void MPU6050_getData(int16_t *MPURawData){
    //Local variables
    char sensordata[14];                  // Char array

    //Read 14 Registers in a single burst
    I2C1_readBytes(MPU_6050_ADDR,ACCEL_XOUT_H, 14, sensordata);
    //Turn into 16bits data
    for(char i = 0; i < 7; i++){
        MPURawData[i] = ((int16_t)sensordata[2*i] << 8) | sensordata[2*i+1];
    }
}

/* void Uart5_begin(void)
 *
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
    UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x70;    // Data length 8-bit, not parity bit, FIFO enable
    UART5_CTL_R &= ~0X20;                           // HSE = 0
    UART5_CTL_R |= 0X301;                           // UARTEN = 1, RXE = 1, TXE =1

    // UART5 TX5 and RX5 use PE4 and PE5.
    // Configure them as digital and enable alternate function.
    GPIO_PORTE_DEN_R = 0x30;             // Set PE4 and PE5 as digital.
    GPIO_PORTE_AFSEL_R = 0x30;           // Use PE4,PE5 alternate function.
    GPIO_PORTE_AMSEL_R = 0X00;           // Turn off analog function.
    GPIO_PORTE_PCTL_R = 0x00110000;      // Configure PE4 and PE5 for UART.
}

void UART5_Transmitter(unsigned char data){
    while((UART5_FR_R & (1<<5)) != 0);  /* wait until Tx buffer not full */
    UART5_DR_R = data;                  /* before giving it another byte */
}

void UART5_printString(char *str){
  while(*str){
        UART5_Transmitter(*(str++));
    }
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

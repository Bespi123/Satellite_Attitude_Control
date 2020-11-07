//*****************************************************************************
//Blink the green LED
//*****************************************************************************
/**
 *
 * main.c
 */
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

int main(void)
{
    volatile uint32_t ui32Loop;

    //Enable GPIO port that is used for the on-board LED
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
    //Do a dummy read to insert a few cycles after enabling the peripheral.
    ui32Loop=SYSCTL_RCGC2_R;
    //Enable the GPIO pin for the LED (PF3). Set the direction as output, and
    //enable the GPIO pin for digital function
    GPIO_PORTF_DIR_R = 0X08;
    GPIO_PORTF_DEN_R = 0X08;
    //Loop forever
    while(1)
    {
        //Turn on the Green Led on PF3.
        GPIO_PORTF_DATA_R |= 0x08;
        //Delay a bit
        for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++){}
        //Turn off the LED
        GPIO_PORTF_DATA_R &= ~(0X08);
        //Delay for a bit
        for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++){}
    }
	//return 0;
}

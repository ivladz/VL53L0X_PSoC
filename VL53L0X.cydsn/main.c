/* ========================================
PSoC4 ToF Sensor VL53L0X
By Vlad Radoiu @  https://vladradoiu.wordpress.com/
*/

#include "project.h"
#include <stdio.h>

#include <VL53L0X.h>

//Use in timed(continuous) or single mode
#define CONTINUOUS


uint16 dist = 0;

// Timer1ms
CY_ISR( timer_isr )
{
   	TIMER_STATUS;     // Clear timer status
    timeout_start_ms++;
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    TIMER_Init();    
    isr_StartEx( timer_isr );
    TIMER_Start();  
    
    I2C_Start();
    UART_Start();
    UART_PutString("Hello\n");
    
    VL53L0X_init(1); //initialize in 2.8V mode, default 500ms timeout
    
    #ifdef CONTINUOUS
    VL53L0X_startContinuous(100); // timed continuous mode, if 0 scan as fast as posible
    #endif
    
    CyDelay(50); // wait for the device to start
    
    for(;;)
    {
        #ifdef CONTINUOUS
        dist = VL53L0X_readRangeContinuousMillimeters();
        #else
        dist = VL53L0X_readRangeSingleMillimeters();
        #endif
        
        // serial print distance
        char buf[20];
        sprintf(buf, "Distance = %d\n", dist);  
        UART_PutString(buf);
        CyDelay(250);          
    }
}

/* [] END OF FILE */

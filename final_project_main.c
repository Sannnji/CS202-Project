//*****************************************************************************
//*****************************    C Source Code    ***************************
//*****************************************************************************
//  DESIGNER NAME:  Nathan Buonemani and James Ji
//
//       LAB NAME:  TBD
//
//      FILE NAME:  main.c
//
//-----------------------------------------------------------------------------
//
// DESCRIPTION:
//    This program serves as a ... 
//
//*****************************************************************************
//*****************************************************************************

//-----------------------------------------------------------------------------
// Loads standard C include files
//-----------------------------------------------------------------------------
#include <stdio.h>

//-----------------------------------------------------------------------------
// Loads MSP launchpad board support macros and definitions
//-----------------------------------------------------------------------------
#include <ti/devices/msp/msp.h>
#include "clock.h"
#include "LaunchPad.h"

#include "lcd1602.h"
#include "uart.h"

#include "adc.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_adc12.h"
#include "ti/devices/msp/peripherals/hw_oa.h"

//-----------------------------------------------------------------------------
// Define function prototypes used by the program
//-----------------------------------------------------------------------------
void start_microwave();
void config_pb1_interrupt(void);
void config_pb2_interrupt(void);

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define LOAD_VALUE           4000

//-----------------------------------------------------------------------------
// Define global variables and structures here.
// NOTE: when possible avoid using global variables
//-----------------------------------------------------------------------------


// Define a structure to hold different data types

int main(void) {
  clock_init_40mhz();
  launchpad_gpio_init();
  
  keypad_init();
  lcd1602_init();
  seg7_init();
  dipsw_init();
  lpsw_init();
  led_init();

  motor0_init();
  motor0_pwm_init(LOAD_VALUE, 0);
  ADC0_init(ADC12_MEMCTL_VRSEL_INTREF_VSSA);

  I2C_init();
  UART_init(115200);
 
  // Endless loop to prevent program from ending
  while (1);

} /* main */

void start_microwave() {
  
}

//------------------------------------------------------------------------------
// DESCRIPTION:
//    This function initializes push button 1 on the extension board to be used
//    as a interrupt mechanic.
//
// INPUT PARAMETERS:
//    none
//
// OUTPUT PARAMETERS:
//    none
//
// RETURN:
//    none
// -----------------------------------------------------------------------------
void config_pb1_interrupt() {
  asm("CPSID I");
  // Enable PB1
  GPIOB->POLARITY31_16 = GPIO_POLARITY31_16_DIO18_RISE;
  GPIOB->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO18_CLR;
  GPIOB->CPU_INT.IMASK = GPIO_CPU_INT_IMASK_DIO18_SET;
  NVIC_SetPriority(GPIOB_INT_IRQn, 2);
  NVIC_EnableIRQ(GPIOB_INT_IRQn);
  asm("CPSIE I");
}

//------------------------------------------------------------------------------
// DESCRIPTION:
//    This function initializes push button 2 on the extension board to be used
//    as a interrupt mechanic.
//
// INPUT PARAMETERS:
//    none
//
// OUTPUT PARAMETERS:
//    none
//
// RETURN:
//    none
// -----------------------------------------------------------------------------
void config_pb2_interrupt() {
  asm("CPSID I");
  // Enable PB2
  GPIOA->POLARITY15_0 = GPIO_POLARITY15_0_DIO15_FALL;
  GPIOA->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO15_CLR;
  GPIOA->CPU_INT.IMASK = GPIO_CPU_INT_IMASK_DIO15_SET;
  NVIC_SetPriority(GPIOA_INT_IRQn, 2);
  NVIC_EnableIRQ(GPIOA_INT_IRQn);
  asm("CPSIE I");
}

//------------------------------------------------------------------------------
// DESCRIPTION:
//    This function configures the GPIO interrupts on the MSPM0+ processor to 
//    detect when Push Button 1 and 2 is pressed. Once either Push button is 
//    pressed a global flag will be set to true indicating which one was pushed.
//    On top of that, push button 2 also obtains the ADC value located on 
//    channel 5 of ADC0.
//
// INPUT PARAMETERS:
//    none
//
// OUTPUT PARAMETERS:
//    none
//
// RETURN:
//    none
// -----------------------------------------------------------------------------
void GROUP1_IRQHandler(void) {
  uint32_t group_iidx_status;
  uint32_t gpio_mis;
  
  do {
    group_iidx_status = CPUSS->INT_GROUP[1].IIDX;

    switch(group_iidx_status) {
      case (CPUSS_INT_GROUP_IIDX_STAT_INT1):    // PB1
        gpio_mis = GPIOB->CPU_INT.MIS;
        if ((gpio_mis & GPIO_CPU_INT_MIS_DIO18_MASK) == GPIO_CPU_INT_MIS_DIO18_SET) {
          g_SW1_pressed = true;
          // Manually clear bit to acknowledge interrupt
          GPIOB->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO18_CLR;
        }
        break;

      case (CPUSS_INT_GROUP_IIDX_STAT_INT0):    // PB2
        gpio_mis = GPIOA->CPU_INT.MIS;
        if ((gpio_mis & GPIO_CPU_INT_MIS_DIO15_MASK) == GPIO_CPU_INT_MIS_DIO15_SET) {
          g_SW2_pressed = true;
          
          // Manually clear bit to acknowledge interrupt
          GPIOA->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO15_CLR;
        }
      break;
    }

  } while (group_iidx_status != 0);
}
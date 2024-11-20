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
#include <cstdint>
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
void GROUP1_IRQHandler(void);
void input_timer(void);
void countdown_timer (void);

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define LOAD_VALUE            (4000)
#define msec_5                (5)
#define debounce_delay        (300)
//-----------------------------------------------------------------------------
// Define global variables and structures here.
// NOTE: when possible avoid using global variables
//-----------------------------------------------------------------------------
g_pb1_pressed = false;
g_pb2_pressed = false;

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
  
} /* main */

void configure_microwave() {
  bool power_on = false;
  bool door_closed = true;

  typedef enum state {
    IDLE_STATE,
    SET_TIME,
    CHECK_DOOR_OPEN,
    MOTOR_CCW
  } FSM_TYPE_t;
  FSM_TYPE_t state = MOTOR_OFF1;


  uint8_t key = 0;
  uint8_t key_index = 3;
  uint8_t key_list[4];
  while (!power_on) {
    // get keypress or start microwave
    do {
      key = keypad_scan();

      if (g_SW2_pressed) {
        if (sizeof(key_list) > 0 && door_closed) {
          start_microwave();
        } 
        g_SW2_pressed = false;
      }

      if (g_SW1_pressed) {
        door_closed = !door_closed;
      }
    } while (key == 0x10);

    // Clear time
    if (key == 'C') {
      key_list.clear()
    }

    if (key != 'ABCD') {
      key_list[key_index--] = key;
    }

    seg7_hex(key);
  }

  start_microwave();
}

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
          g_pb1_pressed = true;
          // Manually clear bit to acknowledge interrupt
          GPIOB->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO18_CLR;
        }
        break;

      case (CPUSS_INT_GROUP_IIDX_STAT_INT0):    // PB2
        gpio_mis = GPIOA->CPU_INT.MIS;
        if ((gpio_mis & GPIO_CPU_INT_MIS_DIO15_MASK) == GPIO_CPU_INT_MIS_DIO15_SET) {
          g_pb2_pressed = true;
          
          // Manually clear bit to acknowledge interrupt
          GPIOA->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO15_CLR;
        }
      break;
    }

  } while (group_iidx_status != 0);
}
//------------------------------------------------------------------------------
// DESCRIPTION:
// 
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
void start_microwave (void)
{
  while (!g_pb1_pressed)
  {
    input_timer();
  }
  while (g_pb1_pressed)
  {
    countdown_timer();
  }
}
//------------------------------------------------------------------------------
// DESCRIPTION:
// 
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
void input_timer (void)
{
  uint8_t key_value;
  static uint8_t seg7_dig = 0;
  const char reset [] = {0x00 , 0x00 , 0x00 ,0x00};
  uint8_t i;

  keypad_scan() = key_value;

  if ((key_value != 0x10) & (key_value < 0x0A) & (seg7_dig < 5))
  {
    wait_no_key_pressed();
    seg7_hex(key_value, uint8_t seg7_dig);
    msec_delay(debounce_delay);
    seg7_dig++;
  }
  
  if (key_value == 0x0C)
  {
    for (i = 0; i < 4; i++)
    {
      seg7_hex(reset[i] , i);
      msec_delay(msec_5);
    }
    seg7_dig = 0;
  }
}
//------------------------------------------------------------------------------
// DESCRIPTION:
// 
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
void countdown_timer (void)
{
  
}
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
#include <math.h>
#include <stdio.h>

//-----------------------------------------------------------------------------
// Loads MSP launchpad board support macros and definitions
//-----------------------------------------------------------------------------
#include <ti/devices/msp/msp.h>
#include "clock.h"
#include "LaunchPad.h"
#include "lcd1602.h"

#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_adc12.h"
#include "ti/devices/msp/peripherals/hw_oa.h"

#include "uart.h"
#include "adc.h"

//-----------------------------------------------------------------------------
// Define function prototypes used by the program
//-----------------------------------------------------------------------------
void start_microwave(uint16_t time, uint8_t power);
int menu_selection();
void configure_microwave();
int power_selection();

void input_timer(void);
void countdown_timer (void);
bool check_time_array(uint8_t key_list[]);

uint16_t time_array_to_int(uint8_t key_list[], uint8_t max_index);
void printMsg(char *str);
void skipLine(void);

void config_pb1_interrupt(void);
void config_pb2_interrupt(void);
void GROUP1_IRQHandler(void);

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define LOAD_VALUE            (4000)
#define msec_5                (5)
#define DEBOUNCE_DELAY        (300)

#define MENU_ITEM1            '1'
#define MENU_ITEM2            '2'
#define MENU_ITEM3            '3'
#define MENU_ITEM4            '4'
#define MENU_ITEM5            '5'
#define MENU_ITEM6            '6'

#define DEFROST_TIME          1000
#define PIZZA_TIME             700
#define POPCORN_TIME           300
#define POTATO_TIME            800
#define BEVERAGE_TIME           40

#define NUMPAD_KEY_A            10
#define NUMPAD_KEY_B            11
#define NUMPAD_KEY_C            12
#define NUMPAD_KEY_D            13
#define NUMPAD_KEY_ASTERISK     14
#define NUMPAD_KEY_#            15

#define MSPM0_CLOCK_FREQUENCY                                             (40E6)
#define SYST_TICK_PERIOD                                              (10.25E-3)
#define SYST_TICK_PERIOD_COUNT        (SYST_TICK_PERIOD * MSPM0_CLOCK_FREQUENCY)

//-----------------------------------------------------------------------------
// Define global variables and structures here.
// NOTE: when possible avoid using global variables
//-----------------------------------------------------------------------------
bool g_pb1_pressed = false;
bool g_pb2_pressed = false;
bool run_microwave = false;

int main(void) {
  clock_init_40mhz();
  launchpad_gpio_init();
  
  keypad_init();
  lcd1602_init();
  seg7_init();
  dipsw_init();
  lpsw_init();
  led_init();
  led_enable();

  I2C_init();
  UART_init(115200);

  // interferes with LEDS so use only when motor runs
  motor0_init();
  motor0_pwm_init(LOAD_VALUE, 0);

  sys_tick_init(SYST_TICK_PERIOD_COUNT);
  ADC0_init(ADC12_MEMCTL_VRSEL_INTREF_VSSA);

  config_pb1_interrupt();
  config_pb2_interrupt();

  lcd_clear();
  configure_microwave();
  
  // Disable interrupts
  NVIC_DisableIRQ(GPIOA_INT_IRQn);
  NVIC_DisableIRQ(GPIOB_INT_IRQn);
} /* main */

void configure_microwave() {
  bool power_on = false;
  bool door_closed = true;

  uint8_t microwave_power = 2;
  uint8_t key = 0;
  uint8_t key_index = 0;
  uint8_t key_list[4];

  while (!power_on) {
    // get keypress or start microwave
    do {
      key = keypad_scan();
      wait_no_key_pressed();
      msec_delay(DEBOUNCE_DELAY);

      // Start Microwave if valid input time
      if (g_pb2_pressed) {
        if (key_index > 0 && door_closed) {
          lcd_set_ddram_addr(LCD_LINE2_ADDR);
          lcd_write_string("Microwave started");
          uint16_t time = time_array_to_int(key_list, key_index);
          run_microwave = true;
          start_microwave(time, microwave_power);
          g_pb2_pressed = false;
        }
        g_pb2_pressed = false;
      }

      // Open or close door
      if (g_pb1_pressed) {
        door_closed = !door_closed;
        if (door_closed) {
          lcd_set_ddram_addr(LCD_LINE2_ADDR);
          lcd_write_string("Door Closed");
        } else {
          lcd_set_ddram_addr(LCD_LINE2_ADDR);
          lcd_write_string("Door Open");
        }
        g_pb1_pressed = false;
      }
    } while (key == 0x10);

    // Clear time on LCD
    if (key == 12) {
      lcd_clear();
      key_index = 0;

      for (uint8_t i = 0; i < sizeof(key_list); i++) {
        key_list[i] = 0;
      }
    }

    if (key == 11) {
      int selected_item = menu_selection();

      lcd_set_ddram_addr(LCD_LINE1_ADDR);
      lcd_write_doublebyte(selected_item);
      
      // start_microwave();
    }

    if (key == 10) {
      microwave_power = power_selection();
      lcd_clear();
    }

    if (key != 10 && key != 11 && key != 12 && key != 13 && key != 14 && key != 15 && key_index <= 3) {
      lcd_set_ddram_addr(LCD_LINE1_ADDR + key_index);
      hex_to_lcd(key);
      key_list[key_index++] = key;
    }
  }

  // start_microwave();
}

int menu_selection() {
  char menu[] = "MENU OPTIONS\n"
                "    1. Defrost\n"
                "    2. Pizza\n"
                "    3. Popcorn\n"
                "    4. Potato\n"
                "    5. Beverage\n"
                "    6. Exit\n\n"
                "Enter your selection: ";
  char errorMsg[] = "Please enter a valid menu option";
  bool finished = false;
  int time = 0;

  while (!finished) {
    printMsg(menu);
    char input = UART_in_char();
    UART_out_char(input);
    skipLine();
    if (input > '6' || input < '1') {
      printMsg(errorMsg);
      skipLine();
    }

    switch (input) {
      case (MENU_ITEM1):
        time = DEFROST_TIME;
        finished = true;
        break;
        
      case (MENU_ITEM2):
        time = PIZZA_TIME;
        break;

      case (MENU_ITEM3):
        time = POPCORN_TIME;
        break;

      case (MENU_ITEM4):
        time = POTATO_TIME;
        break;   

      case (MENU_ITEM5):
        time = BEVERAGE_TIME;
        break;    

      case (MENU_ITEM6):
        finished = true;
        break;        
    }
  }

  return time;
}

int power_selection() {
  uint16_t power = 0;
  uint16_t key = 0;
  bool finished = false;

  lcd_set_ddram_addr(LCD_LINE1_ADDR);
  lcd_write_string("Pwr Level: ");

  lcd_set_ddram_addr(LCD_LINE2_ADDR);
  lcd_write_string("Press * to exit");

  while (!finished) {
    do {
      key = keypad_scan();
      wait_no_key_pressed();
      msec_delay(DEBOUNCE_DELAY);
    } while (key == 0x10);

    switch (key) {
      case (1):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_byte(power);
      
        leds_on(key);
        break;

      case (2):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_byte(power);
      
        leds_on(key);
        break;

      case (3):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_10);
        lcd_write_doublebyte(power);
      
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_write_string("Pwr Level: ");

        leds_on(key);
        break;

      case (4):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(key);
        break;

      case (5):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(key);
        break;
              
      case (6):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(key);
        break;

      case (7):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(key);
        break;

      case (8):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(key);
        break;
        
      case (9):
        power = key * 10;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(key);
        break;

      case (0):
        power = 100;
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
        lcd_write_doublebyte(power);
      
        leds_on(10);
        break;
      
      case (NUMPAD_KEY_ASTERISK): 
        finished = true;
    }
  }

  return power;
}

// void start_microwave(uint16_t time, uint8_t power) {
//   uint16_t time_in_msec = time * 1000;
//   uint8_t duty_cycle = ((power) * 100) / 16;

//   lcd_clear();
//   lcd_write_doublebyte(time);

//   motor0_pwm_enable();
//   motor0_set_pwm_dc(duty_cycle);

//   lcd_set_ddram_addr(LCD_LINE2_ADDR);
//   lcd_write_doublebyte(time_in_msec);

//   // for (uint16_t count = 0; count < time_in_msec; count++) {
//   //   lcd_set_ddram_addr(LCD_LINE1_ADDR);
//   //   lcd_write_doublebyte(count);
//   //   lcd_set_ddram_addr(LCD_LINE2_ADDR);
//   //   lcd_write_doublebyte(time_in_msec);
//   //   msec_delay(1);
//   // }

//   msec_delay(time_in_msec);
  
//   motor0_pwm_disable();
// }

uint16_t time_array_to_int(uint8_t key_list[], uint8_t max_index) {
  uint16_t time = 0;
  uint16_t multiplier = pow(10, max_index - 1);

  for (uint8_t i = 0; i < max_index; i++) {
    time += key_list[i] * multiplier;
    multiplier /= 10;
  }

  return time;
}

//------------------------------------------------------------------------------
// DESCRIPTION:
//    This function takes a string and prints it into the terminal emulator 
// 
// INPUT PARAMETERS:
//    str - a string
//
// OUTPUT PARAMETERS:
//    none
//
// RETURN:
//    none
// -----------------------------------------------------------------------------
void printMsg(char *str) {
  uint8_t index = 0;

  while (str[index] != '\0') {
    UART_out_char(str[index]);
    index++;
  }
}

//------------------------------------------------------------------------------
// DESCRIPTION:
//    This function performs a line skip in the terminal emulator.
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
void skipLine() {
  UART_out_char('\n');
  UART_out_char('\n');
}

// void SysTick_Handler(void) {
//   int second_intervals = 0;

//   if (run_microwave) {
//     for (int i = 0; i < 3000; i++) {
//       if (second_intervals == i) {
//         lcd_set_ddram_addr(LCD_LINE1_ADDR);
//         lcd_write_doublebyte(i);
//         second_intervals += 1000;
//       }
//     }
//   }
// }

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
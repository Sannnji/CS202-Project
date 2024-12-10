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
int menu_selection(uint8_t *time_array[], uint8_t *time_num_size);
void configure_microwave();
int power_selection();
void set_microwave_power(uint16_t power);

void clear_time(uint8_t *time_array[], uint8_t *time_num_size) ;
bool check_time_array(uint8_t time_array[]);

uint16_t time_array_to_int(uint8_t time_array[], uint8_t max_index);
void int_to_array(int num, uint8_t *arr, int *size);
void printMsg(char *str);
void skipLine(void);

void config_pb1_interrupt(void);
void config_pb2_interrupt(void);
void GROUP1_IRQHandler(void);

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define LOAD_VALUE            4000
#define msec_5                   5
#define msec_2000             2000
#define DEBOUNCE_DELAY         300
#define BLINK_DELAY           1000
#define HOLD_ZERO_COUNT       1000
#define MAX_TIME_ARR_IDX         3

#define LED6                   (6)
#define LED5_BI           (0b0101) 

#define d_LETTER              0x5E
#define o_LETTER              0x5C
#define n_LETTER              0x54
#define E_LETTER              0x79

#define MENU_ITEM1             '1'
#define MENU_ITEM2             '2'
#define MENU_ITEM3             '3'
#define MENU_ITEM4             '4'
#define MENU_ITEM5             '5'
#define MENU_ITEM6             '6'

#define DEFROST_TIME          1000
#define PIZZA_TIME             700
#define POPCORN_TIME           300
#define POTATO_TIME            800
#define BEVERAGE_TIME           40

#define DEFROST_TIME_NUM_SIZE    4
#define PIZZA_TIME_NUM_SIZE      3
#define POPCORN_TIME_NUM_SIZE    3
#define POTATO_TIME_NUM_SIZE     3
#define BEVERAGE_TIME_NUM_SIZE   2

#define NUMPAD_KEY_A            10
#define NUMPAD_KEY_B            11
#define NUMPAD_KEY_C            12
#define NUMPAD_KEY_D            13
#define NUMPAD_KEY_ASTERISK     14
#define NUMPAD_KEY_POUND        15

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


uint8_t microwave_power = 5;
uint16_t time_in_msec = 0;
uint16_t time = 0;

int main(void) {
  clock_init_40mhz();
  launchpad_gpio_init();
  
  keypad_init();
  lcd1602_init();
  dipsw_init();
  lpsw_init();
  led_init();
  seg7_init();
  
  I2C_init();
  UART_init(115200);

  motor0_init();
  motor1_init();
  motor0_pwm_init(LOAD_VALUE, 0);

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
  bool reprint_menu = false;

  uint8_t key = 0;
  uint8_t time_num_size = 0;
  uint8_t time_array[4];
  char power_string[2];
  char time_string[4];

  while (!power_on) {
    // get keypress or start microwave
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    lcd_write_string("Time: ");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    lcd_write_string("Power: ");
    lcd_set_ddram_addr(LCD_LINE2_ADDR + LCD_CHAR_POSITION_8);
    sprintf(power_string, "%d", microwave_power);
    lcd_write_string(power_string);
    leds_on(LED5_BI);     
    led_enable();

    do {
      key = keypad_scan();
      wait_no_key_pressed();

      // Start Microwave if valid input time
      if (g_pb2_pressed) {
        if (time_num_size > 0 && door_closed) {
          g_pb2_pressed = false;

          time = time_array_to_int(time_array, time_num_size);
          
          run_microwave = true;
          start_microwave(time, microwave_power);
          clear_time(time_array, &time_num_size);
          
          // Set key to exit loop and reprint
          key = 1;
          reprint_menu = true;
        }
        g_pb2_pressed = false;
      }

      // Open or close door
      if (g_pb1_pressed) {
        door_closed = !door_closed;
        if (door_closed) {
          led_off(LED6);
          lcd_set_ddram_addr(LCD_LINE2_ADDR);
          lcd_write_string("Door Closed");
          msec_delay(msec_2000);
          lcd_set_ddram_addr(LCD_LINE2_ADDR);
          lcd_write_string("Power: ");
          lcd_set_ddram_addr(LCD_LINE2_ADDR + LCD_CHAR_POSITION_8);
          sprintf(power_string, "%d", microwave_power);
          lcd_write_string(power_string);
          lcd_write_string("   ");

        } else {
          led_on(LED6);
          lcd_set_ddram_addr(LCD_LINE2_ADDR);
          lcd_write_string("Door Opened");
        }
        g_pb1_pressed = false;
      }
    } while (key == 0x10);

    if (!reprint_menu) {
      // Add time 
      if (key != NUMPAD_KEY_A && key != NUMPAD_KEY_B && key != NUMPAD_KEY_C && key != NUMPAD_KEY_D && key != NUMPAD_KEY_ASTERISK && key != NUMPAD_KEY_POUND) {
        if (time_num_size <= MAX_TIME_ARR_IDX) {
          lcd_set_ddram_addr(LCD_LINE1_ADDR + LCD_CHAR_POSITION_7 + time_num_size);
          hex_to_lcd(key);
          time_array[time_num_size++] = key;
        }
      }

      // Power Selection
      if (key == NUMPAD_KEY_A) {
        int power = power_selection();
        if (power != 0) {
          microwave_power = power;
        }
        key = NUMPAD_KEY_C;

        lcd_clear();
      }

      // Menu Selection
      if (key == NUMPAD_KEY_B) {
        int selected_item = menu_selection(time_array, &time_num_size);
        time = time_array_to_int(time_array, time_num_size);
        sprintf(time_string, "%d", time);

        // clear_time(time_array, &time_num_size);

        lcd_set_ddram_addr(LCD_LINE1_ADDR + LCD_CHAR_POSITION_7);
        lcd_write_string(time_string);
      }

      // Clear time on LCD
      if (key == NUMPAD_KEY_C) {
        clear_time(time_array, &time_num_size);
      }
    } 

    reprint_menu = false;
  }
}

int menu_selection(uint8_t *time_array[], uint8_t *time_num_size) {
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
  int menu_item_time = 0;
  int size = 0;

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
        menu_item_time = DEFROST_TIME;
        size = DEFROST_TIME_NUM_SIZE;
        *time_num_size = size;
        finished = true;
        break;
        
      case (MENU_ITEM2):
        menu_item_time = PIZZA_TIME;
        size = PIZZA_TIME_NUM_SIZE;
        *time_num_size = size;
        finished = true;
        break;

      case (MENU_ITEM3):
        menu_item_time = POPCORN_TIME;
        size = POPCORN_TIME_NUM_SIZE;
        *time_num_size = size;
        finished = true;
        break;

      case (MENU_ITEM4):
        menu_item_time = POTATO_TIME;
        size = POTATO_TIME_NUM_SIZE;
        *time_num_size = size;
        finished = true;
        break;   

      case (MENU_ITEM5):
        menu_item_time = BEVERAGE_TIME;
        size = BEVERAGE_TIME_NUM_SIZE;
        *time_num_size = size;
        finished = true;
        break;    

      case (MENU_ITEM6):
        finished = true;
        break;        
    }
  }

  if (menu_item_time != 0) {
    int_to_array(menu_item_time, time_array, &size);
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
        power = key;
        set_microwave_power(key);
        break;

      case (2):
        power = key;
        set_microwave_power(key);
        break;

      case (3):
        power = key;
        set_microwave_power(key);
        break;

      case (4):
        power = key;
        set_microwave_power(key);
        break;

      case (5):
        power = key;
        set_microwave_power(key);
        break;
      
      case (NUMPAD_KEY_ASTERISK): 
        finished = true;
    }
  }

  return power;
}

void start_microwave(uint16_t time, uint8_t power) {
  typedef enum state {
    MICROWAVE_RUNNING,
    MICROWAVE_STOPPED
  } FSM_TYPE_t;
  FSM_TYPE_t state = MICROWAVE_RUNNING;

  int pb2_push_count = 0;
  bool microwave_finished = false;
  bool microwave_stopped = false;
  bool turntable_cw = true;
  
  int count = 100;
  int sec_interval_in_msec = 0;
  int blink_count = 2;
  char time_string[4];
  int current_time_in_msec = 0;
  int time_placeholder = 0;

  uint8_t key = 0;

  time_in_msec = time * 1000;
  uint8_t duty_cycle = ((power) * 100) / 5;

  motor0_set_pwm_dc(duty_cycle);
  motor0_pwm_enable();
  led_on(LED6);

  while (!microwave_finished) {
    switch (state) {
      case MICROWAVE_RUNNING:
        // If microwave is resuming set current time to where it left off
        if (time_placeholder != 0) {
          current_time_in_msec = time_placeholder;
        } else {
          current_time_in_msec = 0;
        }

        for (current_time_in_msec; current_time_in_msec <= time_in_msec; current_time_in_msec++) {
          // Stop microwave when pb2 is pressed
          if (g_pb2_pressed) {
            msec_delay(DEBOUNCE_DELAY);
            microwave_stopped = !microwave_stopped;
            g_pb2_pressed = false;
            microwave_stopped ? motor0_pwm_disable() : motor0_pwm_enable();
            state = MICROWAVE_STOPPED;
            // Save time, if it the user does not clear it and wants to resume
            time_placeholder = current_time_in_msec;
            // Set current time to be out of bounds to exit loop
            current_time_in_msec = time_in_msec + 2;
          }

          // If microwave is stopped then deincrement to stall
          if (!microwave_stopped && (sec_interval_in_msec == current_time_in_msec)) {
            motor1_set_pwm_count(count);
            lcd_set_ddram_addr(LCD_CHAR_POSITION_7);
            sprintf(time_string, "%d", time);
            lcd_write_string(time_string);
            lcd_write_string("   ");
            time--;
            sec_interval_in_msec += 1000;
            // Turn turntable clockwise
            if (turntable_cw){
              count += 100;
            }
            // Turn turntable counter clocksise
            else{
              count -= 100;
            }

            if (count == 500){
              turntable_cw = false;
            }

            if (count == 100){
              turntable_cw = true;
            }

          }
          msec_delay(1);
        }
        if (state != MICROWAVE_STOPPED) {
          microwave_finished = true;
        }
        break;

      case MICROWAVE_STOPPED:
        do {
          key = keypad_scan();
          wait_no_key_pressed();
          msec_delay(DEBOUNCE_DELAY);

          if (g_pb2_pressed) {
            msec_delay(DEBOUNCE_DELAY);
            microwave_stopped = !microwave_stopped;
            g_pb2_pressed = false;
            microwave_stopped ? motor0_pwm_disable() : motor0_pwm_enable();
            state = MICROWAVE_RUNNING;

            // Set key to exit loop
            key = NUMPAD_KEY_B;
          }
        } while (key == 0x10);

        if (key == NUMPAD_KEY_C) {
          microwave_finished = true;
        }
        break;
    }
  }

  // If time was not cleared 
  if (key != NUMPAD_KEY_C) {
    motor0_pwm_disable();
    msec_delay(HOLD_ZERO_COUNT);
    lcd_clear();
    led_off(LED6);
    led_disable();
    for (int i = 0; i < blink_count; i++) {

     int count = 0;
     const char letters[] = {d_LETTER,o_LETTER,n_LETTER,E_LETTER};
     uint8_t p;

     while(count < 101)
     {
       for (p=0; p<4; p++)
       {
         seg7_on(letters[p] , p);
         msec_delay(msec_5);
       }
            
       count++;
     }
        
     seg7_off();
     msec_delay(BLINK_DELAY);
    }
  }
      
  sec_interval_in_msec = 0;
  time_in_msec = 0;
  time_placeholder = 0;
  current_time_in_msec = 0;
  run_microwave = false;
}

void set_microwave_power(uint16_t power) {
  lcd_set_ddram_addr(LCD_LINE1_ADDR);
  lcd_set_ddram_addr(LCD_CHAR_POSITION_12);
  hex_to_lcd(power);

  leds_on(power);
}

//------------------------------------------------------------------------------
// DESCRIPTION:
//    This function takes a array of single digit numbers and turns it into a 
//    number. For example: [1,2] == 12
// 
// INPUT PARAMETERS:
//    time_array - an array of single digit numbers
//    max_index  - the max index that denotes the length of the number
//
// OUTPUT PARAMETERS:
//    none
//
// RETURN:
//    time - the number that was created from the array 
// -----------------------------------------------------------------------------
uint16_t time_array_to_int(uint8_t time_array[], uint8_t max_index) {
  uint16_t time = 0;
  uint16_t multiplier = pow(10, max_index - 1);

  for (uint8_t i = 0; i <= max_index; i++) {
    time += time_array[i] * multiplier;
    multiplier /= 10;
  }

  return time;
}

void int_to_array(int num, uint8_t *arr, int *size) {
    // Fill the array with digits
    for (int i = *size - 1; i >= 0; i--) {
        arr[i] = num % 10;
        num /= 10;
    }
}

void clear_time(uint8_t *time_array[], uint8_t *time_num_size) {
  lcd_clear();
  *time_num_size = 0;

  for (int i = 0; i < sizeof(time_array); i++) {
    time_array[i] = 0;
  }
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
          g_pb2_pressed = false;
          msec_delay(DEBOUNCE_DELAY);
          // Manually clear bit to acknowledge interrupt
          GPIOB->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO18_CLR;
        }
        break;

      case (CPUSS_INT_GROUP_IIDX_STAT_INT0):    // PB2
        gpio_mis = GPIOA->CPU_INT.MIS;
        if ((gpio_mis & GPIO_CPU_INT_MIS_DIO15_MASK) == GPIO_CPU_INT_MIS_DIO15_SET) {
          if(!g_pb1_pressed){
          g_pb2_pressed = true;
          }
          msec_delay(DEBOUNCE_DELAY);
          // Manually clear bit to acknowledge interrupt
          GPIOA->CPU_INT.ICLR = GPIO_CPU_INT_ICLR_DIO15_CLR;
        }
        break;
    }

  } while (group_iidx_status != 0);
}
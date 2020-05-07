
#ifndef __MOTOR_H__
#define __MOTOR_H__


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"


/**
 * TEST CODE BRIEF
 *
 * Use PCNT module to count rising edges generated by LEDC module.
 *
 * Functionality of GPIOs used in this example:
 *   - GPIO4 - pulse input pin,
 *   - GPIO5 - control input pin.
 *
 * Load example, open a serial port to view the message printed on your screen.
 *
 * To do this test, you should connect GPIO18 with GPIO4.
 * GPIO5 is the control signal, you can leave it floating with internal pull up,
 * or connect it to ground. If left floating, the count value will be increasing.
 * If you connect GPIO5 to GND, the count value will be decreasing.
 *
 * An interrupt will be triggered when the counter value:
 *   - reaches 'thresh1' or 'thresh0' value,
 *   - reaches 'l_lim' value or 'h_lim' value,
 *   - will be reset to zero.
 */
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      0x7FFF
#define PCNT_L_LIM_VAL      0x8000


#define GPIO_PWM0A_OUT 13   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 12   //Set GPIO 16 as PWM0B


xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;


void encoderInit(pcnt_unit_t pcntUnit, int pinA, int pinB);

void mcpwm_example_gpio_initialize();
void mcpwm_example_brushed_motor_control(void *arg);


/* Task originally from esp idf pcnt example that reads encoder events */
void encoderEventsTask(void *arg);


/* returns the encoder count for specified encoder*/
int32_t encoderCount(pcnt_unit_t pcntUnit);



struct Plotter{
   /* 
      left/right current count and target

      config data:
      velocity, acceleration, motor spacing

    */
};

#endif /* __MOTOR_H__ */

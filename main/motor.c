

#include "motor.h"
#include "wifi.h"

pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

volatile int32_t motorOverflow = 0;


int32_t encoderCount() {
   int16_t count = 0;
   pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
   return (PCNT_H_LIM_VAL * motorOverflow) + count;
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
   uint32_t intr_status = PCNT.int_st.val;
   int i;
   pcnt_evt_t evt;
   portBASE_TYPE HPTaskAwoken = pdFALSE;

   for (i = 0; i < PCNT_UNIT_MAX; i++) {
      if (intr_status & (BIT(i))) {
	 evt.unit = i;
	 /* Save the PCNT event type that caused an interrupt
	    to pass it to the main program */
	 evt.status = PCNT.status_unit[i].val;
	 PCNT.int_clr.val = BIT(i);
	 xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
	 if (evt.status & PCNT_STATUS_L_LIM_M) {
	    motorOverflow--;
	 }
	 if (evt.status & PCNT_STATUS_H_LIM_M) {
	    motorOverflow++;
	 }

	 if (HPTaskAwoken == pdTRUE) {
	    portYIELD_FROM_ISR();
	 }
      }
   }
}


/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
void pcnt_example_init(void)
{
   /* Prepare configuration for the PCNT unit */
   pcnt_config_t pcnt_config = {
				// Set PCNT input signal and control GPIOs
				.pulse_gpio_num = PCNT_INPUT_SIG_IO,
				.ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
				.channel = PCNT_CHANNEL_0,
				.unit = PCNT_TEST_UNIT,
				// What to do on the positive / negative edge of pulse input?
				.pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
				.neg_mode = PCNT_COUNT_DEC,   // Keep the counter value on the negative edge
				// What to do when control input is low or high?
				.lctrl_mode = PCNT_MODE_KEEP, // Keep counting direction if low
				.hctrl_mode = PCNT_MODE_REVERSE,    // reverse the primary counter mode if high
				// Set the maximum and minimum limit values to watch
				.counter_h_lim = PCNT_H_LIM_VAL,
				.counter_l_lim = PCNT_L_LIM_VAL,
   };
   /* Initialize PCNT unit */
   pcnt_unit_config(&pcnt_config);

   /* Configure and enable the input filter */
   /* pcnt_set_filter_value(PCNT_TEST_UNIT, 100); */
   /* pcnt_filter_enable(PCNT_TEST_UNIT); */

   /* Set threshold 0 and 1 values and enable events to watch */
   pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
   pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
   pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
   pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
   /* Enable events on zero, maximum and minimum limit values */
   pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
   pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
   pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

   /* Initialize PCNT's counter */
   pcnt_counter_pause(PCNT_TEST_UNIT);
   pcnt_counter_clear(PCNT_TEST_UNIT);

   /* Register ISR handler and enable interrupts for PCNT unit */
   pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
   pcnt_intr_enable(PCNT_TEST_UNIT);

   /* Everything is set up, now go to counting */
   pcnt_counter_resume(PCNT_TEST_UNIT);
}


void encoderEventsTask(void *arg) {
   int16_t count = 0;
   int32_t overflow = 0;
   pcnt_evt_t evt;
   portBASE_TYPE res;

   while (1) {
      /* Wait for the event information passed from PCNT's interrupt handler.
       * Once received, decode the event type and print it on the serial monitor.
       */
      res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
      if (res == pdTRUE) {
	 pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
	 printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
	 if (evt.status & PCNT_STATUS_THRES1_M) {
	    printf("THRES1 EVT\n");
	 }
	 if (evt.status & PCNT_STATUS_THRES0_M) {
	    printf("THRES0 EVT\n");
	 }
	 if (evt.status & PCNT_STATUS_L_LIM_M) {
	    printf("L_LIM EVT\n");
	    overflow--;
	 }
	 if (evt.status & PCNT_STATUS_H_LIM_M) {
	    printf("H_LIM EVT\n");
	    overflow++;
	 }
	 if (evt.status & PCNT_STATUS_ZERO_M) {
	    printf("ZERO EVT\n");
	 }
      } else {
	 pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
	 printf("Current counter value :%d\n", (PCNT_H_LIM_VAL * overflow) + count);
	    
      }
   }
}

/* ############################# MOTOR ############################### */
void mcpwm_example_gpio_initialize()
{
   printf("initializing mcpwm gpio...\n");
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}


void setSpeed(int motorNum , float duty_cycle) {
   if(motorNum == 0) {
      if(duty_cycle >= 0) {	/* forward */
	 mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
	 mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
	 mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

      } else {			/* backward */
	 mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
	 mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -1*duty_cycle);
	 mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
	 
      }
      
   } else {
      printf("\nINVALID MOTOR CONFIGURATION\n\n");
   }

}


/**
 * @brief Configure MCPWM module for brushed dc motor
 */
void mcpwm_example_brushed_motor_control(void *arg)
{
   char debugBuf[64];
   int32_t motorPosition = 0;
   int32_t targetPosition = 0;
   int32_t error;
   float Kp = -0.5;
   /* wsRegisterVariable( &motorPosition, 'l', "motorPosition"); */
   wsRegisterVariable( &targetPosition, 'l', "targetPosition");
   wsRegisterVariable( &Kp, 'f', "Kp");
   /* wsRegisterVariable( &Kd, 'f', "Kd"); */
   /* wsRegisterVariable( &Ki, 'f', "Ki"); */
   //1. mcpwm gpio initialization
   mcpwm_example_gpio_initialize();

   //2. initial mcpwm configuration
   printf("Configuring Initial Parameters of mcpwm...\n");
   mcpwm_config_t pwm_config;
   pwm_config.frequency = 1000;    //frequency = 500Hz,
   pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
   pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
   while (1) {
      motorPosition = encoderCount();

      
      error = (motorPosition - targetPosition) * Kp;
      if(error >  100) error =  100;
      if(error < -100) error = -100;
      setSpeed(0, error);

      sprintf(debugBuf, "d motorPosition %d, targetPosition %d, Kp %f \n", motorPosition, targetPosition, Kp );
      wsDebug(debugBuf);

      vTaskDelay(10 / portTICK_RATE_MS);
   }
}



/* ########################## ^^ MOTOR ^^ ############################ */




#include "motor.h"
#include "wifi.h"


volatile int32_t motorOverflow[8] = {0}; /* store over/underflow events for encoders */


pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle


/* Returns the encoder count for specified pulse counter,
   incorporating the over/underflow to get a 32 bit num*/
int32_t encoderCount(pcnt_unit_t pcntUnit) {
   int16_t count = 0;
   pcnt_get_counter_value(pcntUnit, &count);

   return (PCNT_H_LIM_VAL * motorOverflow[pcntUnit]) + count;
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
	    motorOverflow[i]--;
	 }
	 if (evt.status & PCNT_STATUS_H_LIM_M) {
	    motorOverflow[i]++;
	 }

	 if (HPTaskAwoken == pdTRUE) {
	    portYIELD_FROM_ISR();
	 }
      }
   }
}

/* Sets up a pulse counter as an interrupt with given 2 pins
   
   TODO: utilize second channel to double encoder resolution
 */
void encoderInit(pcnt_unit_t pcntUnit, int pinA, int pinB) {
   /* Prepare configuration for the PCNT unit */
   pcnt_config_t pcnt_config =
      {
       // Set PCNT input signal and control GPIOs
       .pulse_gpio_num = pinA,
       .ctrl_gpio_num = pinB,
       .channel = PCNT_CHANNEL_0,
       .unit = pcntUnit,
       // What to do on the positive / negative edge of pulse input?
       .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
       .neg_mode = PCNT_COUNT_DEC,   // Count down on negative edge
       // What to do when control input is low or high?
       .lctrl_mode = PCNT_MODE_KEEP, // Keep counting direction if low
       .hctrl_mode = PCNT_MODE_REVERSE,    // reverse the primary counter mode if high
       // Set the maximum and minimum limit values to watch
       .counter_h_lim = PCNT_H_LIM_VAL,
       .counter_l_lim = PCNT_L_LIM_VAL,
   };
   /* Initialize PCNT unit */
   pcnt_unit_config(&pcnt_config);

   /* Here is where I would insert ch1 config */

   
   pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);
   pcnt_event_enable(pcntUnit, PCNT_EVT_L_LIM);

   /* Initialize PCNT's counter */
   pcnt_counter_pause(pcntUnit);
   pcnt_counter_clear(pcntUnit);

   /* Register ISR handler and enable interrupts for PCNT unit */
   pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
   pcnt_intr_enable(pcntUnit);

   /* Everything is set up, now go to counting */
   pcnt_counter_resume(pcntUnit);

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
      motorPosition = encoderCount(0);

      
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


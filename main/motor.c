

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


   /* Use the second channel, switching the control and pulse pins and
      the direction to get full quadrature resolution*/
   pcnt_config.pulse_gpio_num = pinB;
   pcnt_config.ctrl_gpio_num = pinA;
   pcnt_config.channel = PCNT_CHANNEL_1;
   pcnt_config.pos_mode = PCNT_COUNT_DEC;   // Count down on the positive edge
   pcnt_config.neg_mode = PCNT_COUNT_INC;   // Count UP on negative edge
   

   /* Initialize PCNT unit with second channel configured */
   pcnt_unit_config(&pcnt_config);

      
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

/* Initializes the gpio and timer needed for controlling a DC motor.
   mcpwmUnit should be 0 or 1, freq is in Hz (I'm using ~1000), and then the gpio pins */
void motorInit(mcpwm_unit_t mcpwmUnit, uint32_t freq, int pinA, int pinB) {

   /* Config MCPWM peripheral */
   printf("Configuring Initial Parameters of mcpwm...\n");
   mcpwm_config_t pwm_config;
   pwm_config.frequency = freq;
   pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
   pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

   
   /* init GPIO */
   if( mcpwmUnit == MCPWM_UNIT_0){
      mcpwm_gpio_init(mcpwmUnit, MCPWM0A, pinA);
      mcpwm_gpio_init(mcpwmUnit, MCPWM0B, pinB);
      mcpwm_init(mcpwmUnit, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
   }else {
      mcpwm_gpio_init(mcpwmUnit, MCPWM1A, pinA);
      mcpwm_gpio_init(mcpwmUnit, MCPWM1B, pinB);
       mcpwm_init(mcpwmUnit, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
   }

}

/* Set speed of the indicated motor (either 0 or 1) and a duty cycle cycle specified from -100 to +100) */
void setSpeed(mcpwm_unit_t mcpwmUnit , float duty_cycle) {
   if(duty_cycle >= 0) {	/* forward */
      mcpwm_set_signal_low(mcpwmUnit, mcpwmUnit, MCPWM_OPR_B);
      mcpwm_set_duty(mcpwmUnit, mcpwmUnit, MCPWM_OPR_A, duty_cycle);
      mcpwm_set_duty_type(mcpwmUnit, mcpwmUnit, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

   } else {			/* backward */
      mcpwm_set_signal_low(mcpwmUnit, mcpwmUnit, MCPWM_OPR_A);
      mcpwm_set_duty(mcpwmUnit, mcpwmUnit, MCPWM_OPR_B, -1*duty_cycle);
      mcpwm_set_duty_type(mcpwmUnit, mcpwmUnit, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
	 
   }

}


/* 
   given a control structure (which will be unique for each device so it can store history),
   and the set point, and current position, this function calculates output (from -100 to 100)
   
   * Assumes called at constant interval
 */
float pid(control_config_t K, int32_t setpoint, int32_t current) {
   int32_t error;
   int32_t derivative;
   int32_t output;
   error = (setpoint - current);
   K.integral   += error;
   derivative  = error - K.previousError;
   
   output = K.p * error + K.i * K.integral + K.d * derivative;

   K.previousError = error;
   
   if(output >  100) output =  100;
   if(output < -100) output = -100;
   return output;
}
   

void motorControl(void *arg) {
   /* 
      Set period
    */
   int32_t setPoint0 = 0;
   int32_t setPoint1 = 0;
   control_config_t ctrl0 = { .p=0.4, .i=0.1, .d=0.1, .previousError=0, .integral=0};
   control_config_t ctrl1 = { .p=0.4, .i=0.1, .d=0.1, .previousError=0, .integral=0};

   TickType_t xLastWakeTime;
   xLastWakeTime = xTaskGetTickCount ();
   

   while (1) {
      setSpeed(0, pid(ctrl0, setPoint0, encoderCount(0) ) );
      setSpeed(1, pid(ctrl1, setPoint1, encoderCount(1) ) );
      
      vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
   }
}



void initMotors( motor_config_t config){
   /* Initialize PCNT event queue and PCNT functions */
   pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

   encoderInit(PCNT_UNIT_0, config.encoder0A, config.encoder0B);
   encoderInit(PCNT_UNIT_1, config.encoder1A, config.encoder1B);
   motorInit(MCPWM_UNIT_0, 1000, config.motor0A, config.motor0B);
   motorInit(MCPWM_UNIT_1, 1000, config.motor1A, config.motor1B);
}
/* ########################## ^^ MOTOR ^^ ############################ */


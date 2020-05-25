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

extern "C" {
#include "wifi.h"
}
#include "motor.h"

volatile int32_t motorOverflow[8] = {0}; /* store over/underflow events for encoders */
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
static void IRAM_ATTR pcnt_example_intr_handler(void *arg);

Motor leftMotor;
Motor rightMotor;


Motor::Motor(){

}

void Motor::initEncoder( pcnt_unit_t pcntUnit, int pinA, int pinB){
   encoderNum = pcntUnit;
   /* Prepare configuration for the PCNT unit */
   pcnt_config_t pcnt_config =
      {
       // Set PCNT input signal and control GPIOs
      pulse_gpio_num : pinA,
      ctrl_gpio_num : pinB,

      // What to do when control input is low or high?
      lctrl_mode : PCNT_MODE_KEEP, // Keep counting direction if low
      hctrl_mode : PCNT_MODE_REVERSE,    // reverse the primary counter mode if high

      // What to do on the positive / negative edge of pulse input?
      pos_mode : PCNT_COUNT_INC,   // Count up on the positive edge
      neg_mode : PCNT_COUNT_DEC,   // Count down on negative edge

      // Set the maximum and minimum limit values to watch
      counter_h_lim : PCNT_H_LIM_VAL,
      counter_l_lim : PCNT_L_LIM_VAL,

      unit : pcntUnit,
      channel : PCNT_CHANNEL_0,
      };
   // pcnt_config_t pcnt_config =
   //    {
   //     // Set PCNT input signal and control GPIOs
   //    pulse_gpio_num : pinA,
   //    ctrl_gpio_num : pinB,
   //    channel : PCNT_CHANNEL_0,
   //    unit : pcntUnit,
   //    // What to do on the positive / negative edge of pulse input?
   //    pos_mode : PCNT_COUNT_INC,   // Count up on the positive edge
   //    neg_mode : PCNT_COUNT_DEC,   // Count down on negative edge
   //    // What to do when control input is low or high?
   //    lctrl_mode : PCNT_MODE_KEEP, // Keep counting direction if low
   //    hctrl_mode : PCNT_MODE_REVERSE,    // reverse the primary counter mode if high
   //    // Set the maximum and minimum limit values to watch
   //    counter_h_lim : PCNT_H_LIM_VAL,
   //    counter_l_lim : PCNT_L_LIM_VAL,
   //    };
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

/* Initializes the gpio and timer needed for controlling a DC motor.
   mcpwmUnit should be 0 or 1, freq is in Hz (~100-~2000), and then the gpio pins */
void Motor::initPWM( mcpwm_unit_t mcpwmUnit, uint32_t freq, int pinA, int pinB){
   mcpwmNum = mcpwmUnit;
   
   /* Config MCPWM peripheral */
   mcpwm_config_t pwm_config;
   pwm_config.frequency = freq;
   pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
   pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
   
   mcpwm_gpio_init(mcpwmUnit, MCPWM0A, pinA);
   mcpwm_gpio_init(mcpwmUnit, MCPWM0B, pinB);
   mcpwm_init(mcpwmUnit, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

}


/* Set speed of the motor to a duty cycle cycle specified from -100 to +100) */
void Motor::setSpeed(float duty_cycle) {
   if(duty_cycle >= 0) {	/* forward */
      mcpwm_set_signal_low(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B);
      mcpwm_set_duty(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
      mcpwm_set_duty_type(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

   } else {			/* backward */
      mcpwm_set_signal_low(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A);
      mcpwm_set_duty(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B, -1*duty_cycle);
      mcpwm_set_duty_type(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
	 
   }
}

/* Returns the encoder count for specified pulse counter,
   incorporating the over/underflow to get a 32 bit num*/
int32_t Motor::position() {
   int16_t count = 0;
   pcnt_get_counter_value(encoderNum, &count);
   return (PCNT_H_LIM_VAL * motorOverflow[encoderNum]) + count;
}








/* Decode what PCNT's unit originated an interrupt
 * and updates the overflow array
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
   uint32_t intr_status = PCNT.int_st.val;
   int i;
   portBASE_TYPE HPTaskAwoken = pdFALSE;

   for (i = 0; i < PCNT_UNIT_MAX; i++) {
      if (intr_status & (BIT(i))) {

	 PCNT.int_clr.val = BIT(i);
	 if (PCNT.status_unit[i].val & PCNT_STATUS_L_LIM_M) {
	    motorOverflow[i]--;
	 }
	 if (PCNT.status_unit[i].val & PCNT_STATUS_H_LIM_M) {
	    motorOverflow[i]++;
	 }

	 if (HPTaskAwoken == pdTRUE) {
	    portYIELD_FROM_ISR();
	 }
      }
   }
}









/* ############################# MOTOR ############################### */


/* 
   given a control structure (which will be unique for each device so it can store history),
   and the set point, and current position, this function calculates output (from -100 to 100)
   
   * Assumes called at constant interval

   TODO: add feedforward for gravity
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
   control_config_t ctrl0 = { .p=0.2, .i=0.0, .d=0.0, .previousError=0, .integral=0};
   control_config_t ctrl1 = { .p=0.2, .i=0.0, .d=0.0, .previousError=0, .integral=0};

   TickType_t xLastWakeTime;
   xLastWakeTime = xTaskGetTickCount ();

   /* we start with nothing in queue, and so we do nothing.
      once we do have something, we just continue with it until it is done
      and if there is nothing else in the queue, then we maintain last thing
      updateSetpoints is required to return end point if called beyond initial timeframe
   */
   char debugBuf[256] = "  \n";
   wsDebug("hello\n");
   wsRegisterVariable( &setPoint0, 'l', "setPoint0");
   wsRegisterVariable( &setPoint1, 'l', "setPoint1");

   wsRegisterVariable( &(ctrl0.p), 'f', "ctrl0P");
   wsRegisterVariable( &(ctrl1.p), 'f', "ctrl1P");

   wsRegisterVariable( &(ctrl0.i), 'f', "ctrl0I");
   wsRegisterVariable( &(ctrl1.i), 'f', "ctrl1I");

   wsRegisterVariable( &(ctrl0.d), 'f', "ctrl0D");
   wsRegisterVariable( &(ctrl1.d), 'f', "ctrl1D");


   while (1) {

      leftMotor.setSpeed( pid(ctrl0, setPoint0, leftMotor.position()) );
      rightMotor.setSpeed( pid(ctrl1, setPoint1, rightMotor.position()) );

      sprintf(debugBuf, "d setPoint0 %d, setPoint1 %d, leftPos %d, rightPos %d\n", setPoint0, setPoint1, leftMotor.position(), rightMotor.position() );
      wsDebug(debugBuf);
      vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_RATE_MS);
   }
}


void initMotors( motor_config_t config){

   leftMotor.initEncoder(PCNT_UNIT_1, config.encoder1A, config.encoder1B);
   rightMotor.initEncoder( PCNT_UNIT_0, config.encoder0A, config.encoder0B);
   leftMotor.initPWM(MCPWM_UNIT_1, 500, config.motor1A, config.motor1B);
   rightMotor.initPWM( MCPWM_UNIT_0, 500, config.motor0A, config.motor0B);
}
/* ########################## ^^ MOTOR ^^ ############################ */


#include "motor.h"
#include "Arduino.h"
// class for controling a motor using esp32 pcnt, mcpwm, and freeRTOS


// Used to keep track of what index to use for the pulse counter (pcnt) and motorController pwm unit (mcpwm)
int Motor::numberOfMotors = 0;
int Motor::velocityUpdateRate = 50000; // Period over which velocity is calculated in uS
int Motor::countsPerOutput = 24*115.5; // number of counts per desired unit (1 rotation, unit distance, etc.)

const int countsPerOutput = 24*115.5; // number of counts per desired unit (1 rotation, unit distance, etc.)

void saveMotor(int i, Motor* m);
void control_loop_task(void *param);


Motor::Motor(int motA, int motB, int encA, int encB, int pwmFreq) {
    int index = numberOfMotors++;
    pcnt_unit_t pcntUnit;
    mcpwm_unit_t mcpwmUnit;
    if(index == 0) {
        pcntUnit = PCNT_UNIT_0;
        mcpwmUnit = MCPWM_UNIT_0;
    } else if(index == 0) {
        pcntUnit = PCNT_UNIT_1;
        mcpwmUnit = MCPWM_UNIT_1;
    } else {
        // TODO: handle error
        printf("ERROR: MOTOR currently supports only 2 instances.\n");
        return;
    }
    saveMotor(index, this);
    pwmPeriod = 1e6 / pwmFreq;  // TODO: this might cause problems at higher frequencies
    // create configs
    configureEncoder(pcntUnit, encA, encB);
    configurePWM(mcpwmUnit, motA, motB, pwmFreq);

    if(index==0) {              // First motor configured
        // start control loop
        xTaskCreatePinnedToCore(control_loop_task, "control Loop", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
    }
}

void Motor::configureEncoder(pcnt_unit_t pcntUnit, int encA, int encB) {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
    pulse_gpio_num : encA,
    ctrl_gpio_num : encB,

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

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);


    /* Use the second channel, switching the control and pulse pins and
       the direction to get full quadrature resolution*/
    pcnt_config.pulse_gpio_num = encB;
    pcnt_config.ctrl_gpio_num = encA;
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


void Motor::configurePWM(mcpwm_unit_t mcpwmUnit, int motA, int motB, int pwmFreq) {
    // Config MCPWM peripheral
    mcpwm_config_t pwm_config;
    pwm_config.frequency = pwmFreq;
    pwm_config.cmpr_a = 0;      // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;      // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Setup GPIO
    mcpwm_gpio_init(mcpwmUnit, MCPWM0A, motA);
    mcpwm_gpio_init(mcpwmUnit, MCPWM0B, motB);

    mcpwm_init(mcpwmUnit, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B with above settings
}



/* Decode what PCNT's unit originated an interrupt
 * and updates the overflow array.
 * If you want to use the rest of the pcnt units for other things, you need to modify this function
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg) {
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


/* Set duty cycle of the motor to a duty cycle cycle specified from -100 to +100) */
void Motor::setDuty(float duty_cycle) {
    if(duty_cycle >= 0) {        /* forward */
        mcpwm_set_signal_low(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
        // call this each time, if operator was previously in low/high state
        mcpwm_set_duty_type(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    } else {                     /* backward */
        mcpwm_set_signal_low(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B, -1*duty_cycle);
        // call this each time, if operator was previously in low/high state
        mcpwm_set_duty_type(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

// Set velocity of motor
void Motor::setVelocity(float velocity) {
    // TODO: set mode to velocity
    velocitySetPoint = velocity * countsPerOutput * velocityUpdateRate/1e6;
    // targetVelocity[0] = velocity * countsPerOutput * velocityUpdateRate/1e6;
}

void Motor::testControl(float kp) {
    // velocity_KP[0] = kp;
    // setDuty((kp*(velocitySetPoint - velocity)));
}

float Motor::getPosition() {
    return  (float)position() / countsPerOutput;
}


/* Returns the encoder count for specified pulse counter,
   incorporating the over/underflow to get a 32 bit num*/
int32_t Motor::position() {
    int16_t count = 0;
    pcnt_get_counter_value((pcnt_unit_t)pcntUnit, &count);
    return (PCNT_H_LIM_VAL * motorOverflow[pcntUnit]) + count;
}
float Motor::getVelocity() {
    // return velocity[0];         // TODO fix
    return velocity * 1e6/velocityUpdateRate / countsPerOutput;
}

void Motor::controlLoop() {
    int32_t curPosition = position();
    velocity = curPosition - previousPosition;
    previousPosition = curPosition;
    setDuty(20 * (velocitySetPoint - getVelocity()));
 }


///////////////////////////////////////////////////////////
//////////////////// C style Functions ////////////////////
///////////////////////////////////////////////////////////

void IRAM_ATTR velocityControlLoop(uint32_t curTime) {

    // int32_t position0 = getMotorPosition(PCNT_UNIT_0);
    // int32_t position1 = getMotorPosition(PCNT_UNIT_1);

    // velocity[0] = position0 - previousPosition[0];
    // velocity[1] = position1 - previousPosition[1];

    // // TODO: add flag to indicate which mode to be in ('current', velocity, position, etc.)
    // float duty = velocity_KP[0] * (targetVelocity[0] - velocity[0]) + // Proportional
    //     ( (1e6/50000 /(24*115.5)) *targetVelocity[0] * 100/1.8); // Feedforward
    // setMotorDuty(MCPWM_UNIT_0, duty);

    // previousPosition[0] = position0;
    // previousPosition[1] = position1;
}







static SemaphoreHandle_t timer_sem;
// hw_timer_t * timer = NULL;




void IRAM_ATTR onTimer(void * args){
   static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
   if( xHigherPriorityTaskWoken) {
       portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
   }
}

// TODO: switch to new API? I can't find documentation on these functions
void initISRTimer(uint64_t period_us) {

   // timer_isr_register(timer_group, timer_idx, timer_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);




    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &onTimer,
        /* name is optional, but may help identify the timer when debugging */
        // .name = "motor Control"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    /* Start the timers */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5000));

  // // Use 1st timer of 4 (counted from zero).
  // // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // // info).
  // timer = timerBegin(0, 80, true);

  // // Attach onTimer function to our timer.
  // timerAttachInterrupt(timer, &onTimer, true);

  // // Set alarm to call onTimer function every  (value in microseconds).
  // // Repeat the alarm (third parameter)
  // timerAlarmWrite(timer, period_us, true);

  // // Start an alarm
  // timerAlarmEnable(timer);
}


Motor* motors[2];
void saveMotor(int i, Motor* m) {
    motors[i] = m;
}

void control_loop_task(void *param)
{
    timer_sem = xSemaphoreCreateBinary();

    initISRTimer(2000);     // control loop period, in uS

    uint32_t time_since_boot = micros();
    int i = 0;
    while (1) {
        xSemaphoreTake(timer_sem, portMAX_DELAY);

        // Do Work here
        for (int i = 0; i < Motor::numberOfMotors; i++) {
            // Call motor[i] control loop
            motors[i]->controlLoop();
        }
        printf("%d\n", (micros() - 5000*i++) - time_since_boot);


    }
}

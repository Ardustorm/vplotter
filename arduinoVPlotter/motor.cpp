#include "motor.h"

// class for controling a motor using esp32 pcnt, mcpwm, and freeRTOS


// Used to keep track of what index to use for the pulse counter (pcnt) and motorController pwm unit (mcpwm)
int Motor::numberOfMotors = 0;
int Motor::velocityUpdateRate = 50000; // Period over which velocity is calculated in uS
int Motor::countsPerOutput = 24*115.5; // number of counts per desired unit (1 rotation, unit distance, etc.)

const int countsPerOutput = 24*115.5; // number of counts per desired unit (1 rotation, unit distance, etc.)
volatile int32_t previousPosition[2] = {0}; // stores previous position for velocity calculations
volatile int32_t velocity[2] = {0};         // velocity measured in ISR
volatile float velocity_KP[2] = {0};         // velocity measured in ISR
volatile int32_t targetVelocity[2] = {0};         // velocity goal

uint32_t cp0_regs[18];    // FPU enable code from: esp32.com/viewtopic.php?t=1292#p5936

int32_t IRAM_ATTR getMotorPosition(int pcntUnit);
void IRAM_ATTR setMotorDuty( mcpwm_unit_t mcpwmNum, float duty_cycle);

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
        return;
    }

    pwmPeriod = 1e6 / pwmFreq;
    // create configs
    configureEncoder(pcntUnit, encA, encB);
    configurePWM(mcpwmUnit, motA, motB, pwmFreq);
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


/* Set duty cycle of the motor to a duty cycle cycle specified from -100 to +100) */
void Motor::setDuty(float duty_cycle) {
    setMotorDuty(mcpwmNum, duty_cycle);
}

// Set velocity of motor
void Motor::setVelocity(float velocity) {
    // TODO: set mode to velocity
    velocitySetPoint = velocity * countsPerOutput * velocityUpdateRate/1e6;
    targetVelocity[0] = velocity * countsPerOutput * velocityUpdateRate/1e6;
}

void Motor::testControl(float kp) {
    velocity_KP[0] = kp;
    // setDuty((kp*(velocitySetPoint - velocity)));
}

float Motor::getPosition() {
    return  (float)position() / countsPerOutput;
}


/* Returns the encoder count for specified pulse counter,
   incorporating the over/underflow to get a 32 bit num*/
int32_t IRAM_ATTR Motor::position() {
    return getMotorPosition((int)pcntUnit);
}
float Motor::getVelocity() {
    // return velocity[0];         // TODO fix
    return velocity[0] * 1e6/velocityUpdateRate / countsPerOutput;
}


///////////////////////////////////////////////////////////
//////////////////// C style Functions ////////////////////
///////////////////////////////////////////////////////////

void IRAM_ATTR setMotorDuty( mcpwm_unit_t mcpwmNum, float duty_cycle) {
    if(duty_cycle >= 0) {        /* forward */
        mcpwm_set_signal_low(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty_type(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

    } else {                     /* backward */
        mcpwm_set_signal_low(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B, -1*duty_cycle);
        mcpwm_set_duty_type(mcpwmNum, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }


    // // Only goes in 1 direction for now
    // mcpwm_timer_t timer_num = MCPWM_TIMER_0;
    // mcpwm_operator_t op_num = MCPWM_OPR_A;
    // uint32_t set_duty;

    // if(duty >= 0) {
    //     set_duty = (MCPWM0[mcpwm_num]->timer[timer_num].period.period) * (duty) / 100;
    //     MCPWM0[mcpwm_num]->channel[timer_num].cmpr_value[op_num].cmpr_val = set_duty;
    //     MCPWM0[mcpwm_num]->channel[timer_num].cmpr_cfg.a_upmethod = BIT(0);
    //     MCPWM0[mcpwm_num]->channel[timer_num].cmpr_cfg.b_upmethod = BIT(0);
    // } else {

    // }

    // mcpwm_hal_context_t *hal = &context[mcpwm_num].hal;





}
void oldSetMotorDuty (mcpwm_unit_t mcpwmNum, float duty_cycle) {
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


/* Returns the encoder count for specified pulse counter,
   incorporating the over/underflow to get a 32 bit num*/
// int32_t IRAM_ATTR getPosition(pcnt_unit_t pcntUnit) {
int32_t IRAM_ATTR getMotorPosition(int pcntUnit) {
   int16_t count = 0;
   // pcnt_get_counter_value((pcnt_unit_t)pcntUnit, &count);
   // return (PCNT_H_LIM_VAL * motorOverflow[pcntUnit]) + count;

   count = (int16_t) PCNT.cnt_unit[pcntUnit].cnt_val;

   return(PCNT_H_LIM_VAL * motorOverflow[pcntUnit]) + count;
}


void IRAM_ATTR velocityControlLoop(uint32_t curTime) {
    // // Save and enable the FPU if needed
    // get FPU state
    uint32_t cp_state = xthal_get_cpenable();

    if(cp_state) {
        // Save FPU registers
        xthal_save_cp0(cp0_regs);
    } else {
        // enable FPU
        xthal_set_cpenable(1);
    }

    int32_t position0 = getMotorPosition(PCNT_UNIT_0);
    int32_t position1 = getMotorPosition(PCNT_UNIT_1);

    velocity[0] = position0 - previousPosition[0];
    velocity[1] = position1 - previousPosition[1];


    float duty = velocity_KP[0] * (targetVelocity[0] - velocity[0]) + // Proportional
        ( (1e6/50000 /(24*115.5)) *targetVelocity[0] * 100/1.8); // Feedforward
    setMotorDuty(MCPWM_UNIT_0, duty);

    previousPosition[0] = position0;
    previousPosition[1] = position1;




    if(cp_state) {
        // Restore FPU registers
        xthal_restore_cp0(cp0_regs);
    } else {
        // turn it back off
        xthal_set_cpenable(0);
    }
}

#ifndef __MOTOR_H__
#define __MOTOR_H__


// #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"


#define PCNT_H_LIM_VAL      ((int16_t) 0x7FFF)
#define PCNT_L_LIM_VAL      ((int16_t) 0x8000)



volatile static int32_t motorOverflow[8] = {0}; /* store over/underflow events for encoders */
static pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
static void IRAM_ATTR pcnt_example_intr_handler(void *arg);



class Motor{
 public:
    Motor(int motA, int motB, int encA, int encB, int pwmFreq);
    void configureEncoder(pcnt_unit_t pcntUnit, int encA, int encB);
    void configurePWM(mcpwm_unit_t mcpwmUnit, int motA, int motB, int pwmFreq);
    void setDuty(float duty_cycle);     // from -100 to +100
    int32_t position();                 // returns position in counts
 private:
    pcnt_unit_t encoderNum;     // Which encoder this is using
    mcpwm_unit_t mcpwmNum;      // Which motor controller this is using

    static int numberOfMotors;  // Indexes which pcnt/mcpwm to use

};



#endif /* __MOTOR_H__ */

/*
 * Global_Functions.h
 *
 *  Created on: 2025 Jan 22
 *      Author:
 */

#ifndef INCLUDE_GLOBAL_FUNCTIONS_H_
#define INCLUDE_GLOBAL_FUNCTIONS_H_

#define PWM_DB 200

extern void pwm_freq_set(float freq);
extern void pwm_output_disable(void);
extern void pwm_output_enable(void);
extern void can_transmit_data(void);
extern void ADC_ISR(void);
extern void Main2msMotor(void);
#endif /* INCLUDE_GLOBAL_FUNCTIONS_H_ */

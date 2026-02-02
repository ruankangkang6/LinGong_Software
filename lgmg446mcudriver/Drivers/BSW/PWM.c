#include "main.h"
#include "Global_Variables.h"
//#include "Global_Functions.h"

void pwm_freq_set(float freq);
void PWM_enable(void);
void PWM_disable(void);
void TIM_SetChannalCompare(TIM_TypeDef* TIMx, uint16_t channel, uint16_t Compare);

void pwm_freq_set(float freq)
{
	float tmr_set=0;
	tmr_set = 90000000/freq;
	TIM1->ARR = tmr_set;
	TIM8->ARR = (tmr_set*2);
}
void TIM_SetChannalCompare(TIM_TypeDef* TIMx, uint16_t channel, uint16_t Compare)
{
	assert_param(IS_TIM_LIST6_PERIPH(TIMX));
	if(channel == 1)
	{
		TIMx->CCR1 = Compare;
	}
	else if(channel == 2)
	{
		TIMx->CCR2 = Compare;
	}
	else if(channel == 3)
	{
		TIMx->CCR3 = Compare;
	}
	else
	{
		;
	}
}

void PWM_disable(void)
{
	TIM1->CCER = TIM1->CCER&0xFAAA;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void PWM_enable(void)
{

	TIM1->CCER = TIM1->CCER|0x0555;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


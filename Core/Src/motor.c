//
// Created by Laity on 2026/10/15.
//


#include "stm32f1xx_hal.h"
#include "tim.h"

void Motor_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void Motor_open(float speed)
{
    if(speed>100){speed=100;}
    if(speed<-100){speed=-100;}

    if(speed>0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);//zhengzhuang
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else if(speed<0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//fanzhuang
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        speed = -speed;   //占空比只能给正值
    }
    else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (unsigned int)speed);
}

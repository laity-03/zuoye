#include "stm32f1xx_hal.h"
#include "tim.h"

void Encoder_Init(void)
{

    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2,0);

}

// int16_t read_encoder(void)
// {
//     return ((int16_t)__HAL_TIM_GET_COUNTER(&htim2));
// }

float get_speed(void)
{
    int16_t last_wz=0;
    int16_t last_time=0;

    int16_t now_wz=(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    float diff=now_wz-last_wz;
    last_time=now_wz;

    float one_quan=13*4*48;

    float count_quan=diff/one_quan;
    float rpm=count_quan/(10/1000.0f)*60.0f;

    return rpm;
}

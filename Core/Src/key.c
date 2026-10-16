#include "stm32f1xx_hal.h"
//
// Created by Laity on 2026/10/15.
//
#include "key.h"

uint8_t key_read(void)
{
    uint8_t a = 0;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) { a = 1; }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) { a = 2; }
    return a;
}

int key_scan(void)
{
    static uint8_t  last_state = 0;
    static uint8_t  now_state  = 0;
    static uint32_t last_time  = 0;
    static uint8_t  down_state = 0;
    static uint32_t time = 0;         //按下时刻

    uint8_t  raw  = key_read();
    uint32_t tick = HAL_GetTick();


    if (raw != last_state)
    {
        last_state = raw;
        last_time  = tick;
    }
    else if (tick - last_time >= 10)
    {
        now_state = last_state;
    }

//变化了就开始判断
    if (now_state != down_state)
    {
        if (now_state != 0)
        {
            down_state = now_state;
            time = tick;
        }
        else
        {
            if (down_state == 1)
            {
                down_state = 0;
                if (tick - time >= 800)
                {
                    return 4;
                }
                return 1;
            }
            if (down_state == 2)
            {
                down_state = 0;
                if (tick - time >= 800)
                {
                    return 3;
                }
                return 2;
            }
            down_state = 0;
        }
    }

    return 0;
}

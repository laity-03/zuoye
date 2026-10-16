/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "ins.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float key_ang[4] = {0, 10, 30, 60};
static float key_spd[7] = {0, 30, 60, 100, -30, -60, -100};

static int     ang_i = 0;
static int     spd_i = 0;
static uint8_t mode  = 0;

static float a_sum  = 0.0f;
static float a_last = 0.0f;
static float s_sum  = 0.0f;
static float s_last = 0.0f;

static float ang_zero = 0.0f;                    //上电零点
static float ang_now  = 0.0f;                    //实际角度
static float ang_tar  = 0.0f;                    //目标角度(°)
static float spd_now  = 0.0f;                    //实际转速
static float spd_tar  = 0.0f;                    //目标转速
static float pwm_out  = 0.0f;                    //PWM 输出

static uint8_t  rx_byte  = 0;                    //串口刚收到的字节
static char     rx_buf[16];                      //VOFA 下命令
static uint8_t  rx_len   = 0;                    //已有字节数
static volatile uint8_t rx_ready = 0;            //收到一整行再处理
static uint32_t vofa_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float get_angle(void);
static float angle_pid(float target, float now, float dt);
static float speed_pid(float target, float now, float dt);
static void  pid_reset(void);
static float wrap180(float a);
static void  zroe_angle(void);
static void  show(void);
static void  vofa_send(void);
static void  vofa_cmd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  int key = 0;
  uint32_t ctrl_tick = 0;
  uint32_t show_tick = 0;

  Encoder_Init();
  Motor_Init();

  HAL_Delay(20);
  OLED_Init();
  OLED_NewFrame();
  OLED_PrintASCIIString(0, 24, "I AM BACK", &afont12x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();

  ins_init();
  zroe_angle();

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  ctrl_tick = HAL_GetTick();
  show_tick = ctrl_tick;
  vofa_tick = ctrl_tick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//按键
    key = key_scan();

    if (key == 1)
    {
        mode = 1;
        spd_i = (spd_i + 1) % 7;
        spd_tar = key_spd[spd_i];
        pid_reset();
    }
    else if (key == 4)
    {
        mode = 0;
        pid_reset();
    }
    else if (key == 2)
    {
        mode = 0;
        ang_i = (ang_i + 1) % 4;
        ang_tar = key_ang[ang_i];
        pid_reset();
    }
    else if (key == 3)
    {
        mode = 0;
        ang_i = 0;
        ang_tar = key_ang[0];
        pid_reset();
    }

    uint32_t tick = HAL_GetTick();
    if (tick - ctrl_tick >= 10)
    {
        float dt = (tick - ctrl_tick) / 1000.0f;

        ctrl_tick = tick;


        ins_update();
        ang_now = get_angle();
        spd_now = get_speed();

        if (mode == 0)
        {
            //速度内环没有加，我看电机上下乱晃，好像积分到 ±100，
            //角度环自己带积分和微分，o了。
            pwm_out = angle_pid(ang_tar, ang_now, dt);
            spd_tar = 0.0f;
            Motor_open(pwm_out);
        }
        else
        {
            pwm_out = speed_pid(spd_tar, spd_now, dt);
            Motor_open(pwm_out);
        }
    }

    vofa_cmd();

    if (tick - vofa_tick >= 50)
    {
        vofa_tick = tick;
        vofa_send();
    }

    if (tick - show_tick >= 200)
    {
        show_tick = tick;
        show();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


static void vofa_send(void)
{
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    float data[5];

    data[0] = ang_tar;      //目标角度 
    data[1] = ang_now;      //实际角度
    data[2] = spd_tar;      //目标速度
    data[3] = spd_now;      //实际速度
    data[4] = pwm_out;      //PWM

    HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), 100);
    HAL_UART_Transmit(&huart1, (uint8_t *)tail, sizeof(tail), 100);
}

//VOFA
static void vofa_cmd(void)
{
    char  c;
    char *p;

    if (rx_ready == 0)
    {
        return;
    }

    c = rx_buf[0];
    if (c >= 'a' && c <= 'z')
    {
        c = (char)(c - 'a' + 'A');
    }

    p = rx_buf + 1;
    if (*p == '=' || *p == ':')
    {
        p++;
    }

    if (c == 'A' && *p != '\0')
    {
        ang_tar = (float)atof(p);
        if (ang_tar > 75.0f)  { ang_tar = 75.0f; }
        if (ang_tar < -75.0f) { ang_tar = -75.0f; }
        mode = 0;
        pid_reset();
    }
    else if (c == 'Z')
    {
        ang_zero = ins_data()->roll;
        pid_reset();
    }

    rx_ready = 0;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            if (rx_len > 0)
            {
                rx_buf[rx_len] = '\0';
                rx_ready = 1;
            }
            rx_len = 0;
        }
        else if (rx_ready == 0 && rx_len < sizeof(rx_buf) - 1)
        {
            rx_buf[rx_len] = (char)rx_byte;
            rx_len++;
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}


static float get_angle(void)
{
    return wrap180(ins_data()->roll - ang_zero);
}


static float wrap180(float a)
{
    while (a > 180.0f)  { a = a - 360.0f; }
    while (a < -180.0f) { a = a + 360.0f; }
    return a;
}


static void zroe_angle(void)
{
    int i;

    for (i = 0; i < 100; i++)
    {
        ins_update();
        HAL_Delay(10);
    }

    ang_zero = ins_data()->roll;

    ang_i   = 0;
    spd_i   = 0;
    mode    = 0;
    ang_tar = key_ang[0];
    ang_now = 0.0f;
    spd_tar = 0.0f;
    pwm_out = 0.0f;
    pid_reset();
    Motor_open(0.0f);
}

//OLED 四行：模式/目标/实际、误差、转速、PWM
static void show(void)
{
    char line[48];
    int  tgt;
    int  act;
    int  err;
    int  s_tgt;
    int  s_act;

    if (mode == 0)
    {
        tgt = (int)ang_tar;
        act = (int)ang_now;
        err = (int)wrap180(ang_tar - ang_now);
    }
    else
    {
        tgt = (int)spd_tar;
        act = (int)spd_now;
        err = (int)(spd_tar - spd_now);
    }

    s_tgt = (int)spd_tar;
    s_act = (int)spd_now;

    OLED_NewFrame();

    if (mode == 0)
    {
        snprintf(line, sizeof(line), "ANG T:%d A:%d", tgt, act);
    }
    else
    {
        snprintf(line, sizeof(line), "SPD T:%d A:%d", tgt, act);
    }
    OLED_PrintASCIIString(0, 0, line, &afont12x6, OLED_COLOR_NORMAL);

    snprintf(line, sizeof(line), "ERR:%d", err);
    OLED_PrintASCIIString(0, 16, line, &afont12x6, OLED_COLOR_NORMAL);

    snprintf(line, sizeof(line), "SPD:%d/%d", s_tgt, s_act);
    OLED_PrintASCIIString(0, 32, line, &afont12x6, OLED_COLOR_NORMAL);

    snprintf(line, sizeof(line), "PWM:%d", (int)pwm_out);
    OLED_PrintASCIIString(0, 48, line, &afont12x6, OLED_COLOR_NORMAL);

    OLED_ShowFrame();
}


static float angle_pid(float target, float now, float dt)
{
    float err = target - now;
    float out;

    a_sum = a_sum + err * dt;
    if (a_sum > 80.0f)  { a_sum = 80.0f; }
    if (a_sum < -80.0f) { a_sum = -80.0f; }

    out = 3.0f * err + 0.4f * a_sum - 0.07f * (now - a_last) / dt;
    a_last = now;

    if (out > 80.0f)  { out = 80.0f; }
    if (out < -80.0f) { out = -80.0f; }
    return out;
}


static float speed_pid(float target, float now, float dt)
{
    float err = target - now;
    float out;

    s_sum = s_sum + err * dt;
    if (s_sum > 100.0f)  { s_sum = 100.0f; }
    if (s_sum < -100.0f) { s_sum = -100.0f; }

    out = 1.0f * err + 1.5f * s_sum - 0.0f * (now - s_last) / dt;
    s_last = now;

    if (out > 100.0f)  { out = 100.0f; }
    if (out < -100.0f) { out = -100.0f; }
    return out;
}

static void pid_reset(void)
{
    a_sum  = 0.0f;
    s_sum  = 0.0f;
    a_last = get_angle();
    s_last = get_speed();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

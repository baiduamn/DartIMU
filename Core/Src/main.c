/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <delay.h>
#include "SEGGER_RTT.h"
#include <stdlib.h>
#include "TenAxisIMU.h"
#include "w25q128.h"
#include "openmv_parser.h"

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
TenAxisIMU ten_axis_imu;
W25Q128_Device flash_dev;
OpenMV_Parser openmv_parser;
uint8_t openmv_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern volatile uint8_t imu_update_flag;
extern volatile uint32_t nowtime;
extern TIM_HandleTypeDef htim2;
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
  *(int *)0xE000ED14 = *(int *)0xE000ED14 & ~0x18;
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
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  HAL_TIM_Base_Start_IT(&htim2);
  
  __HAL_SPI_ENABLE(&hspi1);
  __HAL_SPI_ENABLE(&hspi2);
  
  delay_init(170);

  SEGGER_RTT_Init();
  SEGGER_RTT_printf(0, "\r\n--- DartIMU OOP System Starting ---\r\n");

  W25Q128_Create(&flash_dev, &hspi2, GPIOB, GPIO_PIN_12);
  if (flash_dev.Init(&flash_dev) == HAL_OK) {
      uint32_t flash_id = flash_dev.ReadID(&flash_dev);
      SEGGER_RTT_printf(0, "W25Q128 Flash Init Successful! ID: 0x%06X\r\n", flash_id);
      
      uint8_t write_buf[32] = "DartIMU Flash test OK 123456";
      uint8_t read_buf[32] = {0};
      SEGGER_RTT_printf(0, "Flash Test: Erasing Sector 0...\r\n");
      flash_dev.EraseSector(&flash_dev, 0);
      
      SEGGER_RTT_printf(0, "Flash Test: Writing test pattern...\r\n");
      flash_dev.Write(&flash_dev, 0, write_buf, sizeof(write_buf));
      
      SEGGER_RTT_printf(0, "Flash Test: Reading back...\r\n");
      flash_dev.Read(&flash_dev, 0, read_buf, sizeof(read_buf));
      
      SEGGER_RTT_printf(0, "Flash Test Verification: %s\r\n", 
                        (strcmp((char*)write_buf, (char*)read_buf) == 0) ? "PASS" : "FAIL");
  } else {
      SEGGER_RTT_printf(0, "W25Q128 Flash Init FAILED!\r\n");
  }

  TenAxisIMU_Create(&ten_axis_imu, &hspi1, GPIOA, GPIO_PIN_4, &hi2c3);
  if (ten_axis_imu.Init(&ten_axis_imu) == HAL_OK) {
      SEGGER_RTT_printf(0, "TenAxisIMU Init Successful!\r\n");
  } else {
      SEGGER_RTT_printf(0, "TenAxisIMU Init FAILED!\r\n");
  }

  OpenMV_Parser_Create(&openmv_parser, &huart1);
  HAL_UART_Receive_IT(&huart1, &openmv_rx_byte, 1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (imu_update_flag) {
        imu_update_flag = 0;
        ten_axis_imu.Update(&ten_axis_imu);
        
        static uint16_t print_cnt = 0;
        if (++print_cnt >= 200) {
            print_cnt = 0;
            float yaw, pitch, roll;
            ten_axis_imu.GetEuler(&ten_axis_imu, &yaw, &pitch, &roll);
            float alt = ten_axis_imu.GetAltitude(&ten_axis_imu);
            
            int16_t yaw_i = (int16_t)(yaw * 10.0f);
            int16_t pitch_i = (int16_t)(pitch * 10.0f);
            int16_t roll_i = (int16_t)(roll * 10.0f);
            int32_t alt_cm = (int32_t)(alt * 100.0f);
            
            OpenMV_Target_t target;
            openmv_parser.GetTarget(&openmv_parser, &target);
            
            SEGGER_RTT_printf(0, "Attitude -> Y:%d.%d P:%d.%d R:%d.%d | Alt:%d.%02dm | P:%dPa T:%dC | Vision -> Det:%d X:%d Y:%d W:%d H:%d\r\n",
                yaw_i / 10, abs(yaw_i % 10),
                pitch_i / 10, abs(pitch_i % 10),
                roll_i / 10, abs(roll_i % 10),
                alt_cm / 100, abs(alt_cm % 100),
                (int)ten_axis_imu.raw_pressure, (int)ten_axis_imu.raw_temperature,
                target.detected, target.x, target.y, target.width, target.height);
        }
    }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        openmv_parser.ParseByte(&openmv_parser, openmv_rx_byte);
        HAL_UART_Receive_IT(&huart1, &openmv_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Clear UART error flags to recover from overrun, noise, framing, or parity errors
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
        
        // Restart interrupt RX
        HAL_UART_Receive_IT(&huart1, &openmv_rx_byte, 1);
    }
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

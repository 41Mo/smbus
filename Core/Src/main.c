/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEV_ADDR 0x0B << 1
enum SMBUS_INQ_T {
  INQ_SMBUS_NONE = 0x00,
  INQ_SMBUS_SPECINFO = 0x1A,
  INQ_SMBUS_VOLTAGE = 0x09,
  INQ_SMBUS_CURRENT = 0x0A,
  INQ_SMBUS_FULL_CHARGE_CAPACITY = 0x10,
  INQ_SMBUS_REMAINING_CAPACITY = 0x0F,
  INQ_SMBUS_TEMP = 0x08,
  INQ_SMBUS_SERIAL = 0x1C,
  INQ_SMBUS_CYCLE_COUNT = 0x17,
  INQ_SMBUS_CELL1_VOLT = 0x3f,  // cell 1
  INQ_SMBUS_CELL2_VOLT = 0x3e,  // cell 2
  INQ_SMBUS_CELL3_VOLT = 0x3d,  // cell 3
  INQ_SMBUS_CELL4_VOLT = 0x3c,  // cell 4
  INQ_SMBUS_CELL5_VOLT = 0x3b,  // cell 5
  INQ_SMBUS_CELL6_VOLT = 0x3a,  // cell 6
  INQ_SMBUS_CELL7_VOLT = 0x39,  // cell 7
  INQ_SMBUS_CELL8_VOLT = 0x38,  // cell 8
  INQ_SMBUS_CELL9_VOLT = 0x37,  // cell 9
  INQ_SMBUS_CELL10_VOLT = 0x36, // cell 10
  INQ_SMBUS_CELL11_VOLT = 0x35, // cell 11
  INQ_SMBUS_CELL12_VOLT = 0x34, // cell 12
} SMBUS_INQ;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if 1
#define DEBUG(fmt, args...)                                                    \
  do {                                                                         \
    printf(fmt, ##args);                                                       \
  } while (0)
#else

#define DEBUG(fmt, args...)
#endif
#ifndef SMBUS
#define SMBUS 1
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SMBUS_HandleTypeDef hsmbus3;

/* USER CODE BEGIN PV */

static uint8_t opcode = 0;
static uint8_t specinfo_answer[] = {0x10, 0x0};
static uint8_t make_reset = 0;
static SMBUS_HandleTypeDef* smbus_handle_ptr = &hsmbus3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_SMBUS_Init(void);
/* USER CODE BEGIN PFP */
void stdio_setup();
void reply_to_master();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if SMBUS
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus,
                            uint8_t TransferDirection, uint16_t AddrMatchCode) {
  if (AddrMatchCode != DEV_ADDR) {
    return;
  }
  if (TransferDirection == SMBUS_DIRECTION_TRANSMIT) {
    HAL_SMBUS_Slave_Receive_IT(smbus_handle_ptr, &opcode, 1, SMBUS_FIRST_FRAME);
  } else {
    reply_to_master();
  }
}

void reply_to_master() {
  uint16_t batt_voltage = 5000; // in millivolts
  uint16_t cell_voltage = 5000; // in millivolts
  int16_t current = 1000;       // in milliamps
  uint16_t zero_answer = 0x00;
  uint16_t capacity_mah = 1500;
  uint16_t remaining_capacity_mah = 1000;
  int16_t temp_canti_kelvin = (273.15 + 30) * 10; // 30 degrees
  uint16_t serial_num = 0x0001;
  uint16_t cycle_count = 1;

  switch (opcode) {
  case INQ_SMBUS_SPECINFO:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, specinfo_answer, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied specinfo\n");
    break;
  case INQ_SMBUS_VOLTAGE:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&batt_voltage, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied batt_voltage\n");
    break;
  case INQ_SMBUS_CURRENT:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&current, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied batt_voltage\n");
    break;
  case INQ_SMBUS_FULL_CHARGE_CAPACITY:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&capacity_mah, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied capacity\n");
    break;
  case INQ_SMBUS_REMAINING_CAPACITY:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&remaining_capacity_mah, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied remaining_capacity\n");
    break;
  case INQ_SMBUS_TEMP:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&temp_canti_kelvin, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied temp\n");
    break;
  case INQ_SMBUS_SERIAL:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&serial_num, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied serial num\n");
    break;
  case INQ_SMBUS_CYCLE_COUNT:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&cycle_count, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied serial num\n");
    break;
  case INQ_SMBUS_CELL1_VOLT:
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&cell_voltage, 2,
                                SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied cell1 volt\n");
    break;
  case INQ_SMBUS_NONE:
    printf("opcode zero should never happen?\n");
    break;
  case INQ_SMBUS_CELL2_VOLT:
  case INQ_SMBUS_CELL3_VOLT:
  case INQ_SMBUS_CELL4_VOLT:
  case INQ_SMBUS_CELL5_VOLT:
  case INQ_SMBUS_CELL6_VOLT:
  case INQ_SMBUS_CELL7_VOLT:
  case INQ_SMBUS_CELL8_VOLT:
  case INQ_SMBUS_CELL9_VOLT:
  case INQ_SMBUS_CELL10_VOLT:
  case INQ_SMBUS_CELL11_VOLT:
  case INQ_SMBUS_CELL12_VOLT:
    // if we unuse cell, answer with zero
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&zero_answer,
                                sizeof(zero_answer), SMBUS_LAST_FRAME_NO_PEC);
    DEBUG("replied to unused cell inq");
    break;
  default:
    // handle unknown opcode
    HAL_SMBUS_Slave_Transmit_IT(smbus_handle_ptr, (uint8_t *)&zero_answer,
                                sizeof(zero_answer), SMBUS_LAST_FRAME_NO_PEC);
    printf("SMBUS inq: %x\n", opcode);
    printf("replied with zeros to %x\n", opcode);
    break;
  }

  opcode = 0;
}

void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
  HAL_SMBUS_EnableListen_IT(smbus_handle_ptr);
  // printf("listen cbk\n");
}

void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus){
    // printf("rx cbk\n");
};

void HAL_SMBUS_AbortCpltCallback(SMBUS_HandleTypeDef *hsmbus){
    // printf("tx cbk\n");
};

void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *SMBUSHandle) {
  uint32_t err = HAL_SMBUS_GetError(SMBUSHandle);
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
  switch (err) {
  case HAL_SMBUS_ERROR_BERR:
  case HAL_SMBUS_ERROR_ARLO:
  case HAL_SMBUS_ERROR_AF:
    printf("af");
    break;
  case HAL_SMBUS_ERROR_OVR:
    printf("ovr");
    break;
  case HAL_SMBUS_ERROR_TIMEOUT:
    printf("timeout");
    return;
  case HAL_SMBUS_ERROR_ALERT:
    printf("alert");
    break;
  case HAL_SMBUS_ERROR_PECERR:
    printf("pec");
    break;
  case HAL_SMBUS_ERROR_NONE:
  default:
    printf("err unknown %ld", err);
    break;
  }
  printf("\n");
  HAL_SMBUS_DisableListen_IT(smbus_handle_ptr);
  make_reset = 1;
}
#endif

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
  MX_I2C3_SMBUS_Init();
  /* USER CODE BEGIN 2 */
  stdio_setup();
#if SMBUS
  HAL_SMBUS_EnableListen_IT(smbus_handle_ptr);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t reset_initiated = 0;
  uint32_t reset_initiated_time = 0;
  uint32_t blink_delay = 1000;
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
#if SMBUS
    if (make_reset) {
      if (!reset_initiated) {
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
        HAL_SMBUS_DeInit(smbus_handle_ptr);
        reset_initiated = 1;
        reset_initiated_time = HAL_GetTick();
        blink_delay = 100;
      }

      if (HAL_GetTick() - reset_initiated_time > 5000) {
        HAL_SMBUS_Init(smbus_handle_ptr);
        HAL_SMBUS_EnableListen_IT(smbus_handle_ptr);
        reset_initiated = 0;
        make_reset = 0;
        blink_delay = 1000;
      }
    }
#endif
    HAL_Delay(blink_delay);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hsmbus3.Instance = I2C3;
  hsmbus3.Init.ClockSpeed = 100000;
  hsmbus3.Init.OwnAddress1 = 0;
  hsmbus3.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus3.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus3.Init.OwnAddress2 = 0;
  hsmbus3.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus3.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus3.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus3.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  if (HAL_SMBUS_Init(&hsmbus3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  hsmbus3.Init.OwnAddress1 = DEV_ADDR;

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
  ITM_SendChar(ch);
  return ch;
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

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

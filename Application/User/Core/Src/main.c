/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @author  GPM WBL Application Team
 * @brief   This code implements a bidirectional point to point communication.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "my_lpawur.h"
#include "my_mrsubg.h"
#include "stm32_lpm.h"
#include "crc_4wkup_rf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LPAWUR_PAYLOAD_LEN 7
#define LPAWUR_FRAME_LEN 15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPAWUR_Init(void);
static void MX_MRSUBG_Init(void);
static void utils_init(void) {
	COM_InitTypeDef COM_Init = { 0 };

	COM_Init.BaudRate = 115200;
	COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
	COM_Init.WordLength = COM_WORDLENGTH_8B;
	COM_Init.Parity = COM_PARITY_NONE;
	COM_Init.StopBits = COM_STOPBITS_1;
	BSP_COM_Init(COM1, &COM_Init);
	/* USER CODE END APPE_Init_1 */

	BSP_LED_Init(LD2);

	BSP_PB_Init(B1, GPIO_MODE);
	BSP_PB_Init(B2, GPIO_MODE);

	/* Low Power Manager Init */
	UTIL_LPM_Init();
}

void enter_low_power(PowerSaveLevels lv) {
#if (CFG_LPM_SUPPORTED == 1)

	volatile uint32_t dummy[15];
	uint8_t i;

	for (i = 0; i < 10; i++) {
		dummy[i] = 0;
		__asm("NOP");
	}

	switch (lv) {
	case POWER_SAVE_LEVEL_DISABLED:
		/* Not Power Save device is busy */
		return;
		break;
	case POWER_SAVE_LEVEL_SLEEP:
		UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
		UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
		break;
	case POWER_SAVE_LEVEL_DEEPSTOP_TIMER:
		UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
		UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);
		break;
	case POWER_SAVE_LEVEL_DEEPSTOP_NOTIMER:
		UTIL_LPM_SetStopMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
		UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_ENABLE);
		break;
	}

	UTIL_LPM_EnterLowPower();
#endif /* CFG_LPM_SUPPORTED */
}

static void CreateLPAWURFrame(uint8_t *data) {

	/* bit sync */
	for (int i = 0; i < 5; i++)
		data[i] = 0x00;

	/* Frame sync */
	data[5] = 0x99;

	/* Payload */
	data[6] = 0x07;
	data[7] = 0x06;
	data[8] = 0x05;
	data[9] = 0x04;
	data[10] = 0x03;
	data[11] = 0x02;
	data[12] = 0x01;

	/* CRC */
	EvaluateCrc(&data[6]);
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void task_subg_tx(void) {

	MX_MRSUBG_Init();
	utils_init();

	printf("MRSUBG - TX example.\r\n");

	uint8_t lpawur_frame[LPAWUR_FRAME_LEN] = { 0 };
	CreateLPAWURFrame(lpawur_frame);

	while (1) {

		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN11,
		PWR_WUP_FALLEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN0,
		PWR_WUP_FALLEDG);

		uint32_t wakeupPin = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA);

		if (wakeupPin & B2_PIN) {
			BSP_LED_On(LD2);

			printf("Transmitting to LPAWUR: [ ");
			for (uint8_t i = 0; i < LPAWUR_FRAME_LEN; i++) {
				printf("0x%02x ", lpawur_frame[i]);
			}
			printf("]\r\n");

			mrsubg_tx(lpawur_frame, LPAWUR_FRAME_LEN);
			BSP_LED_Off(LD2);
		}

		else if (wakeupPin & B1_PIN) {
			printf("Flash?\r\n");
			while (4)
				;
		}

		enter_low_power(POWER_SAVE_LEVEL_DEEPSTOP_TIMER);
	}
}

void task_lpawur_rx(void) {

	MX_LPAWUR_Init();
	//MX_MRSUBG_Init();
	utils_init();

	printf("LPAWUR - Receiver example.\r\n");

	uint8_t lpawur_data[LPAWUR_PAYLOAD_LEN] = { 100, 101, 102, 103, 104, 105,
			106 };

	while (1) {
		printf("WAKE UP\r\n");

		/* Wakeup source configuration */
		HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN0,
		PWR_WUP_FALLEDG);

		uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

		/* Wakeup on LPAWUR Frame Valid */

		if (wakeupSource & PWR_WAKEUP_LPAWUR) {
			lpawur_disable();
			BSP_LED_On(LD2);
			LPAWUR_Status status = lpawur_recv(lpawur_data, LPAWUR_PAYLOAD_LEN);
			printf("LPAWUR status: 0x%02x\r\n", status);

			if (status != NO_STATUS) {

				printf("LPAWUR data received: [ ");
				for (uint8_t i = 0; i < LPAWUR_PAYLOAD_LEN; i++) {
					printf("0x%02x ", lpawur_data[i]);
				}
				printf("]\r\n");

			}
			BSP_LED_Off(LD2);
			lpawur_enable();
		}

		wakeupSource = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA);
		if (wakeupSource & B1_PIN) {
			printf("GPIO wakeup for flashing\r\n");
			while (4)
				;
		}
		lpawur_enable();
		enter_low_power(POWER_SAVE_LEVEL_DEEPSTOP_TIMER);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	// task_lpawur_rx();
	task_subg_tx();
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}/** Configure the SYSCLKSource and SYSCLKDivider
	 */
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;
	PeriphClkInitStruct.KRMRateMultiplier = 4;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Radio Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPAWUR_Init(void) {

	SLPAWUR_RFConfig LPAWUR_RadioInitStruct = LPAWUR_DEFAULT_CFG()
	;
	SLPAWUR_FrameInit LPAWUR_FrameInitStruct = LPAWUR_DEFAULT_FRAME_CFG()
	;

	lpawur_init(&LPAWUR_RadioInitStruct);
	lpawur_frame_init(&LPAWUR_FrameInitStruct);

}

static void MX_MRSUBG_Init(void) {

	SMRSubGConfig MRSUBG_RadioInitStruct = MRSUBG_DEFAULT_LPAWUR_CFG()
	;
	MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct =
			MRSUBG_DEFAULT_LPAWUR_FRAME_CFG()
	;

	mrsubg_init(&MRSUBG_RadioInitStruct);
	mrsubg_frame_init(&MRSUBG_PacketSettingsStruct);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


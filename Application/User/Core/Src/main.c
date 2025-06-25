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
#include <stdbool.h>
#include "my_lpawur.h"
#include "my_mrsubg.h"
#include "stm32_lpm.h"
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
static void MX_LPAWUR_Init(uint8_t fs_high, uint8_t fs_low, bool fs_16bit);
static void MX_MRSUBG_Init(void);
// init mrsubg for talking to lpawur
static void MX_MRSUBG_Init_LPAWUR(void);
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
	BSP_LED_Init(LD3);

	BSP_PB_Init(B1, GPIO_MODE);
	BSP_PB_Init(B2, GPIO_MODE);
	//BSP_PB_Init(B3, GPIO_MODE);

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

/* USER CODE BEGIN PFP */

const uint8_t fs_high = 0x11;
const uint8_t fs_low = 0x55;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint16_t manchester_encode(uint8_t in) {
	uint16_t ret = 0;
	for (int i = 0; i < 8 * sizeof(in); ++i) {
		if ((in >> i) & 0x1) {
			ret |= (1 << (2 * i + 1));
		} else {
			ret |= (1 << (2 * i));
		}
	}

	return ret;
}

static void wakeup_frame_create(uint8_t *lpawur_frame, uint8_t fs_high,
		uint8_t fs_low, bool fs_16bit, const uint8_t *data, size_t sz) {

	/* bit sync */
	memset(&lpawur_frame[0], 0, 5);

	uint8_t idx = 5;

	/* Frame sync */
	if (fs_16bit) {
		lpawur_frame[idx++] = fs_high;
		lpawur_frame[idx++] = fs_low;
	} else {
		lpawur_frame[idx++] = fs_low;
	}

	/* Payload */
	memcpy(&lpawur_frame[idx], data, sz);
	idx += sz;

	/* CRC */
	uint16_t crc = EvaluateCrc(data);
	lpawur_frame[idx++] = crc >> 8;
	lpawur_frame[idx++] = crc & 0xFF;
}

/*
 * tx to wakeup n times
 * B1 for falsh
 * B2 for sending msgs with 8bit frame_sync
 * B3 for sending msgs with 16bit frame_sync
 */
void task_wakeup_tx(int n) {
	uint8_t lpawur_frame[LPAWUR_FRAME_LEN_MAX];
	size_t lpawur_frame_len = 0;

	uint8_t lpawur_data[LPAWUR_PAYLOAD_LEN] = { 7, 6, 5, 4, 3, 2, 1 };

	printf("TX example Wakeup %i times.\r\n", n);

	while (1) {

		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA,
		PWR_WAKEUP_PIN11 | PWR_WAKEUP_PIN0,
		PWR_WUP_FALLEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTB, PWR_WAKEUP_PIN15,
		PWR_WUP_FALLEDG);

		uint32_t wakeupPin = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA)
				| HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTB);

		if (wakeupPin & (B2_PIN | GPIO_PIN_15)) {

			printf("------------\r\n");

			BSP_LED_On(LD2);

			MX_MRSUBG_Init_LPAWUR();

			if (wakeupPin & B2_PIN) {
				wakeup_frame_create(lpawur_frame, fs_high, fs_low, false,
						lpawur_data,
						LPAWUR_PAYLOAD_LEN);
				lpawur_frame_len = LPAWUR_FRAME_LEN;
			} else {
				wakeup_frame_create(lpawur_frame, fs_high, fs_low, true,
						lpawur_data,
						LPAWUR_PAYLOAD_LEN);
				lpawur_frame_len = LPAWUR_FRAME_LEN + 1;
			}

			printf("Transmitting to LPAWUR: [ ");
			for (uint8_t i = 0; i < lpawur_frame_len; i++) {
				printf("0x%02X ", lpawur_frame[i]);
			}
			printf("]\r\n");

			for (int i = 0; i < n; ++i) {
				mrsubg_send(lpawur_frame, lpawur_frame_len);
				//HAL_Delay(10);
			}

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

/*
 * tx to wakeup + tx to subg
 */
void task_wakeup_tx_subg_tx(void) {

	uint8_t lpawur_frame[LPAWUR_FRAME_LEN_MAX];
	size_t lpawur_frame_len = 0;

	uint8_t lpawur_data[LPAWUR_PAYLOAD_LEN] = { 7, 6, 5, 4, 3, 2, 1 };

	printf("TX example wakeup + TX subg\r\n");
	uint8_t mrsubg_payload[MRSUBG_PAYLOAD_LEN] = { 0xc0, 0xFF, 0xEE, 0x00 };

	while (1) {

		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA,
		PWR_WAKEUP_PIN11 | PWR_WAKEUP_PIN0,
		PWR_WUP_FALLEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTB, PWR_WAKEUP_PIN15,
		PWR_WUP_FALLEDG);

		uint32_t wakeupPin = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA)
				| HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTB);

		if (wakeupPin & (B2_PIN | GPIO_PIN_15)) {

			printf("------------\r\n");

			BSP_LED_On(LD2);

			MX_MRSUBG_Init_LPAWUR();

			if (wakeupPin & B2_PIN) {
				wakeup_frame_create(lpawur_frame, fs_high, fs_low, false,
						lpawur_data,
						LPAWUR_PAYLOAD_LEN);
				lpawur_frame_len = LPAWUR_FRAME_LEN;
			} else {
				wakeup_frame_create(lpawur_frame, fs_high, fs_low, true,
						lpawur_data,
						LPAWUR_PAYLOAD_LEN);
				lpawur_frame_len = LPAWUR_FRAME_LEN + 1;
			}

			printf("Transmitting to LPAWUR: [ ");
			for (uint8_t i = 0; i < lpawur_frame_len; i++) {
				printf("0x%02X ", lpawur_frame[i]);
			}
			printf("]\r\n");
			mrsubg_send(lpawur_frame, lpawur_frame_len);

			HAL_Delay(100);

			MX_MRSUBG_Init();
			printf("Transmitting to MRSUBG: [ ");
			for (uint8_t i = 0; i < MRSUBG_PAYLOAD_LEN; i++) {
				printf("0x%02X ", mrsubg_payload[i]);
			}
			printf("]\r\n");
			mrsubg_send(mrsubg_payload, MRSUBG_PAYLOAD_LEN);

			BSP_LED_Off(LD2);
		}

		else if (wakeupPin & B1_PIN) {
			printf("Flash?\r\n");
			while (4)
				;
		}

		++mrsubg_payload[3];
		enter_low_power(POWER_SAVE_LEVEL_DEEPSTOP_TIMER);
	}
}

/*
 * rx for wakeup
 * B1 for falsh
 * B2 for changing 8/16 bit framesync (red LED means 16 bit)
 * B3 for printing number of received msgs *
 */
void task_lpawur_rx(void) {

	bool sync_16bit = false;

	int recv_pkt_n = 0;

	MX_LPAWUR_Init(fs_high, fs_low, sync_16bit);

	printf("RX example wakeup.\r\n");

	uint8_t lpawur_data[LPAWUR_PAYLOAD_LEN] = { 0 };

	while (1) {
		/* Wakeup source configuration */
		HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA,
		PWR_WAKEUP_PIN0 | PWR_WAKEUP_PIN11,
		PWR_WUP_FALLEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTB, PWR_WAKEUP_PIN15,
		PWR_WUP_FALLEDG);

		uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

		/* Wakeup on LPAWUR Frame Valid */

		if (wakeupSource & PWR_WAKEUP_LPAWUR) {

			//printf("------------\r\n");

			lpawur_disable();
			BSP_LED_On(LD2);
			LPAWUR_Status status = lpawur_recv(lpawur_data, LPAWUR_PAYLOAD_LEN);
			//printf("LPAWUR status: 0x%02x\r\n", status);

			if (status != NO_STATUS) {

				/*printf("LPAWUR data received: [ ");
				 for (uint8_t i = 0; i < LPAWUR_PAYLOAD_LEN; i++) {
				 printf("0x%02X ", lpawur_data[i]);
				 }
				 printf("]\r\n");
				 */
				++recv_pkt_n;
			}

			BSP_LED_Off(LD2);
		}

		wakeupSource = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA)
				| HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTB);
		;
		if (wakeupSource & B1_PIN) {
			printf("GPIO wakeup for flashing\r\n");
			while (4)
				;
		} else if (wakeupSource & GPIO_PIN_15) {
			printf("recv_pkt_n: %i\r\n", recv_pkt_n);
		} else if (wakeupSource & B2_PIN) {
			sync_16bit = !sync_16bit;
			MX_LPAWUR_Init(fs_high, fs_low, sync_16bit);
			printf("Changed sync length to %s bit\r\n",
					sync_16bit ? "16" : "8");
			if (sync_16bit) {
				BSP_LED_On(LD3);
			} else {
				BSP_LED_Off(LD3);
			}
		}

		lpawur_enable();

		enter_low_power(POWER_SAVE_LEVEL_DEEPSTOP_TIMER);
	}
}

/*
 * rx for wakeup + rx for subg
 */
void task_lpawur_rx_subg_rx(void) {

	MX_LPAWUR_Init(fs_high, fs_low, false);
	MX_MRSUBG_Init();

	printf("RX example wakeup + RX subg\r\n");

	uint8_t lpawur_data[LPAWUR_PAYLOAD_LEN] = { 0 };
	uint8_t mrsubg_data[MRSUBG_PAYLOAD_LEN] = { 0 };

	while (1) {
		printf("WAKE UP\r\n");

		/* Wakeup source configuration */
		HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
		HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN0,
		PWR_WUP_FALLEDG);

		uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

		/* Wakeup on LPAWUR Frame Valid */

		if (wakeupSource & PWR_WAKEUP_LPAWUR) {

			printf("------------\r\n");

			lpawur_disable();
			BSP_LED_On(LD2);
			LPAWUR_Status status = lpawur_recv(lpawur_data, LPAWUR_PAYLOAD_LEN);
			printf("LPAWUR status: 0x%02x\r\n", status);

			if (status != NO_STATUS) {

				printf("LPAWUR data received: [ ");
				for (uint8_t i = 0; i < LPAWUR_PAYLOAD_LEN; i++) {
					printf("0x%02X ", lpawur_data[i]);
				}
				printf("]\r\n");

			}

			uint32_t mrsubg_status = mrsubg_recv(mrsubg_data,
			MRSUBG_PAYLOAD_LEN);

			printf("MRSUBG status: 0x%06x\r\n", mrsubg_status);

			if (mrsubg_status & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F) {

				printf("MRSUBG data received: [ ");
				for (uint8_t i = 0; i < MRSUBG_PAYLOAD_LEN; i++) {
					printf("0x%02X ", mrsubg_data[i]);
				}
				printf("]\r\n");

				/* Clear the IRQ flag */
				__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
						MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F);

			} else if (mrsubg_status
					& MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F) {
				printf("CRC Error\r\n");

				/* Clear the IRQ flag */
				__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
						MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F);

			} else if (mrsubg_status
					& MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F) {
				printf("RX Timeout\r\n");

				/* Clear the IRQ flag */
				__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
						MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F);

			}

			BSP_LED_Off(LD2);
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

	utils_init();
	HAL_Delay(100);

	//task_lpawur_rx_subg_rx();
	//task_wakeup_tx_subg_tx();

	// distance test
	//task_wakeup_tx(100);
	task_lpawur_rx();
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
static void MX_LPAWUR_Init(uint8_t fs_high, uint8_t fs_low, bool fs_16bit) {

	SLPAWUR_RFConfig LPAWUR_RadioInitStruct = LPAWUR_DEFAULT_CFG();
	SLPAWUR_FrameInit LPAWUR_FrameInitStruct = LPAWUR_DEFAULT_FRAME_CFG();

	LPAWUR_FrameInitStruct.FrameSyncPatternLow = manchester_encode(fs_low);
	LPAWUR_FrameInitStruct.SyncLength = fs_16bit;

	if (fs_16bit) {
		LPAWUR_FrameInitStruct.FrameSyncPattenHigh = manchester_encode(fs_high);
		LPAWUR_FrameInitStruct.SyncThr = 32;
	}

	lpawur_init(&LPAWUR_RadioInitStruct);
	lpawur_frame_init(&LPAWUR_FrameInitStruct);
	lpawur_wake_up_lvl_set(WAKEUP_FRAME_VALID);

}

static void MX_MRSUBG_Init_LPAWUR(void) {

	SMRSubGConfig MRSUBG_RadioInitStruct = MRSUBG_DEFAULT_WAKEUP_CFG();
	MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct =
			MRSUBG_DEFAULT_WAKEUP_FRAME_CFG();

	//MRSUBG_RadioInitStruct.outputPower = 20;

	if(MRSUBG_RadioInitStruct.outputPower <= 10)
		MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX;
	else if(MRSUBG_RadioInitStruct.outputPower <= 14)
		MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_HP;
	else
		MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_TX_HP;

	mrsubg_init(&MRSUBG_RadioInitStruct);
	mrsubg_frame_init(&MRSUBG_PacketSettingsStruct);
}

static void MX_MRSUBG_Init(void) {
	SMRSubGConfig MRSUBG_RadioInitStruct = MRSUBG_DEFAULT_CFG();
	MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct =
			MRSUBG_DEFAULT_FRAME_CFG();

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

	GPIO_InitTypeDef GPIO_Init;

	GPIO_Init.Pin = GPIO_PIN_15;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init.Mode = GPIO_MODE_INPUT;

	HAL_GPIO_Init(GPIOB, &GPIO_Init);

	if (LL_PWR_IsEnabledPUPDCfg() != 0) {

		LL_PWR_EnableGPIOPullUp( LL_PWR_GPIO_B, GPIO_Init.Pin);
	}

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


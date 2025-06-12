/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_entry.c
 * @author  GPM WBL Application Team
 * @brief   Entry point of the application
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
#include "stm32_lpm.h"
#include "stm32wl3x_ll_usart.h"
#include "stm32wl3x_ll_lpawur.h"
#include "crc_4wkup_rf.h"

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define LPAWUR_PAYLOAD_LEN 7
/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define MIN(a,b)                        (((a) < (b))? (a) : (b))
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define PACKET_LEN 15
uint8_t LPAWUR_Payload[LPAWUR_PAYLOAD_LEN];
static uint8_t payload_memory[PACKET_LEN] = { 0 };

static uint8_t vectcTxBuff[PACKET_LEN];
/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/

/* USER CODE BEGIN GV */

/* USER CODE END GV */

/* Private functions prototypes-----------------------------------------------*/

/* USER CODE BEGIN PFP */

static void CreateLPAWURFrame(uint8_t* data){

  /* bit sync */
  for(int i = 0; i<5; i++)
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

void MX_APPE_Process_TX(void)
{
  /* USER CODE BEGIN MX_APPE_Process_1 */

  /* USER CODE END MX_APPE_Process_1 */

  /* USER CODE BEGIN MX_APPE_Process_2 */

  /* Wakeup source configuration */
  HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN11, PWR_WUP_FALLEDG);
  HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN0, PWR_WUP_FALLEDG);

  uint32_t wakeupPin = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA);

  if (wakeupPin & B2_PIN)
  {
    BSP_LED_On(LD2);

    printf("Transmitting: [ ");

    for(uint8_t i=0 ; i<PACKET_LEN ; i++)
      printf("%d ", vectcTxBuff[i]);
    printf("]\n\r");

    /* Send the TX command */
    __HAL_MRSUBG_STROBE_CMD(CMD_TX);

    /* Wait for TX done */
    while((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {};

    /* Clear the IRQ flag */
    __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);

    BSP_LED_Off(LD2);


    SMRSubGConfig MRSUBG_RadioInitStruct;
    MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct;
    /** Configures the radio parameters
      */
      MRSUBG_RadioInitStruct.lFrequencyBase = 868000000;
      MRSUBG_RadioInitStruct.xModulationSelect = MOD_2FSK;
      MRSUBG_RadioInitStruct.lDatarate = 38400;
      MRSUBG_RadioInitStruct.lFreqDev = 20000;
      MRSUBG_RadioInitStruct.lBandwidth = 100000;
      MRSUBG_RadioInitStruct.dsssExp = 0;
      MRSUBG_RadioInitStruct.outputPower = 14;
      MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_HP;
      HAL_MRSubG_Init(&MRSUBG_RadioInitStruct);

      /** Configures the packet parameters
      */
      MRSUBG_PacketSettingsStruct.PreambleLength = 16;
      MRSUBG_PacketSettingsStruct.PostambleLength = 0;
      MRSUBG_PacketSettingsStruct.SyncLength = 31;
      MRSUBG_PacketSettingsStruct.SyncWord = 0x88888888;
      MRSUBG_PacketSettingsStruct.FixVarLength = FIXED;
      MRSUBG_PacketSettingsStruct.PreambleSequence = PRE_SEQ_0101;
      MRSUBG_PacketSettingsStruct.PostambleSequence = POST_SEQ_0101;
      MRSUBG_PacketSettingsStruct.CrcMode = PKT_CRC_MODE_8BITS;
      MRSUBG_PacketSettingsStruct.Coding = CODING_NONE;
      MRSUBG_PacketSettingsStruct.DataWhitening = ENABLE;
      MRSUBG_PacketSettingsStruct.LengthWidth = BYTE_LEN_1;
      MRSUBG_PacketSettingsStruct.SyncPresent = ENABLE;
      HAL_MRSubG_PacketBasicInit(&MRSUBG_PacketSettingsStruct);

    HAL_Delay(1000);

    BSP_LED_On(LD2);

   printf("Transmitting: [ ");

   for(uint8_t i=0 ; i<PACKET_LEN ; i++)
	 printf("%d ", vectcTxBuff[i]);
   printf("]\n\r");

   /* Send the TX command */
   __HAL_MRSUBG_STROBE_CMD(CMD_TX);

   /* Wait for TX done */
   while((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS() & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {};

   /* Clear the IRQ flag */
   __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);

   BSP_LED_Off(LD2);
  }
  else if (wakeupPin & B1_PIN) {
	  printf("Flash?");
	  while (4){}
  }


  /* USER CODE END MX_APPE_Process_2 */
}

/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/

/* USER CODE BEGIN FD */

/**
 * @brief  Check if UART is BUSY or not.
 * @retval TRUE if UART is BUSY, FALSE otherwise.
 */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void subg_recv() {

	printf("subg_recv\r\n");

	/* Payload length config */
	HAL_MRSubG_PktBasicSetPayloadLength(PACKET_LEN);

	__HAL_MRSUBG_SET_RX_MODE(RX_NORMAL);

	__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t )&payload_memory);

	/* Start RX */
	__HAL_MRSUBG_STROBE_CMD(CMD_RX);

	HAL_Delay(1000);

	uint32_t irq = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

	printf("%i\r\n", irq);

	if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F) {
		/* Clear the IRQ flag */
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
				MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F);

		BSP_LED_On(LD2);

		/* print the received data */
		printf("SUBG RX - Data received: [ ");

		for (uint8_t i = 0; i < PACKET_LEN; i++)
			printf("%u ", payload_memory[i]);

		printf("]\r\n");

		/* Restart RX */
		//__HAL_MRSUBG_STROBE_CMD(CMD_RX);
	} else if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F) {
		printf("CRC Error\r\n");

		/* Clear the IRQ flag */
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
				MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F);

		/* Restart RX */
		// __HAL_MRSUBG_STROBE_CMD(CMD_RX);
	} else if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F) {
		printf("RX Timeout\r\n");

		/* Clear the IRQ flag */
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
				MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F);

		/* Restart RX */
		//__HAL_MRSUBG_STROBE_CMD(CMD_RX);
	}

	BSP_LED_Off(LD2);
}



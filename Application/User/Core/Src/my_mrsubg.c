/*
 * my_mrsubg.c
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */

#include "my_mrsubg.h"

void mrsubg_init(SMRSubGConfig *radio_cfg, MRSubG_PcktBasicFields *frame_cfg) {
	HAL_MRSubG_Init(radio_cfg);
	HAL_MRSubG_PacketBasicInit(frame_cfg);
}

void mrsubg_set_manchester_type(MRSubG_ManchesterPolarity t) {
	LL_MRSubG_PacketHandlerManchesterType(t);
}

void mrsubg_tx(uint8_t *data, size_t sz) {

	HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN11,
			PWR_WUP_FALLEDG);
	HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN0,
			PWR_WUP_FALLEDG);

	uint32_t wakeupPin = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA);

	if (wakeupPin & B2_PIN) {

		HAL_MRSubG_PktBasicSetPayloadLength(sz);

		__HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);

		__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t) data);

		BSP_LED_On(LD2);

		/* Send the TX command */
		__HAL_MRSUBG_STROBE_CMD(CMD_TX);

		/* Wait for TX done */
		while ((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS()
				& MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {
		}
		;

		/* Clear the IRQ flag */
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
				MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);

		BSP_LED_Off(LD2);
	}

	else if (wakeupPin & B1_PIN) {
		  printf("Flash?");
		  while (4){}
	  }
}

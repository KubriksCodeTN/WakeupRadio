/*
 * my_mrsubg.c
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */

#include <assert.h>
#include "my_mrsubg.h"

void mrsubg_init(SMRSubGConfig *radio_cfg) {
	assert(radio_cfg != NULL);
	HAL_MRSubG_Init(radio_cfg);
}

void mrsubg_frame_init(MRSubG_PcktBasicFields *frame_cfg) {
	assert(frame_cfg != NULL);

	HAL_MRSubG_PacketBasicInit(frame_cfg);
}

void mrsubg_set_manchester_type(MRSubG_ManchesterPolarity t) {
	LL_MRSubG_PacketHandlerManchesterType(t);
}

void mrsubg_send(uint8_t *data, size_t sz) {

	HAL_MRSubG_PktBasicSetPayloadLength(sz);

	__HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);

	__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t ) data);

	/* Send the TX command */
	__HAL_MRSUBG_STROBE_CMD(CMD_TX);

	/* Wait for TX done */
	// busy waiting - could be improved with sleep + IRQ
	while ((__HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS()
			& MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) == 0) {
	};

	/* Clear the IRQ flag */
	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
			MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F);
}

uint32_t mrsubg_recv(uint8_t *data, size_t sz) {

	/* Payload length config */
	HAL_MRSubG_PktBasicSetPayloadLength(sz);

	__HAL_MRSUBG_SET_RX_MODE(RX_NORMAL);

	__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t ) data);

	/* Start RX */
	__HAL_MRSUBG_STROBE_CMD(CMD_RX);

	HAL_Delay(100);

	uint32_t irq = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

	if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F) {
		/* Clear the IRQ flag */
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
				MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F);

		/* Restart RX */
		//__HAL_MRSUBG_STROBE_CMD(CMD_RX);
	} else if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F) {

		/* Clear the IRQ flag */
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(
				MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F);

		/* Restart RX */
		// __HAL_MRSUBG_STROBE_CMD(CMD_RX);
	}
	return irq;
}


/*
 * lpawur.c
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */
#include "stm32wl3x_hal.h"

#include <my_lpawur.h>
#include "main.h"
#include "stm32_lpm.h"
#include "stm32wl3x_ll_usart.h"

void lpawur_init(SLPAWUR_RFConfig *radio_cfg) {
	assert(radio_cfg != NULL);
	HAL_LPAWUR_RFConfigInit(radio_cfg);
}

void lpawur_frame_init(SLPAWUR_FrameInit *frame_cfg) {
	assert(frame_cfg != NULL);
	HAL_LPAWUR_FrameInit(frame_cfg);
}

void lpawur_wake_up_lvl_set(WakeUpLevel lv) {
	LL_LPAWUR_SetWakeUpLevel(WAKEUP_FRAME_VALID);
}

void lpawur_enable() {
	LL_LPAWUR_SetState(ENABLE);
}

void lpawur_disable() {
	LL_LPAWUR_SetState(DISABLE);
}

LPAWUR_Status lpawur_recv(uint8_t *data, size_t sz) {
	uint8_t length = LL_LPAWUR_GetPayloadLength();
	if(sz < length){
		return NO_STATUS;
	}

	HAL_LPAWUR_GetPayload(data);

	LPAWUR_Status ret = HAL_LPAWUR_GetStatus();
	HAL_LPAWUR_ClearStatus();

	return ret;
}

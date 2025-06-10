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

int lpawur_init(SLPAWUR_RFConfig *radio_cfg,
		SLPAWUR_FrameInit *frame_cfg) {

	/* USER CODE BEGIN LPAWUR_Init 0 */

	/* USER CODE END LPAWUR_Init 0 */

	/* USER CODE BEGIN LPAWUR_Init 1 */

	/* USER CODE END LPAWUR_Init 1 */

	/** Configures the radio parameters
	 */

	HAL_LPAWUR_RFConfigInit(radio_cfg);

	HAL_LPAWUR_FrameInit(frame_cfg);

	LL_LPAWUR_SetWakeUpLevel(WAKEUP_FRAME_VALID);
	LL_LPAWUR_SetState(ENABLE);

	/* USER CODE BEGIN LPAWUR_Init 2 */

	/* USER CODE END LPAWUR_Init 2 */
}

void lpawur_recv(uint8_t *data, size_t sz) {
	/* USER CODE BEGIN MX_APPE_Process_1 */

	/* USER CODE END MX_APPE_Process_1 */

	/* USER CODE BEGIN MX_APPE_Process_2 */

	/* Wakeup source configuration */
	HAL_PWREx_EnableInternalWakeUpLine(PWR_WAKEUP_LPAWUR, PWR_WUP_RISIEDG);
	HAL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PORTA, PWR_WAKEUP_PIN0,
	PWR_WUP_FALLEDG);

	uint32_t wakeupSource = HAL_PWREx_GetClearInternalWakeUpLine();

	/* Wakeup on LPAWUR Frame Valid */

	if (wakeupSource & PWR_WAKEUP_LPAWUR) {
		BSP_LED_On(LD2);

		HAL_LPAWUR_GetPayload(data);

		HAL_LPAWUR_ClearStatus();

		LL_LPAWUR_SetState(ENABLE);

		BSP_LED_Off(LD2);
	}

	wakeupSource = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA);
	if (wakeupSource & B1_PIN) {
		printf("GPIO wakeup\r\n");
		while (4)
			;
	}

	/* USER CODE END MX_APPE_Process_2 */
}


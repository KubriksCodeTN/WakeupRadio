/*
 * lpawur.h
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */

#ifndef APPLICATION_USER_CORE_INC_LPAWUR_H_
#define APPLICATION_USER_CORE_INC_LPAWUR_H_

#include "stm32wl3x_ll_lpawur.h"

#include <stdlib.h>

int lpawur_init(SLPAWUR_RFConfig* radio_cfg, SLPAWUR_FrameInit* frame_cfg);
void lpawur_recv(uint8_t* data, size_t sz);

#endif /* APPLICATION_USER_CORE_INC_LPAWUR_H_ */

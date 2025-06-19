/*
 * lpawur.h
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */

#ifndef APPLICATION_USER_CORE_INC_LPAWUR_H_
#define APPLICATION_USER_CORE_INC_LPAWUR_H_

#include "stm32wl3x_ll_lpawur.h"
#include "crc_4wkup_rf.h"

#include <stdlib.h>

#define LPAWUR_PAYLOAD_LEN_MAX 8
#define LPAWUR_FRAME_LEN_MAX 17

#define LPAWUR_DEFAULT_CFG() (SLPAWUR_RFConfig){ \
	.EnergyDetectorIcal = ED_ICAL_VBAT_3_25_TO_3_50, \
	.ClockDivider = 7, \
	.EnergyDetectorSwitch = DISABLE, \
	.AgcResetMode = AGC_RESET_MODE_NEVER, \
	.AgcHoldMode = AGC_HOLD_AFTER_PREAMBLE, \
	.AgcMode = AGC_MODE_OFF, \
	.AgcHiLvl = AGC_VBAT_0800, \
	.DCCurrentSubtraction = ENABLE, \
	.AgcLoLvl = AGC_LOW_0, \
};

#define LPAWUR_DEFAULT_FRAME_CFG() (SLPAWUR_FrameInit){ \
	.TRecAlgoSel = TWO_STEPS, \
	.SlowClkCyclePerBitCnt = 16, \
	.PayloadLength = 7, \
	.SyncThr = 16, \
	.SyncLength = 0, \
	.PreambleThrCnt = 0x3C, \
	.PreambleEnable = ENABLE, \
	.FrameSyncCntTimeout = 0x60, \
	.FrameSyncPattenHigh = 0x00, \
	.FrameSyncPatternLow = 0x9696, \
	.KpGain = 6, \
	.KiGain = 10, \
};

void lpawur_init(SLPAWUR_RFConfig* radio_cfg);
void lpawur_frame_init(SLPAWUR_FrameInit* frame_cfg);
void lpawur_wake_up_lvl_set(WakeUpLevel lv);
void lpawur_enable();
void lpawur_disable();

LPAWUR_Status lpawur_recv(uint8_t* data, size_t sz);

#endif /* APPLICATION_USER_CORE_INC_LPAWUR_H_ */

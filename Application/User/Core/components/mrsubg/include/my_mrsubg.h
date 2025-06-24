/*
 * my_mrsubg.h
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */

#ifndef APPLICATION_USER_CORE_INC_MY_MRSUBG_H_
#define APPLICATION_USER_CORE_INC_MY_MRSUBG_H_

#include "stm32wl3x_hal_mrsubg.h"

#define MRSUBG_PAYLOAD_LEN 4

#define MRSUBG_DEFAULT_CFG() (SMRSubGConfig){ \
	.lFrequencyBase = 868000000, \
	.xModulationSelect = MOD_2FSK, \
	.lDatarate = 38400, \
	.lFreqDev = 20000, \
	.lBandwidth = 100000, \
	.dsssExp = 0, \
	.outputPower = 14, \
	.PADrvMode = PA_DRV_TX_HP, \
}

#define MRSUBG_DEFAULT_FRAME_CFG() (MRSubG_PcktBasicFields){ \
	.PreambleLength = 16, \
	.PostambleLength = 0, \
	.SyncLength = 31, \
	.SyncWord = 0x88888888, \
	.FixVarLength = FIXED, \
	.PreambleSequence = PRE_SEQ_0101, \
	.PostambleSequence = POST_SEQ_0101, \
	.CrcMode = PKT_CRC_MODE_8BITS, \
	.Coding = CODING_NONE, \
	.DataWhitening = ENABLE, \
	.LengthWidth = BYTE_LEN_1, \
	.SyncPresent = ENABLE, \
}

#define MRSUBG_DEFAULT_WAKEUP_CFG() (SMRSubGConfig){ \
	.lFrequencyBase = 868000000, \
	.xModulationSelect = MOD_OOK, \
	.lDatarate = 2000, \
	.lFreqDev = 20000, \
	.lBandwidth = 50000, \
	.dsssExp = 0, \
	.outputPower = 14, \
	.PADrvMode = PA_DRV_TX_HP, \
}

#define MRSUBG_DEFAULT_WAKEUP_FRAME_CFG() (MRSubG_PcktBasicFields){ \
	.PreambleLength = 0, \
	.PostambleLength = 0, \
	.SyncLength = 0, \
	.SyncWord = 0x88888888, \
	.FixVarLength = FIXED, \
	.PreambleSequence = PRE_SEQ_0101, \
	.PostambleSequence = POST_SEQ_0101, \
	.CrcMode = PKT_NO_CRC, \
	.Coding = CODING_MANCHESTER, \
	.DataWhitening = DISABLE, \
	.LengthWidth = BYTE_LEN_1, \
	.SyncPresent = DISABLE, \
}

void mrsubg_init(SMRSubGConfig *radio_cfg);
void mrsubg_frame_init(MRSubG_PcktBasicFields *frame_cfg);
void mrsubg_set_manchester_type(MRSubG_ManchesterPolarity t);

void mrsubg_send(uint8_t *data, size_t sz);
uint32_t mrsubg_recv(uint8_t* data, size_t sz);

#endif /* APPLICATION_USER_CORE_INC_MY_MRSUBG_H_ */

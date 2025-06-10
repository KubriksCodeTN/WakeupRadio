/*
 * my_mrsubg.h
 *
 *  Created on: Jun 10, 2025
 *      Author: sdagn
 */

#ifndef APPLICATION_USER_CORE_INC_MY_MRSUBG_H_
#define APPLICATION_USER_CORE_INC_MY_MRSUBG_H_

#include "main.h"

void mrsubg_init(SMRSubGConfig* radio_cfg, MRSubG_PcktBasicFields* frame_cfg);
void mrsubg_set_manchester_type(MRSubG_ManchesterPolarity t);

void mrsubg_tx(uint8_t* data, size_t sz);


#endif /* APPLICATION_USER_CORE_INC_MY_MRSUBG_H_ */

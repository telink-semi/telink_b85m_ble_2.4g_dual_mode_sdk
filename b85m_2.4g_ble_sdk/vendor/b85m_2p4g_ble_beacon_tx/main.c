/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the source file for 8355
 *
 * @author	2.4G Group
 * @date	2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/

#include "drivers.h"
#include "ble_adv.h"

#if(MCU_CORE_TYPE == MCU_CORE_825x)
#define RF_POWER RF_POWER_P0p04dBm
#elif(MCU_CORE_TYPE == MCU_CORE_827x)
#define RF_POWER  RF_POWER_P0p52dBm
#endif

#define CAP_VALUE  0x77000
unsigned char cap;
volatile unsigned char tx_done_flag = 0;
volatile unsigned char tx_cnt = 0;
static unsigned char test_pdu[] = {0x02, 0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                   0x05, 0x09, 0x74, 0x48, 0x49, 0x44, 0x02, 0x01,
                                   0x05};
int main(void)
{
#if(MCU_CORE_TYPE == MCU_CORE_825x)
    cpu_wakeup_init();
#elif(MCU_CORE_TYPE == MCU_CORE_827x)
    cpu_wakeup_init(LDO_MODE,EXTERNAL_XTAL_24M);
#endif

    clock_init(SYS_CLK_24M_Crystal);

	rf_irq_disable(FLD_RF_IRQ_ALL);
	rf_irq_enable(FLD_RF_IRQ_TX);
	irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
	irq_enable(); //enable general irq

    ble_adv_init(37, RF_POWER);

    flash_read_page(CAP_VALUE,1,&cap);
    if(cap!=0xff)
	{
    	rf_update_internal_cap(cap);
	}

    while (1) {
        ble_adv_send(test_pdu, sizeof(test_pdu));
        while(0 == tx_done_flag);
        tx_done_flag = 0;
    }
}

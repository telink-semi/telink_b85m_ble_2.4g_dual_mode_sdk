/********************************************************************************************************
 * @file    main.c
 *
 * @brief   This is the source file for 8355
 *
 * @author  2.4G Group
 * @date    2022
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
#include "stack/2p4g/esb_ll/esb_ll.h"

#define GREEN_LED_PIN           GPIO_PD3
#define RED_LED_PIN             GPIO_PD5

extern volatile unsigned char rx_flag,ds_flag,maxretry_flag;
unsigned char preamble_len = 0;
unsigned int loop_cnt = 0;
unsigned int tmp = 0;

static volatile unsigned char tx_data[32] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
                                             0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,
                                             0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,
                                             0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20};



void user_init(unsigned char chnn)
{
    gpio_set_func(GREEN_LED_PIN | RED_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN | RED_LED_PIN, 1);
    gpio_set_input_en(GREEN_LED_PIN | RED_LED_PIN, 0);
    gpio_write(GREEN_LED_PIN | RED_LED_PIN, 0);

    ESB_Init(ESB_BITRATE_2MBPS);
    ESB_SetOutputPower(ESB_RF_POWER_0DBM);
    ESB_SetAddressWidth(ADDRESS_WIDTH_5BYTES);
    ESB_ClosePipe(ESB_PIPE_ALL);

    unsigned char tx_address[5] = {0xe7,0xe7,0xe7,0xe7,0xe7}; //{0xaa,0xbb,0xcc,0xdd,0xee};
    ESB_SetAddress(ESB_PIPE0, tx_address);
    ESB_OpenPipe(ESB_PIPE0);
    ESB_SetTXPipe(ESB_PIPE0);

    ESB_ModeSet(ESB_MODE_PTX);
    ESB_SetRFChannel(chnn);
    ESB_SetAutoRetry(0,150);  //5,150
    ESB_RxTimeoutSet(500);//if the mode is 250k ,the rx_time_out need more time, as so 1000us is ok!
    ESB_RxSettleSet(99);
    ESB_TxSettleSet(149);

    ESB_Preamble_Set(8);

    WaitUs(150);
    //configure irq
    irq_clr_src();
    rf_irq_clr_src(FLD_RF_IRQ_ALL);

    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq

    rf_irq_disable(FLD_RF_IRQ_ALL);

    rf_irq_enable(FLD_RF_IRQ_TX|FLD_RF_IRQ_TX_DS|FLD_RF_IRQ_RETRY_HIT|FLD_RF_IRQ_RX_DR);

    irq_enable(); //enable general irq
}



void main_loop(void)
{
    loop_cnt++;
    preamble_len = ESB_Preamble_Read();
    if (ds_flag || maxretry_flag)
    {
        if (ds_flag)
        {
            gpio_toggle(GREEN_LED_PIN);
        }
        else if (maxretry_flag)
        {
            gpio_toggle(RED_LED_PIN);
        }

        ds_flag = 0;
        maxretry_flag = 0;
        WaitMs(500);

        tx_data[0]++;
        tmp = ESB_WriteTxPayload(ESB_PIPE0, tx_data, 32);
        if (tmp)
        {
            ESB_PTXTrig();
        }
    }
}


/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_ int main (void)    //must run in ramcode
{
#if(MCU_CORE_TYPE == MCU_CORE_825x)
    cpu_wakeup_init();
#elif(MCU_CORE_TYPE == MCU_CORE_827x)
    cpu_wakeup_init(LDO_MODE,EXTERNAL_XTAL_24M);
#endif

	gpio_init(1);  //analog resistance will keep available in deepSleep mode, so no need initialize again

	clock_init(SYS_CLK_24M_Crystal);

	user_init(4);

    irq_enable();

    ds_flag = 1;
    while (1)
    {
        main_loop();
    }

    return 0;
}

/********************************************************************************************************
 * @file	dual_mode.c
 *
 * @brief	This is the source file for BLE SDK
 *
 * @author	BLE GROUP
 * @date	06,2020
 *
 * @par     Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "stack/2p4g/genfsk_ll/genfsk_ll.h"
#include "dual_mode.h"

#if DUAL_MODE_ENABLE
dual_mode_st_e s_dual_mode_st = DUAL_MODE_ST_BLE;

#if (GFSK_RX_MODE_SEL == GFSK_RX_MODE_ENUM_NORMAL)
#error TODO NORMAL MODE
#else
#define RX_BUF_LEN              64
#define RX_BUF_NUM              4

MYFIFO_INIT(scan_rx_fifo, RX_BUF_LEN, RX_BUF_NUM);

volatile static unsigned char *rx_packet = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned int rx_timestamp = 0;
volatile static unsigned char rssi = 0;
volatile static unsigned int rx_timeout_cnt, rx_cnt = 0;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) int gfsk_srx_irq_handler (void)
{
	int rf_irq_done_flag = 0;
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN)
    {
        if (rf_irq_src & FLD_RF_IRQ_RX)
        {
            rf_irq_clr_src(FLD_RF_IRQ_RX);
            rx_cnt++;
			u8 * raw_pkt = (scan_rx_fifo_b + (scan_rx_fifo.wptr++ & (scan_rx_fifo.num-1)) * scan_rx_fifo.size);
			u8 * new_pkt = (scan_rx_fifo_b + (scan_rx_fifo.wptr & (scan_rx_fifo.num-1)) * scan_rx_fifo.size);		
			gen_fsk_rx_buffer_set(new_pkt, scan_rx_fifo.size);
			
			if(gen_fsk_is_rx_crc_ok(raw_pkt)){
				
			}
			else{
				scan_rx_fifo.wptr--;
				gen_fsk_rx_buffer_set(raw_pkt, scan_rx_fifo.size);
			}

			gen_fsk_srx_start(clock_time() + 10, 0);	// RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives
        }

        if (rf_irq_src & FLD_RF_IRQ_FIRST_TIMEOUT)
        {
            rf_irq_clr_src(FLD_RF_IRQ_FIRST_TIMEOUT);
			gen_fsk_srx_start(clock_time() + 10, 0); 	// RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives
            rx_timeout_cnt++;
        }

        rf_irq_done_flag = 1;
    }

    return rf_irq_done_flag;
}

void gfsk_srx_init(unsigned char chn)
{
	u8 r = irq_disable();
	s_dual_mode_st = DUAL_MODE_ST_GFSK;
	
    // LED pin config
    gpio_set_func(GREEN_LED_PIN | RED_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN | RED_LED_PIN, 1);
    gpio_write(GREEN_LED_PIN | RED_LED_PIN, 0);

    // generic FSK Link Layer configuratioin
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    gen_fsk_datarate_set(GEN_FSK_DATARATE_2MBPS);               // Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word);            // set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0);                           // enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0);                         // set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, 8);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((scan_rx_fifo_b + (scan_rx_fifo.wptr & (scan_rx_fifo.num-1)) * scan_rx_fifo.size), scan_rx_fifo.size);
    gen_fsk_channel_set(chn);
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO);                // set transceiver to basic RX state
    gen_fsk_rx_settle_set(89);
    // irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_FIRST_TIMEOUT);    // enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN);                          // enable RF irq
    irq_restore(r);                                               // enable general irq
}

void gfsk_srx_main_loop(void)
{
	u8 *p = my_fifo_get(&scan_rx_fifo);
	if(p){
		rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);
        rssi = (gen_fsk_rx_packet_rssi_get((unsigned char *)rx_packet) + 110);
        rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);
		
		switch_to_gfsk_only_mode();
		my_fifo_pop(&scan_rx_fifo);
        gpio_toggle(GREEN_LED_PIN);
	}

    if (is_gfsk_only_mode())
    {
    	gpio_write(RED_LED_PIN, 1);
    }
	else{
		gpio_write(RED_LED_PIN, 0);
	}
}
#endif

_attribute_ram_code_sec_noinline_ int dual_mode_gfsk_rf_irq_handler (void)
{
	int rf_irq_done_flag = 0;
	if(DUAL_MODE_ST_BLE != s_dual_mode_st){
		rf_irq_done_flag = gfsk_srx_irq_handler();
	}

	return rf_irq_done_flag;
}

void switch_to_gfsk_only_mode()
{
	if(DUAL_MODE_ST_GFSK_ONLY != s_dual_mode_st)
	{
		blc_ll_setIdleState();
		blc_ll_switchScanChannel(0, 0);
		s_dual_mode_st = DUAL_MODE_ST_GFSK_ONLY;
	}
}

void blc_ll_switchScanChannel (int scan_mode, int set_chn)
{
	if(is_gfsk_only_mode() || blotaSvr.ota_busy){
		return;
	}
	
	if(scan_mode){
		static volatile u32 dual_mode_1switch_chn_timer;dual_mode_1switch_chn_timer++;
	}else{
		static volatile u32 dual_mode_2switch_in_adv_conn;dual_mode_2switch_in_adv_conn++;
		reset_rf_power();
		gfsk_srx_init(GFSK_RX_CHN);
		gen_fsk_srx_start(clock_time() + 10, 0); // RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives

		while(0){
			gfsk_srx_main_loop();
			static volatile u32 dual_mode_3switch_loop;dual_mode_3switch_loop++;
		}
	}
}

_attribute_ram_code_ void dual_mode_switch2ble_init () // about 12us in 16MHz
{
	if(is_gfsk_only_mode()){
		return;
	}
	
	if(DUAL_MODE_ST_GFSK == s_dual_mode_st){
		// switch to ADV tx state
		u8 r = irq_disable();
		reset_rf_power();
		s_dual_mode_st = DUAL_MODE_ST_BLE;
		rf_drv_init(RF_MODE_BLE_1M);
		rf_rx_buffer_set((blt_rxfifo_b + (blt_rxfifo.wptr & (blt_rxfifo.num-1)) * blt_rxfifo.size), blt_rxfifo.size, 0);
		irq_restore(r);
	}
}

int dual_mode_app_advertise_prepare (rf_packet_adv_t * p)
{
	int ret = 1;
	dual_mode_switch2ble_init();

	return ret;
}

_attribute_ram_code_ void dual_mode_blt_brx_start_init () // call in irq
{
	dual_mode_switch2ble_init();
}

void dual_mode_main_loop()
{
	gfsk_srx_main_loop();
}


// ---- init -----
void dual_mode_init()
{
#if 0 // test difference of rf init
	static volatile u32 A_0_next,A_1,A_2,A_3,A_4,A_5,A_6,A_7,A_8,A_9,A_10; //while(0 == A_0_next){A_1++;wd_clear();  }A_0_next--;
	while(0 == A_0_next){A_1++;wd_clear();	}A_0_next--;
	rf_drv_init_BLE_1M_with_all_reg();
	while(0 == A_0_next){A_2++;wd_clear();	}A_0_next--;
	gfsk_srx_init(GFSK_RX_CHN);
	gen_fsk_srx_start(clock_time() + 50 * 16, 0); // RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives
	while(0 == A_0_next){A_3++;wd_clear();	}A_0_next--;
	rf_drv_init_BLE_1M_with_all_reg();
	while(0 == A_0_next){A_4++;wd_clear();	}A_0_next--;
#endif

	bls_set_advertise_prepare(dual_mode_app_advertise_prepare);
	blc_ll_addScanningInAdvState();
	blc_ll_addScanningInConnSlaveRole();
}
#endif

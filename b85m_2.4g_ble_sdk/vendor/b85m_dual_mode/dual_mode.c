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

dual_mode_scan_serial_t scan_serial[] = {
	{1, 37, 10},
	{1, 38, 10},
	{1, 39, 10},
	{0, GFSK_RX_CHN, 10},
};

dual_mode_scan_st_t scan_state;

#define RX_BUF_LEN              64
#define RX_BUF_NUM              4

MYFIFO_INIT(scan_rx_fifo, RX_BUF_LEN, RX_BUF_NUM);

volatile static unsigned char *rx_packet = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned int rx_timestamp = 0;
volatile static unsigned char rssi = 0;
volatile static unsigned int rx_timeout_cnt, rx_cnt = 0;

#if DUAL_MODE_LED_INDICATE_EN
int gfsk_rx_tick = 0;
int ble_rx_tick = 0;
u32 gfsk_led_start_tick = 0;
u32 ble_led_start_tick = 0;
#endif

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
	s_dual_mode_st = is_gfsk_only_mode() ? DUAL_MODE_ST_GFSK_ONLY:DUAL_MODE_ST_GFSK;

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
		#if DUAL_MODE_LED_INDICATE_EN
		gfsk_rx_tick = clock_time()|1;
		#endif
	}
}
#endif

//------------------ ble adv filter start-------------------------------------//
#ifndef USER_ADV_FILTER_EN
#define USER_ADV_FILTER_EN		1
#endif

/**
 * @brief:return 1 means keep this packet, return 0 to discard.
 * @Note: user should not keep all adv packets, because they are too much. only keep those necessary packets by comparing playload, the less the better.
 *        so that the blt_rxfifo_ can be more efficient. 
 *        if "fifo_free_cnt" is less than 4, the packet should not be kept, or it may cause rx fifo overflowed.
 */
_attribute_ram_code_ u8 user_adv_filter_proc(u8 * p_rf_pkt)
{
	#if 1 // demo
	rf_packet_adv_t * pAdv = (rf_packet_adv_t *)p_rf_pkt;
	u8	sample_advData[] = { 0x05, 0x09, 'V', 'H', 'I', 'D'};
	if(0 == memcmp(pAdv->data, sample_advData, 10)){
		static u32 A_filter_adv_cnt;
		A_filter_adv_cnt++;
		return 1;
	}
	#endif
	
	return 0;
}

_attribute_ram_code_ int dual_mode_ble_adv_filter(u8 *raw_pkt)
{

    #define BLE_RCV_FIFO_MAX_CNT 	(8)	// set to 8 to keep same with last version. // refer to default buffer count of BLE generic SDK which is 8.
	static u32 A_filter_cnt;
	A_filter_cnt++;
	u8 next_buffer =1;
	u8 adv_type = 0;
	u8 mesh_msg_type = 0;
	u8 *p_mac = 0;
	u8 *p_rf_pkt =	(raw_pkt + 0);

	{ // make sure pAdv can be used only here.
		rf_packet_adv_t * pAdv = (rf_packet_adv_t *)p_rf_pkt;
		adv_type = pAdv->type;
		mesh_msg_type = pAdv->data[1];
		p_mac = pAdv->advA;
	}

	int adv_type_accept_flag = (LL_TYPE_ADV_NONCONN_IND == adv_type); // set default accept type.

	// "fifo_free_cnt" here means if accepte this packet, there still is the number of "fifo_free_cnt" remained, because wptr has been ++.
	u8 fifo_free_cnt = blt_rxfifo.num - ((u8)(blt_rxfifo.wptr - blt_rxfifo.rptr)&(blt_rxfifo.num-1));
	if(blc_ll_getCurrentState() == BLS_LINK_STATE_CONN){
		if(bltParam.ble_state == BLE_STATE_BRX_E){
			if(fifo_free_cnt < max2(BLE_RCV_FIFO_MAX_CNT, 2)){
				next_buffer = 0;
			}else if(0 == adv_type_accept_flag){
				next_buffer = 0;
			#if (USER_ADV_FILTER_EN)
				if(0 == next_buffer){
					next_buffer = user_adv_filter_proc(p_rf_pkt);
				}
			#endif
			}
					
		}else{			
			if(fifo_free_cnt < 1){
				write_reg8(0x800f00, 0x80); 		// stop ,just stop BLE state machine is enough 
			}
		}
	}else{
		if(0 == adv_type_accept_flag){
			next_buffer = 0;

		#if (USER_ADV_FILTER_EN)
			if(0 == next_buffer){
				next_buffer = user_adv_filter_proc(p_rf_pkt);
			}
		#endif
		}
		
	    if (fifo_free_cnt < 4){
			// can not make the fifo overflow 
			next_buffer = 0;
		}
	}
	
	return next_buffer;

}

_attribute_ram_code_ int  blc_ll_procScanPkt_ble_adv(u8 *raw_pkt, u8 *new_pkt, u32 tick_now)
{
	return dual_mode_ble_adv_filter(raw_pkt);
}

void blc_ll_initScanning_ble_adv(void)
{
	blc_ll_procScanPktCb = blc_ll_procScanPkt_ble_adv;
}
//-------------------------ble adv filter end----------------------------------------//

/**
 * @brief      callback function of LinkLayer Event
 * @param[in]  h     - LinkLayer Event type
 * @param[in]  param - data pointer of event
 * @param[in]  n     - data length of event
 * @return     none
 */
int controller_event_handler(u32 h, u8 *para, int n)
{
	if(h == (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META)){
		u8 subcode = para[0];

		//------------ ADV packet --------------------------------------------
		if (subcode == HCI_SUB_EVT_LE_ADVERTISING_REPORT)	// ADV packet
		{
			#if DUAL_MODE_LED_INDICATE_EN
			ble_rx_tick = clock_time()|1;
			#endif
			event_adv_report_t *pa = (event_adv_report_t *)para;
			if(LL_TYPE_ADV_NONCONN_IND != (pa->event_type & 0x0F)){
				// LL_TYPE_ADV_IND
				
				return 0;
			}
			
			// LL_TYPE_ADV_NONCONN_IND
		}
	}

	return 0;
}

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
		s_dual_mode_st = DUAL_MODE_ST_GFSK_ONLY; 	
	}
}

void switch_ble_scan_channel(int chn){
	rf_set_tx_rx_off ();
	rf_set_ble_channel (chn);
	blt_ll_set_ble_access_code_adv ();
	rf_set_ble_crc_adv ();
	if(blc_rf_pa_cb){	blc_rf_pa_cb(PA_TYPE_RX_ON);  }

	CLEAR_ALL_RFIRQ_STATUS;
	rf_set_rxmode ();
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

/**
 * @brief       This function server to get next gfsk scan serial index
 * @return      
 * @note        
 */
u8 get_next_gfsk_scan_serial()
{
	for(int i = 0; i < ARRAY_SIZE(scan_serial); i++){
		u8 next_idx = (i + scan_state.serial_idx + 1) % ARRAY_SIZE(scan_serial);
		if(scan_serial[next_idx].adv_scan == 0){
			return next_idx;
		}
	}

	return -1;
}


/**
 * @brief       This function server to get next scan serial index
 * @return      -1:invalid  other:next serial index
 * @note        
 */
void dual_mode_scan_serial_update()
{
	u8 idx = -1;
	if(is_gfsk_only_mode() || (blc_ll_getCurrentState() == BLS_LINK_STATE_CONN)){ // 2.4G scan only, get next 2.4G channel
		idx = get_next_gfsk_scan_serial();
	}
	else{
		idx = (scan_state.serial_idx + 1) % ARRAY_SIZE(scan_serial);
	}

	if(-1 != idx){
		scan_state.start_tick = clock_time()|1;
		scan_state.serial_idx = idx;
	}
	
}

u32 get_next_brx_invl_us()
{
	u32 t = reg_system_tick_irq - clock_time ();
	if(t < BIT(31)){
		return t/sys_tick_per_us;
	}
	else{
		return 0;
	}
}

/**
 * @brief       This function server to switch scan channel.
 * @param[in]   scan_mode	- 0:back to rx from tx, should force enable rx. 1:call by main loop, switch scan channel if need.
 * @param[in]   set_chn	- scan channel.
 * @return      none
 * @note        
 */
void blc_ll_switchScanChannel (int scan_mode, int set_chn)
{
	if(blc_ll_getCurrentState() == BLS_LINK_STATE_CONN){
		if(!(blts.scan_extension_mask & BLS_FLAG_SCAN_IN_SLAVE_MODE)){
			return;
		}
		
		u32 brx_time = get_next_brx_invl_us(); 
		if((bltParam.ble_state != BLE_STATE_BRX_E) || (brx_time < 1500)){ // don't scan in brx state, not use get_next_brx_invl_us() instead of brx_time here.
			return;	
		}
	}
	else{
		if(!(blts.scan_extension_mask & BLS_FLAG_SCAN_IN_ADV_MODE)){
			return;
		}
	}

	int scan_window_hit = (u32)(clock_time() - scan_state.start_tick) > scan_serial[scan_state.serial_idx].time_ms*1000*sys_tick_per_us;
	if(scan_window_hit){
		dual_mode_scan_serial_update();
	}

	if(scan_mode && !scan_window_hit){
		return;
	}

	dual_mode_scan_serial_t *p_scan = &scan_serial[scan_state.serial_idx];
	if(p_scan->adv_scan){ // ble channel
		dual_mode_switch2ble_init();
		switch_ble_scan_channel(p_scan->chn);
	}
	else{	// 2.4G channel
		static u32 A_gfsk = 0;A_gfsk++;
		reset_rf_power();
		gfsk_srx_init(p_scan->chn);
		gen_fsk_srx_start(clock_time() + 10, 0); // RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives			
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

#if DUAL_MODE_LED_INDICATE_EN
void dual_mode_led_indicate()
{
	#define LED_ONOFF_TIME_MS  200

	// 2.4G only led 
	if(is_gfsk_only_mode()){ 
		gpio_write(RED_LED_PIN, 1);	// always on
	}
	else{
		gpio_write(RED_LED_PIN, 0);
	}
	
	// 2.4G rx led
	if(gfsk_rx_tick){
		if(clock_time_exceed(gfsk_led_start_tick, LED_ONOFF_TIME_MS*1000)){
			gfsk_led_start_tick = clock_time()|1;
			gpio_toggle(BLUE_LED_PIN);
		}
	}

	if(gfsk_rx_tick && clock_time_exceed(gfsk_rx_tick, LED_ONOFF_TIME_MS*1000)){
		gfsk_rx_tick = 0;
		gpio_write(BLUE_LED_PIN, 0);
	}

	// ble adv led	
	if(ble_rx_tick){
		if(clock_time_exceed(ble_led_start_tick, LED_ONOFF_TIME_MS*1000)){
			ble_led_start_tick = clock_time()|1;
			gpio_toggle(GREEN_LED_PIN);
		}
	}

	if(ble_rx_tick && clock_time_exceed(ble_rx_tick, LED_ONOFF_TIME_MS*1000)){
		ble_rx_tick = 0;
		gpio_write(GREEN_LED_PIN, 0);
	}
}
#endif

void dual_mode_main_loop()
{
	gfsk_srx_main_loop();
	blc_ll_switchScanChannel(1, 0);
#if DUAL_MODE_LED_INDICATE_EN
	dual_mode_led_indicate();
#endif
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

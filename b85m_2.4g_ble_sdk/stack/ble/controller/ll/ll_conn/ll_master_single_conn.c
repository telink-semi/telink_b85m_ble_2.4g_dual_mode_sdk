/********************************************************************************************************
 * @file	ll_master_single_conn.c
 *
 * @brief	This is the source file for BLE SDK
 *
 * @author	BLE GROUP
 * @date	06,2020
 *
 * @par		Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd.
 *			All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/controller/ble_controller.h"
#include "stack/ble/host/ble_host.h"



#if (!LL_MASTER_MULTI_CONNECTION)   //single connection


#define FIX_SN_PROBLEM				1 //must open,hw bug,if dma tx fifo is empty, RF will send pkt by reg50c

#define TEST_SN_ISSUE				0
#if (TEST_SN_ISSUE)
u8				pkt_terminate_test[] =   { 0x04, 0, 0, 0,  0x03, 0x02, 0x02, 0x14};
#endif

#define ENCRYPT_START_SEND_PROTECT_EN		1

#if ENCRYPT_START_SEND_PROTECT_EN
#define	PKT_SKIP_ENCRYPT		(BIT(3))

#define		MAX_NUM_PENDING		4
typedef struct
{
	u8 wptr;
	u8 rptr;
	u8 buff[MAX_NUM_PENDING][48];
}blm_pkt_pending_t;

blm_pkt_pending_t blm_pkt_pending;
#endif


st_ll_conn_master_t blm[1] = { };


extern u32 lmp_tick;
extern u32 lmp_timeout;


void blm_set_software_timeout(u16 interval);



/////////////////////// SN/NESN /////////////////////////////////////////////////////
static inline void	blm_save_snnesn(st_ll_conn_master_t *p)
{
	p->blm_bb.sn_nesn = ((REG_ADDR8(0xf22) & BIT(0)) << 6) | ((REG_ADDR8(0xf23) & BIT(4)) << 3);
}

static inline void	blm_restore_snnesn(st_ll_conn_master_t *p)
{
	REG_ADDR8(0xf03) = (REG_ADDR8(0xf03) & ~BITS(6,7)) | (p->blm_bb.sn_nesn & BITS(6,7));
}

/////////////////////// DMA Tx fifo rptr /////////////////////////////////////////////
static inline void	blm_save_dma_tx_rptr(st_ll_conn_master_t *p)
{
	//TX Fifo: 0xc2a[0:3] means rptr
	p->blm_bb.dma_tx_rptr = reg_dma_tx_rptr & 0x0f;
}

static inline void	blm_restore_dma_tx_rptr(st_ll_conn_master_t *p)
{
	//0xc2a[6] rptr set
	reg_dma_tx_rptr = (BIT(6) | p->blm_bb.dma_tx_rptr);//restore tx_rptr
}


void irq_blm_tx(void);
int irq_blc_master_rx_data(u8 *raw_pkt, u32 tick_now);
int irq_blc_master_rx_post(void);
void irq_master_system_timer(void);
int blt_master_main_loop_data(u8 *raw_pkt);
int blt_master_main_loop_pre(void);
int blt_master_main_loop_post(void);

int blt_send_connection_establish_event(void);

///// Master Single Connection module ////
extern ll_irq_tx_callback_t ll_irq_tx_cb;
extern ll_irq_rx_data_callback_t ll_irq_rx_data_cb;
extern ll_irq_rx_post_callback_t ll_irq_rx_post_cb;
extern ll_irq_systemTick_conn_callback_t ll_irq_systemTick_conn_cb;
extern blc_main_loop_data_callback_t blc_main_loop_data_cb;
extern blc_main_loop_post_callback_t blc_main_loop_post_cb;

extern ll_conn_complete_handler_t			ll_connComplete_handler;

extern u8 blt_ll_version_ind_rsp_flag;

void blc_ll_initMasterRoleSingleConn_module(void) {
	//
	//ll_irq_tx_cb = irq_blm_tx;
	ll_irq_rx_data_cb = irq_blc_master_rx_data;
	ll_irq_rx_post_cb = irq_blc_master_rx_post;
	ll_irq_systemTick_conn_cb = irq_master_system_timer;

	blc_main_loop_data_cb = blt_master_main_loop_data;
	blc_main_loop_post_cb = blt_master_main_loop_post;

	ll_push_tx_fifo_handler = blm_push_fifo;

	FSM_TIMEOUT_DISABLE;			 //do not use FSM Timeout, use software timeout to end ERR BTX

	//test result: only rx_timeout valid in in BTX mode. first_rx_timeout not valid
	reg_rf_rx_timeout = 600; 		 //RX timeout:  > 150uS + 80uS(Coded PHY preamble) + 256uS(Coded access code) = 486 uS


	//st_ll_conn_master_t *pm = &blm[0];

}

///////////////////////////////////////////
//	adv							40
//  scan (initial state)		80
//	connect slave mode			1x
//	connect master mode			2x
//
//	rx: adv, scan, initiate
///////////////////////////////////////////
extern st_ll_scan_t blts;

extern _attribute_aligned_(4) st_ll_init_t blti;

u32 master_connecting_tick_flag;


//----------------------------------------------------------------
u8 blm_rxbuff[L2CAP_RX_BUFF_LEN_MAX];
//------------------------------------------------------------------
int blm_debug_adv_channel = 0;

int blm_encryption_st = 0;
int blm_disconnect = 0;
int blm_conn_update_complete = 0;
int blm_conn_update_pending = 0;
#if 1//(BQB_5P0_TEST_ENABLE) //during encrypt,slave can't send data to master, otherwise master will terminate connection with reason of 0x3d
	typedef enum
	{
		CRYPTE_IDLYT,
		CRYPTE_AFTER_SEND_LL_ENC_REQ ,// before encrypt is finished, master can receive non-data from slave, but can send non-data
		CRYPTE_AFTER_REC_LL_ENC_RSP , // before encrypt is finished, master can't receive and send non-data
	}blm_crypt_busy_t;
	blm_crypt_busy_t  blm_crypt_busy = CRYPTE_IDLYT;
#endif



//------ 	slot parameter --------------
st_ll_conn_master_t *blm_pconn = &blm[0];


//------ 	scan parameter --------------
u32 blm_scan_window = 55000; // 55 ms

//////////////////// rf packets //////////////////////////////
u8 ble_pkt_empty[] = { 0x02, 0, 0, 0, 0x01, 0x00 };


///////////////////////////////////////////////////////////////

extern u32 enc_skdm[];
extern u32 enc_ivm;
extern u8 blt_push_secReq_enable;

int blm_connect(u8 *raw_pkt);
///////////////////////////////////////////////////////////////










bool blm_push_fifo(int h, u8 *p) {
	st_ll_conn_master_t *pc = &blm[h & 7];


	int n = (reg_dma_tx_wptr - reg_dma_tx_rptr) & 15;

#if (TX_FIFO_DEFINED_IN_APP)
	n += (blt_txfifo.wptr - blt_txfifo.rptr) & 31;

	if ( n >= ((h & HANDLE_STK_FLAG) ? blt_txfifo.num : blt_txfifo.num - STACK_FIFO_NUM) ) {
		return 0;
	}

	u8 *pd = (u8 *)blt_txfifo_b + (blt_txfifo.wptr & (blt_txfifo.num-1)) * blt_txfifo.size;
#else
	n += (pc->tx_wptr - pc->tx_rptr) & 31;

	if ( n >=  ( (h & HANDLE_STK_FLAG) ? BLM_TX_FIFO_NUM : (BLM_TX_FIFO_NUM - STACK_FIFO_NUM) ) ) {
		return 0;
	}

	u8 *pd = (u8 *) pc->tx_fifo[pc->tx_wptr & (BLM_TX_FIFO_NUM - 1)];
#endif
	memcpy(pd + 4, p, p[1] + 2);

	if (pc->crypt.enable) {
		//aes_ll_ccm_encryption ( pd + 4, pc->crypt.enable == 2, & pc->crypt);
		aes_ll_ccm_encryption(pd + 4, 1, &pc->crypt);
	}

#if (1) // support RF RX/TX MAX data Length: 251byte
	pd[0] = (pd[5] + 2);    //DMA length low value
	pd[1] = (pd[5] + 2)>>8; //DMA length high value
#else
	pd[0] = (pd[5] + 2);
#endif

#if (TX_FIFO_DEFINED_IN_APP)
	blt_txfifo.wptr++;
#else
	pc->tx_wptr++;
#endif
	return 1;
}

_attribute_ram_code_ u8 blm_push_fifo_hw(int h) {

	st_ll_conn_master_t *pc = &blm[h & 7];

	pc->blm_bb.save_flg = 0;
	blm_restore_snnesn(pc);
	blm_restore_dma_tx_rptr(pc);

	//------------ push to HW fifo ----------------------------
	int n = (reg_dma_tx_wptr - reg_dma_tx_rptr) & 15;


#if (TX_FIFO_DEFINED_IN_APP)
#if (FIX_SN_PROBLEM)
	/*******************************************************************************************************************************************
	 * receive peer device's ACK  -> Action 1. local SN change; Action 2. HW_TX FIFO old data kick off
	 *  when data sending through TX FIFO (core_c2c),   Action 1 & Action 2 both correctly controlled by Hardware
	 *  but  data sending through RF_TX_DMA (core_c0c), only Action 1 correctly controlled by Hardware
	 *
	 *  if an empty packet(assume SN is 0b'0) is send through RF_TX_DMA in 1st conn_interval, peer device receive it, mark that new SN is 0b,0
	 *     then peer device send a ACK packet to local device, but local device did not receive it correctly(local SN no change)
	 *     if host push a valid data to controller between 1st conn_interval and 2nd conn_interval, this new data will be pushed to
	 *     HW_TX_FIFOin "Btx_start", then this packet will be send with SN 0b'0, when peer device receive this packet, consider it a retry data,
	 *      dropping it.    So this valid data is lost.
	 *
	 *  To solve the problem described above, a empty packet should be pushed to HW_TX_FIFO when SW_TX_FIFO is not empty, refer to slave's design
	 *******************************************************************************************************************************************/
	if (n == 0 && blt_txfifo.rptr != blt_txfifo.wptr)
	{
		reg_dma_tx_fifo = (u16)(u32) ble_pkt_empty;
		n = 1;
	}
	while (blt_txfifo.rptr != blt_txfifo.wptr && n < 6) //disable more data mode
#else
	while (blt_txfifo.rptr != blt_txfifo.wptr && n < 7)
#endif
	{
		reg_dma_tx_fifo = (u16)(u32)(blt_txfifo_b + (blt_txfifo.rptr++ & (blt_txfifo.num-1)) * blt_txfifo.size);
		n++;
	}

#else

#if (FIX_SN_PROBLEM)
	if (n == 0 && pc->tx_rptr == pc->tx_wptr)
	{
		reg_dma_tx_fifo = (u16)(u32) ble_pkt_empty;
		n = 1;
	}
	while (pc->tx_rptr != pc->tx_wptr && n < 6) //disable more data mode
#else
	while (pc->tx_rptr != pc->tx_wptr && n < 7)
#endif
	{
		reg_dma_tx_fifo = (u16) (u32) pc->tx_fifo[pc->tx_rptr
				& (BLM_TX_FIFO_NUM - 1)];

		pc->tx_rptr++;
		n++;
	}

#endif

	return 1;
}

////////////////////////////////////////////////


u32	btx_offset[4] = {	0, 150*SYSTEM_TIMER_TICK_1US, 141*SYSTEM_TIMER_TICK_1US, 147*SYSTEM_TIMER_TICK_1US };

_attribute_ram_code_ void blm_btx(void)
{

	blt_ll_start_common(blt_pconn);


	blm_push_fifo_hw(0);



#if(LL_FEATURE_ENABLE_LE_2M_PHY || LL_FEATURE_ENABLE_LE_CODED_PHY)
	tx_settle_adjust( tx_settle_master[blt_conn_phy.conn_cur_phy] );
	u32 t = systimer_get_irq_capture() + btx_offset[blt_conn_phy.conn_cur_phy];
#else
	tx_settle_adjust(LL_MASTER_TX_SETTLE);
	u32 t = systimer_get_irq_capture() + 150 * SYSTEM_TIMER_TICK_1US;
#endif


#if (TEST_SN_ISSUE)
	rf_start_btx (&pkt_terminate_test, t);  //pkt_terminate_test
#else
	rf_start_btx(&ble_pkt_empty, t);
#endif

	if(blc_rf_pa_cb){	blc_rf_pa_cb(PA_TYPE_TX_ON);  }


	if (blm_pconn->slave_terminate_conn_flag) {
		blm_pconn->slave_terminate_conn_flag++;
	}
}

void blm_btx_post(void) {

	if(blc_rf_pa_cb){	blc_rf_pa_cb(PA_TYPE_OFF);  }

	if(!blm_pconn->blm_bb.save_flg){   //for more insurance, if rx irq lost(empty packet), operation here would get correct sn/nesn/tx_rptr
		blm_save_snnesn (blm_pconn);
		blm_save_dma_tx_rptr(blm_pconn);
	}

	#if 1//(BQB_5P0_TEST_ENABLE)
		if(lmp_tick && clock_time_exceed(lmp_tick, lmp_timeout)) //timeout is conn_supervision_timeout or ll_response_timeout(40s)
		{
			blm_ll_enc_proc_disconnect(BLM_CONN_HANDLE, HCI_ERR_LMP_LL_RESP_TIMEOUT);

		}
	#endif

	if (!blttcon.conn_receive_packet){

	}
	else if (blm_pconn->master_terminate_conn_flag){ //master terminate

		#if (TX_FIFO_DEFINED_IN_APP)
			if( (((reg_dma_tx_wptr - reg_dma_tx_rptr) & 15)==0  &&  blt_txfifo.rptr == blt_txfifo.wptr) ) //receive terminate ack or timeout
		#else
				if (blm_pconn->tx_rptr == blm_pconn->tx_wptr) //empty
		#endif
			{
				blm_pconn->conn_terminate_pending = BLM_CONN_TERMINATE_SEND;
				blm_pconn->master_terminate_conn_flag = 0;
			}
		#if(BQB_5P0_TEST_ENABLE)
			else if( clock_time_exceed(blm_pconn->master_teminate_time, 500000))
			{
				blm_pconn->conn_terminate_pending = BLM_CONN_TERMINATE_SEND;
				blm_pconn->master_terminate_conn_flag = 0;
				blm_pconn->conn_terminate_reason = HCI_ERR_CONN_TERM_BY_LOCAL_HOST;
			}
		#endif
	}


	if (blm_pconn->conn_Req_waitAck_enable && clock_time_exceed(blttcon.conn_tick, blm_pconn->conn_Req_noAck_timeout)) {
		blc_ll_setIdleState();

		blttcon.conn_update = 0;
		blm_disconnect = BLM_CONN_HANDLE;
		blm_pconn->conn_terminate_reason = HCI_ERR_CONN_FAILED_TO_ESTABLISH;

		return;
	}

	if (clock_time_exceed(blttcon.conn_tick, blm_pconn->conn_timeout) || blm_pconn->conn_terminate_pending) {
		blttcon.conn_update = 0;
		blm_disconnect = BLM_CONN_HANDLE;

		if (blm_pconn->conn_terminate_pending == BLM_CONN_TERMINATE_SEND){
			#if (BQB_5P0_TEST_ENABLE)
				//here can not be commented ,or will be optimized by complie
				blm_pconn->conn_terminate_reason = blm[0].conn_terminate_reason;
			#else
				blm_pconn->conn_terminate_reason = HCI_ERR_CONN_TERM_BY_LOCAL_HOST;
			#endif
		}
		else if (blm_pconn->conn_terminate_pending == BLM_CONN_SLAVE_TERMINATE){
			blm_pconn->conn_terminate_reason = HCI_ERR_REMOTE_USER_TERM_CONN;
		}
		else{
			if (blm_pconn->conn_rcvd_slave_pkt) {
				blm_pconn->conn_terminate_reason = HCI_ERR_CONN_TIMEOUT;
			}
			else {
				blm_pconn->conn_terminate_reason = HCI_ERR_CONN_FAILED_TO_ESTABLISH;
			}
		}

		blm_pconn->conn_terminate_pending = 0;

		blttcon.conn_update = 0;

		blc_ll_setIdleState();

		#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
			bltParam.btxbrx_status = BTXBRX_NONE;
			bltParam.conn_role = LL_ROLE_NONE;
		#endif

		return;
	}

	blttcon.conn_inst++;

#if (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)
	if(blttcon.conn_chnsel==0)
	{
		blttcon.chn_idx++;
		if (blttcon.chn_idx >= 37) {
			blttcon.chn_idx -= 37;
		}
	}
#else
	blttcon.chn_idx++;
	if (blttcon.chn_idx >= 37) {
		blttcon.chn_idx -= 37;
	}
#endif

	if (blm_conn_update_pending) {
		blm_conn_update_pending = 0;
		blm_conn_update_complete = BLM_CONN_HANDLE;
	}
	//----------  conn_update -----------------------------------------------------------------------
	if ((blttcon.conn_update == 2) && (u16) (blttcon.conn_inst) == blttcon.conn_inst_next) {
		blttcon.conn_update = 0;

		blm_pconn->conn_interval = blm_pconn->conn_interval_next;
		blm_pconn->conn_timeout = blm_pconn->conn_timeout_next;
		blm_pconn->conn_latency = blm_pconn->conn_latency_next;

		blm_conn_update_pending = 1;

		blm_set_software_timeout((u16)blm_pconn->conn_interval);
		u32 new_btx_tick = systimer_get_irq_capture() + (BLM_WINOFFSET * 1250 + BLM_MID_WINSIZE) * SYSTEM_TIMER_TICK_1US;
		systimer_set_irq_capture(new_btx_tick);

		#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
			bltParam.acl_conn_start_tick = new_btx_tick;
		#endif

	}
	//--------------- channel map update ------------------------------------------------------------
	else if ((blttcon.conn_update == 1) && blttcon.conn_inst == blttcon.conn_inst_next) {
		blttcon.conn_update = 0;
		memcpy(blttcon.conn_chn_map, blttcon.conn_chn_map_next, 5); //not useful

		#if (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)
			if(blttcon.conn_chnsel)// channel selection alg2
			{
				blttcon.channel_id = (blttcon.conn_access_code>>16) ^ (blttcon.conn_access_code&0xffff);
				blc_calc_remapping_table(blttcon.conn_chn_map);//A remapping table is built that contains all the used channels.
			}
			else// channel selection alg1
			{
				blt_ll_channelTable_calc (blttcon.conn_chn_map, blttcon.conn_chn_hop, blttcon.chn_tbl);
			}
		#else
			blt_ll_channelTable_calc(blttcon.conn_chn_map, blttcon.conn_chn_hop & 31, blttcon.chn_tbl);
		#endif
	}
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	else if ((blttcon.conn_update == 3) && blttcon.conn_inst == blttcon.conn_inst_next) {
		blttcon.conn_update = 0;

		if(ll_conn_phy_update_cb){
			ll_conn_phy_update_cb();
		}
	}
#endif

}

st_ll_conn_master_t * blm_ll_getConnection(u16 h) {
	return &blm[h & 7];
}



void blm_set_software_timeout(u16 interval)
{
	if(interval < 8){  // <= 7*1.25ms = 8.75 mS
		blm[0].conn_software_timeout = (interval-1) * SYSTEM_TIMER_TICK_1250US;   // conn_interval - 1.25mS
	}
	else if(interval < 17){  // <= 16*1.25 = 20mS
		blm[0].conn_software_timeout = (interval-2) * SYSTEM_TIMER_TICK_1250US;   // conn_interval - 2.5 mS
	}
	else if(interval < 33){  // <= 32*1.25 = 40mS
		blm[0].conn_software_timeout = (interval-4) * SYSTEM_TIMER_TICK_1250US;   // conn_interval - 5 mS
	}
	else{  // > 40mS
		blm[0].conn_software_timeout = interval * SYSTEM_TIMER_TICK_625US;   	 // conn_interval/2
	}
}

int blm_connect(u8 *raw_pkt) {
	st_ll_conn_master_t *pm = &blm[0];

	//---------------       connect -------------------------------------------------


	pm->conn_btx_not_working = 1;


	blttcon.conn_tick = clock_time();

	pm->conn_rcvd_slave_pkt = 0;


	pm->conn_interval = pkt_init.interval;
	pm->conn_latency = pkt_init.latency;

	pm->conn_Req_waitAck_enable = 1;
	pm->conn_Req_noAck_timeout = (CONN_REQ_WAIT_ACK_NUM - 1) * pm->conn_interval * 1250 - 5000;

	pm->conn_timeout = pkt_init.timeout * 10000; //based on us

//	blttcon.conn_chn_hop = pkt_init.hop;    // where is " & 0x1f "

	reg_dma_tx_rptr = FLD_DMA_RPTR_CLR;
#if (TX_FIFO_DEFINED_IN_APP)
	blt_txfifo.rptr = blt_txfifo.wptr = 0;
#else
	pm->tx_wptr = pm->tx_rptr = 0;
#endif
	pm->blm_bb.sn_nesn = 0;
	pm->blm_bb.dma_tx_rptr = reg_dma_tx_rptr & 0x0f;

	pm->peer_adr_type = pkt_init.rxAddr; //RX_ADDR
	memcpy(pm->peer_adr, pkt_init.advA, 6);

	// wait tx done
	sleep_us(600);
	rf_ble_tx_done();


	blt_ll_connect_common(blt_pconn, (rf_packet_connect_t*)&pkt_init);

	blm_set_software_timeout(pkt_init.interval);


#if (FIX_HW_CRC24_EN)
	extern u32 revert_conn_crc;
	extern u32 reverse_32bit(volatile u32 x);
	u32 crc_init = (((u32)pkt_init.crcinit[2] << 16) | ((u32)pkt_init.crcinit[1]<<8) | pkt_init.crcinit[0]) & 0xffffff;
	revert_conn_crc = (reverse_32bit(crc_init) >> 8) & 0xffffff;
#endif

	reset_sn_nesn();

	pm->conn_sn = 100;

	bltParam.ble_state = BLE_STATE_BTX_E;
	u32 master_start_tick  = clock_time() + (BLM_WINOFFSET * 1250 + 3000)* SYSTEM_TIMER_TICK_1US;
	systimer_set_irq_capture(master_start_tick);
	systimer_clr_irq_status();

	#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
		bltParam.acl_conn_start_tick = master_start_tick;
		bltParam.btxbrx_status = BTXBRX_NEARBY;
		bltParam.conn_role = LL_ROLE_MASTER;
	#endif

#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
	if (0 && bltData.connInitialMaxTxOctets > MAX_OCTETS_DATA_LEN_27) {
		bltData.connMaxTxRxOctets_req = DATA_LENGTH_REQ_PENDING;
		bltData.connMaxTxOctets = bltData.connInitialMaxTxOctets;
	} else {
		bltData.connMaxTxRxOctets_req = 0;
		bltData.connMaxTxOctets = MAX_OCTETS_DATA_LEN_27;
	}
#endif

	blm_pconn->master_terminate_conn_flag = 0;
	blm_pconn->slave_terminate_conn_flag = 0;

	lmp_tick = 0;
	lmp_timeout = 0;
	blm_crypt_busy = CRYPTE_IDLYT;

	if(ll_connComplete_handler){
		ll_connComplete_handler(BLM_CONN_HANDLE, ((u8 *)&pkt_init.rf_len - 1));
	}

#if ENCRYPT_START_SEND_PROTECT_EN
	blm_pkt_pending.wptr = blm_pkt_pending.rptr = 0;
#endif

	blttcon.connHandle = BLM_CONN_HANDLE;


	return 0;
}


/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ int irq_blc_master_rx_data(u8 *raw_pkt, u32 tick_now) {
	int next_buffer = 0;
	u8 r_sn = (raw_pkt[DMA_RFRX_OFFSET_HEADER] >> 3) & 1; // blm_pconn->conn_sn;

#if (1)
	if(bltParam.drop_rx_data){

		blm_pconn->blm_bb.save_flg = 2; //mark, not save sn_nesn && dma_tx_rptr in btx_post stage, during btx stage: avoid 1st rx's crc24 software check err

		STOP_RF_STATE_MACHINE;  //stop state machine
		systimer_set_irq_capture(clock_time () + 800);
	}
	else{
		blm_pconn->blm_bb.save_flg = 1;  //mark
		blm_save_snnesn(blm_pconn);
		blm_save_dma_tx_rptr(blm_pconn);
	}
#endif



	if (bltParam.ble_state == BLE_STATE_BTX_S && !bltParam.drop_rx_data)
	{
#if (FIX_SN_PROBLEM)
		//refer to "blm_push_fifo_hw" FIX_SN_PROBLEM explanation, empty packet send through RF_TX_DMA is dangerous
		//If empty packet is inserted in Btx_start when HW_TX_FIFO empty and SW_TX_FIFO not empty, code below is not necessary(sihui & yafei 20190725)
		//but if "TEST_SN_ISSUE" is enabled to debug, code below should be added to prevent a terminate packet sending when more_data is used.
		if( (raw_pkt[DMA_RFRX_OFFSET_HEADER] & BIT(4)) && reg_dma_tx_wptr == reg_dma_tx_rptr)  //more data
		{
			reg_dma_tx_fifo = (u16)(u32) ble_pkt_empty;
		}
#endif

		if (r_sn != blm_pconn->conn_sn) {
			blm_pconn->conn_sn = r_sn;

			if (raw_pkt[DMA_RFRX_OFFSET_RFLEN] > 0) //non empty packet
			{ //flag handle of master connection
				next_buffer = 1;
				raw_pkt[2] = BLM_CONN_HANDLE;
			}

			blm_pconn->newRx = 1;
		}

		blttcon.conn_tick = clock_time();
		blttcon.conn_receive_packet = 1;

	}

	return next_buffer;
}

_attribute_ram_code_ int irq_blc_master_rx_post(void)
{

	if (bltParam.ble_state == BLE_STATE_BTX_S) {

		if(blm_pconn->conn_Req_waitAck_enable){
			blm_pconn->conn_Req_waitAck_enable = 0;
			blti.conn_established = 1;
			master_connecting_tick_flag = clock_time() | 1;
		}



		blm_pconn->conn_rcvd_slave_pkt = 1;
	}

	return 0;
}

void irq_blm_tx(void) {
	static u32 no_tx_data;

	if (bltParam.ble_state == BLE_STATE_BTX_S) {
		no_tx_data++;
	}
}


bool blm_ll_isRfStateMachineBusy(void)
{
	return bltParam.blm_btx_busy;
}

_attribute_ram_code_ void irq_master_system_timer(void) {
	static u32 blm_conn_slot_next_tick = 0;

	if (bltParam.ble_state == BLE_STATE_BTX_S) //start of BTX -> BTX end
	{
		if (bltParam.blm_btx_busy) {
			STOP_RF_STATE_MACHINE;		// fix boundary RX
			systimer_set_irq_capture( clock_time() + 200 * SYSTEM_TIMER_TICK_1US);
			bltParam.blm_btx_busy = 0;
			return;
		} else {
			systimer_set_irq_capture( blm_conn_slot_next_tick);
			bltParam.ble_state = BLE_STATE_BTX_E;

			#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
				bltParam.btxbrx_status = BTXBRX_NEARBY;
				bltParam.acl_conn_start_tick = blm_conn_slot_next_tick;
			#endif

			blm_btx_post();
			reg_rf_irq_mask = FLD_RF_IRQ_RX | FLD_RF_IRQ_TX;

			DBG_CHN1_LOW;
		}
	} else if (bltParam.ble_state == BLE_STATE_BTX_E) {
		st_ll_conn_master_t *pm = &blm[0];
		blm_pconn = pm;
		blm_conn_slot_next_tick = systimer_get_irq_capture() + pm->conn_interval * SYSTEM_TIMER_TICK_1250US;

		bltParam.blm_btx_busy = 1;

		//invalidate curret packet received
		u8 *pr = (u8 *) (blt_rxfifo_b + (blt_rxfifo.wptr & (blt_rxfifo.num - 1)) * blt_rxfifo.size);
		pr[DMA_RFRX_OFFSET_RFLEN] = 1;

		DBG_CHN1_HIGH;

		bltParam.ble_state = BLE_STATE_BTX_S;
		blm_btx();

		u32 cur_software_timeout = pm->conn_software_timeout; //unit: tick
		#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
			bltParam.btxbrx_status = BTXBRX_BUSY;
			if(bltParam.wirte_sonos_flash_req){
				cur_software_timeout -= (2*SYSTEM_TIMER_TICK_1250US);  //leave at least 3750uS for flash_write
			}
		#endif

		systimer_set_irq_capture( systimer_get_irq_capture() + cur_software_timeout);


		if (pm->conn_btx_not_working) {
			pm->conn_btx_not_working = 0;
			blttcon.conn_tick = clock_time();
		}

	}
}

rf_packet_l2cap_t * blm_l2cap_packet_pack(u16 conn, u8 * raw_pkt) {
	rf_packet_l2cap_req_t *pl = (rf_packet_l2cap_req_t *) (raw_pkt); //+DMA_RFRX_LEN_HW_INFO
	u8 type = pl->type & 3;

	if (type == 2) {
		if (pl->rf_len == (pl->l2capLen + 4)) {
			return (rf_packet_l2cap_t *)(raw_pkt); //+DMA_RFRX_OFFSET_HEADER
		} else if (pl->l2capLen + 4 > pl->rf_len) {

			if(l2cap_buff.rx_p == NULL)
			{
				return NULL;
			}

			u8 len = pl->rf_len + 2;//DMA_RFRX_OFFSET_DATA
			pl->rf_len = pl->l2capLen + 4;

			memcpy(l2cap_buff.rx_p, raw_pkt, len);
			U16_SET(l2cap_buff.rx_p, len);
		} else {
			U16_SET(l2cap_buff.rx_p, 0);
		}
	}
	//------------- LL L2CAP continuous packet ---------------------------------
	else if (type == 1) {

		if((l2cap_buff.rx_p == NULL)  || (U16_GET(l2cap_buff.rx_p)==0))
		{
			return NULL;
		}

		u16 att_len = U16_GET(l2cap_buff.rx_p)-2 -4 + pl->rf_len; //-DMA_RFRX_OFFSET_DATA --> - 2
		if(att_len > (l2cap_buff.max_rx_size-6))//excess MTU. //-6 confirmed with Yafei/Qinghua.
		{
			U16_SET(l2cap_buff.rx_p, 0);
			return (rf_packet_l2cap_t *)(l2cap_buff.rx_p);//+DMA_RFRX_OFFSET_HEADER
		}

		memcpy(l2cap_buff.rx_p + U16_GET(l2cap_buff.rx_p), raw_pkt + 2, pl->rf_len);//+DMA_RFRX_OFFSET_DATA--> + 2
		U16_SET(l2cap_buff.rx_p, U16_GET(l2cap_buff.rx_p) + pl->rf_len);

		rf_packet_l2cap_req_t *ps = (rf_packet_l2cap_req_t *) (l2cap_buff.rx_p);//+DMA_RFRX_LEN_HW_INFO
		if (U16_GET(l2cap_buff.rx_p) >= ps->l2capLen + (2 + 4)) //DMA_RFRX_OFFSET_DATA --> 2
		{
			U16_SET(l2cap_buff.rx_p, 0);
			return (rf_packet_l2cap_t *)(l2cap_buff.rx_p);  //DMA_RFRX_OFFSET_HEADER
		}
	}

	return NULL;
}

//----------------------------------


void blm_ll_pushEncPkt(u8 conn_handle, int type) {
	u8 pkt_enc[28];

	pkt_enc[0] = 0x03;

	if (type == LL_ENC_REQ) {

		pkt_enc[1] = 0x17;
		memcpy(pkt_enc + 3, blm[conn_handle].enc_random, 8);
		memcpy(pkt_enc + 11, &blm[conn_handle].enc_ediv, 2);
		memcpy(pkt_enc + 13, enc_skdm, 8);
		memcpy(pkt_enc + 21, &enc_ivm, 4);

	} else if (type == LL_START_ENC_RSP) {

		pkt_enc[1] = 1;

	} else if (type == LL_REJECT_IND) {

		pkt_enc[1] = 0x2;
		pkt_enc[3] = 0x06; //PIN missing

	}

	pkt_enc[2] = type;

	blm_push_fifo(conn_handle, pkt_enc);
}

void blm_ll_security_proc() {

	int i_connhandle = 0;

	if(blm_pconn->newRx)
	{
		blm_pconn->newRx = 0;

		if ((blm_pconn->crypt.st != MASTER_LL_ENC_OFF) && (blm_pconn->crypt.st != LL_REJECT_IND))
		{

			if (blm_pconn->crypt.st == MASTER_LL_ENC_REQ)   //for master
			{
				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = clock_time() | 1;
					lmp_timeout = blm_pconn->conn_timeout;

					blm_crypt_busy = CRYPTE_AFTER_SEND_LL_ENC_REQ;
				#endif

				generateRandomNum(8, (u8 *)enc_skdm);//skdm and ivm random generate
				generateRandomNum(4, (u8 *)&enc_ivm);

				blm_ll_pushEncPkt (i_connhandle, LL_ENC_REQ);
				blm_pconn->crypt.st = MASTER_LL_ENC_RSP_T;

				blc_ll_setEncryptionBusy (1);

			}
			else if( blm_pconn->crypt.st == MASTER_LL_ENC_PAUSE_RSP)
			{
				blm_ll_pushEncPkt (i_connhandle, LL_PAUSE_ENC_RSP);
				blm_pconn->crypt.st = MASTER_LL_ENC_REQ;
			}
			else if( blm_pconn->crypt.st == MASTER_LL_ENC_PAUSE_REQ)
			{
				blm_ll_pushEncPkt (i_connhandle, LL_PAUSE_ENC_REQ);
				blm_pconn->crypt.st = MASTER_LL_ENC_PAUSE_RSP_T;
				blm_pconn->crypt.enable = 0;
				blc_ll_setEncryptionBusy (0);
			}
			else if(blm_pconn->crypt.st == MASTER_LL_REJECT_IND_T)
			{
				blm_pconn->crypt.st = MASTER_LL_ENC_OFF;
				blm_ll_pushEncPkt (i_connhandle, LL_REJECT_IND);
			}
			else if (blm_pconn->crypt.st == MASTER_LL_ENC_SMP_INFO_S)  // start send distribute key information
			{

				if(func_smp_info){
					u8 *pr = (u8 *)func_smp_info(i_connhandle); 	//send encryption info
					if (pr){
						blm_push_fifo (i_connhandle, pr);
					}else{
						blm_pconn->crypt.st = MASTER_LL_ENC_SMP_INFO_E;  // distribute key information end
					}
				}
				else{
					blm_pconn->crypt.st = MASTER_LL_ENC_SMP_INFO_E;
				}

			}

		}

	}
#if (ENCRYPT_START_SEND_PROTECT_EN)
		if((blm_crypt_busy == CRYPTE_IDLYT))
		{
			//(blm_pkt_pending.buff[blm_pkt_pending.rptr & (MAX_NUM_PENDING-1)][DMA_RFRX_OFFSET_HEADER]>0) &&
			while(blm_pkt_pending.wptr != blm_pkt_pending.rptr)
			{
				if(blm_pkt_pending.buff[blm_pkt_pending.rptr & (MAX_NUM_PENDING-1)][DMA_RFRX_OFFSET_RFLEN]>0)
				{
					if(!blt_master_main_loop_data(blm_pkt_pending.buff[blm_pkt_pending.rptr & (MAX_NUM_PENDING-1)]))
					{
						blm_pkt_pending.buff[blm_pkt_pending.rptr & (MAX_NUM_PENDING-1)][0] = 0;
						blm_pkt_pending.rptr++;
					}
				}
				else{
					blm_pkt_pending.rptr++;
				}
				blm_pkt_pending.rptr &= (MAX_NUM_PENDING-1);
			}
		}
#endif
}

int blm_ll_startEncryption(u8 connhandle, u16 ediv, u8* random, u8* ltk) {

	if (blm[connhandle & 7].crypt.enable)
	{
		blm[connhandle & 7].crypt.st = MASTER_LL_ENC_PAUSE_REQ;
	}
	else
	{
		blm[connhandle & 7].crypt.st = MASTER_LL_ENC_REQ;
	}
	blm[connhandle & 7].enc_ediv = ediv;
	memcpy(blm[connhandle & 7].enc_random, random, 8);
	memcpy(blm[connhandle & 7].crypt.sk, ltk, 16);

	blm[connhandle &7].crypt.mic_fail = BLE_SUCCESS; //clear fail reason

	return BLE_SUCCESS;
}

void blm_ll_startDistributeKey(u8 connhandle) {
	blm[connhandle&7].crypt.st = MASTER_LL_ENC_SMP_INFO_S;
}

int blt_master_main_loop_data(u8 *raw_pkt)
{
	#if 1//(BQB_5P0_TEST_ENABLE) // for LL_SEC_MAS_BV_12, after finish encryption can response LL_VERSION_IND or LL_FEATURE_RSP to slave
//		static u8 ll_req_pending = 0;  //remove warning
		#define 	LL_VERSION_IND_PENDING		(BIT(0))
		#define 	LL_FEATURE_IND_PENDING		(BIT(1))
	#endif
	//any data callback must after connection establish event
	if(blti.conn_established)    //telink private
	{
		blt_send_connection_establish_event();
	}


	rf_packet_l2cap_req_t *pl = (rf_packet_l2cap_req_t *) (raw_pkt + DMA_RFRX_OFFSET_HEADER);
	u16 dat16[32];
	u8 *p8 = (u8 *) dat16;

	u8 type = pl->type & 3;
	u8 idx = raw_pkt[2] & 7;
	st_ll_conn_master_t *pc = &blm[idx];

	if (raw_pkt[DMA_RFRX_OFFSET_RFLEN])
	{
		int st = 0;
#if(ENCRYPT_START_SEND_PROTECT_EN)
		if (pc->crypt.enable && (raw_pkt[0]!=PKT_SKIP_ENCRYPT)) {

			if(blm_pconn->crypt.mic_fail){
				return 0;
			}

			st = aes_ll_ccm_decryption(raw_pkt + DMA_RFRX_OFFSET_HEADER, 0, &pc->crypt);
		}

		if( st )  //decrypt err
		{
			pc->crypt.mic_fail = 1;
			blm_ll_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
			return 0;
		}


		if(blm_crypt_busy == CRYPTE_AFTER_SEND_LL_ENC_REQ)
		{
			rf_packet_ll_control_t *pll = (rf_packet_ll_control_t *) (raw_pkt + DMA_RFRX_OFFSET_HEADER);
			u8 opcode = pll->opcode;

			if((type == LLID_CONTROL) &&(opcode == LL_TERMINATE_IND || opcode == LL_ENC_RSP || opcode == LL_START_ENC_REQ \
					|| opcode == LL_START_ENC_RSP || opcode == LL_REJECT_IND_EXT || opcode == LL_REJECT_IND ))
			{

			}
			else
			{
				memcpy(blm_pkt_pending.buff[blm_pkt_pending.wptr & (MAX_NUM_PENDING-1)], raw_pkt, raw_pkt[DMA_RFRX_OFFSET_RFLEN] +6);
				blm_pkt_pending.buff[blm_pkt_pending.wptr][0] = PKT_SKIP_ENCRYPT;
				blm_pkt_pending.wptr ++;
				blm_pkt_pending.wptr &= (MAX_NUM_PENDING-1);
				return 1;
			}
		}

		if(blm_crypt_busy == CRYPTE_AFTER_REC_LL_ENC_RSP)
		{
			rf_packet_ll_control_t *pll = (rf_packet_ll_control_t *) (raw_pkt + DMA_RFRX_OFFSET_HEADER);
			u8 opcode = pll->opcode;

			if((type == LLID_CONTROL) &&(opcode == LL_TERMINATE_IND || opcode == LL_ENC_RSP || opcode == LL_START_ENC_REQ \
					|| opcode == LL_START_ENC_RSP || opcode == LL_REJECT_IND_EXT || opcode == LL_REJECT_IND ))
			{

			}
			else{
				pc->crypt.mic_fail = 1;
				blm_ll_enc_proc_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
				return 0;
			}
		}
#else
		if (pc->crypt.enable) {

			if(blm_pconn->crypt.mic_fail){
				return 0;
			}

			st = aes_ll_ccm_decryption(raw_pkt + DMA_RFRX_OFFSET_HEADER, 0, &pc->crypt);
		}

		if( st )  //decrypt err
		{
			pc->crypt.mic_fail = 1;
			blm_ll_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
			return 0;
		}
#endif


		//------------------ LL control --------------------------------------------
		if (type == LLID_CONTROL)
		{
			rf_packet_ll_control_t *pll = (rf_packet_ll_control_t *) (raw_pkt + DMA_RFRX_OFFSET_HEADER);
			u8 opcode = pll->opcode;
#if(!ENCRYPT_START_SEND_PROTECT_EN)
			if(blm_crypt_busy == CRYPTE_AFTER_REC_LL_ENC_RSP)
			{
				if(opcode == LL_TERMINATE_IND || opcode == LL_ENC_RSP || opcode == LL_START_ENC_REQ \
						|| opcode == LL_START_ENC_RSP || opcode == LL_REJECT_IND_EXT || opcode == LL_REJECT_IND )
				{

				}
				else{
					pc->crypt.mic_fail = 1;
					blm_ll_enc_proc_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
					return 0;
				}

			}
#endif
			#if 1//(BQB_5P0_TEST_ENABLE)
				extern u8 const cmd_length_array[26];
				if(opcode < (LL_MIN_USED_CHN_IND+1) && pll->rf_len != cmd_length_array[opcode])
				{
					blt_ll_unknown_rsp(idx, opcode);
					return 0;
				}
			#endif




			if (opcode == LL_ENC_RSP)
			{
				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = clock_time() | 1;
					lmp_timeout = blm_pconn->conn_timeout;

					blm_crypt_busy = CRYPTE_AFTER_REC_LL_ENC_RSP;
				#endif

				memcpy(&pc->enc_ivs, pll->dat + 8, 4);
				memcpy(pc->enc_skds, pll->dat, 8);
				aes_ll_ccm_encryption_init(pc->crypt.sk, (u8 *)enc_skdm, pc->enc_skds,
						(u8 *)&enc_ivm, (u8 *)&pc->enc_ivs, &pc->crypt);
			}
			else if (opcode == LL_START_ENC_REQ) { // control packets

				pc->crypt.enable = 1;
				//start encryption
				blm_ll_pushEncPkt(idx, LL_START_ENC_RSP);

				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = clock_time() | 1;
					lmp_timeout = blm_pconn->conn_timeout;
				#endif
			}
			else if (opcode == LL_START_ENC_RSP)
			{
				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = 0;
					lmp_timeout = 0;
				#endif

				pc->crypt.st = MASTER_LL_ENC_START_RSP_T;

				blm_encryption_st |= idx;
				if (blm_encryption_st & BLM_CONN_ENC_REFRESH_T)
				{
					blm_encryption_st |= BLM_CONN_ENC_REFRESH;
				}
				else
				{
					#if(BQB_5P0_TEST_ENABLE)
						u8 p8[32];
						event_enc_change_t* pc = (event_enc_change_t*) p8;
						pc->status = blm_pconn->crypt.mic_fail;  //BLE_SUCCESS
						pc->handle = BLM_CONN_HANDLE; // handle
						pc->enc_enable = blm_pconn->crypt.enable;
						blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_ENCRYPTION_CHANGE, p8, 4);
						blm_encryption_st = 0;
					#else
						blm_encryption_st |= BLM_CONN_ENC_CHANGE;
					#endif
				}
				blm_crypt_busy = CRYPTE_IDLYT;
				blc_ll_setEncryptionBusy (0);
				#if (!ENCRYPT_START_SEND_PROTECT_EN)//(BQB_5P0_TEST_ENABLE)
					if(ll_req_pending & LL_VERSION_IND_PENDING)
					{
						ll_req_pending &= ~LL_VERSION_IND_PENDING;
						dat16[0] = 0x0603; //type, len
						dat16[1] = LL_VERSION_IND | (BLUETOOTH_VER << 8);
						dat16[2] = VENDOR_ID;
						dat16[3] = BLUETOOTH_VER_SUBVER;
						blm_push_fifo(idx, (u8 *) dat16);
						blt_ll_version_ind_rsp_flag = 1;
					}
					if(ll_req_pending & LL_FEATURE_IND_PENDING)
					{
						ll_req_pending &= ~LL_FEATURE_IND_PENDING;
					}
				#endif

			}
			else if (opcode == LL_PAUSE_ENC_RSP)
			{
				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = 0;
					lmp_timeout = 0;
				#endif

				blm_encryption_st |= BLM_CONN_ENC_REFRESH_T;
				pc->crypt.st = MASTER_LL_ENC_PAUSE_RSP;
				#if 1//(BQB_5P0_TEST_ENABLE)
					blm_crypt_busy = CRYPTE_IDLYT;
				#endif
			}
			else if(opcode == LL_REJECT_IND)
			{
				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = 0;
					lmp_timeout = 0;

					blm_crypt_busy = CRYPTE_IDLYT;
				#endif

				blm_encryption_st |= BLM_CONN_ENC_CHANGE;
				pc->crypt.mic_fail = pll -> dat[0];
				pc->crypt.st = MASTER_LL_ENC_SMP_INFO_E;
			}
			else if(opcode == LL_REJECT_IND_EXT)
			{
				#if 1//(BQB_5P0_TEST_ENABLE)
					lmp_tick = 0;
					lmp_timeout = 0;

					blm_crypt_busy = CRYPTE_IDLYT;
				#endif

				blm_encryption_st |= BLM_CONN_ENC_CHANGE;
				pc->crypt.mic_fail = pll -> dat[1];
				pc->crypt.st = MASTER_LL_ENC_SMP_INFO_E;
			}
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
			else if (opcode == LL_PHY_REQ || opcode == LL_PHY_RSP)	//LL Control Opcode: 0x16/0x17
			{
				if( !(LL_FEATURE_MASK_0 & (LL_FEATURE_ENABLE_LE_2M_PHY<<8 | LL_FEATURE_ENABLE_LE_CODED_PHY<<11) ) ){
					dat16[0] = 0x0203; //type, len
					dat16[1] = LL_UNKNOWN_RSP | (opcode << 8); //feature
					blm_push_fifo(idx, (u8 *) dat16);
				}
				else{

					rf_pkt_ll_phy_req_rsp_t* pReq = (rf_pkt_ll_phy_req_rsp_t *)(raw_pkt+DMA_RFRX_OFFSET_HEADER);

					u8	comm_phy = pReq->tx_phys & pReq->rx_phys;  //support symmetric PHYs only; support 1M/2M/Coded PHY all

					if(opcode == LL_PHY_REQ){  //slave initiated

					}
					else{  //master initiated
						comm_phy = comm_phy & blt_conn_phy.conn_prefer_phys;
					}

					//in case that comm_phy include more than 1 PHY
					if(comm_phy & PHY_PREFER_1M){
						blt_conn_phy.conn_update_phy = PHY_PREFER_1M;
						blt_conn_phy.conn_next_phy = BLE_PHY_1M;
					}
					else if(comm_phy & PHY_PREFER_2M){
						blt_conn_phy.conn_update_phy = PHY_PREFER_2M;
						blt_conn_phy.conn_next_phy = BLE_PHY_2M;
					}
					else if(comm_phy & PHY_PREFER_CODED){
						blt_conn_phy.conn_update_phy = PHY_PREFER_CODED;
						blt_conn_phy.conn_next_phy = BLE_PHY_CODED;
					}
					else{  //no PHY Update
						blt_conn_phy.conn_next_phy = blt_conn_phy.conn_cur_phy;
					}

					blt_conn_phy.phy_update_pending = 1;

					#if 0//(BQB_5P0_TEST_ENABLE)
						if(opcode == LL_PHY_RSP){
							if(lmp_tick)
							{
								lmp_tick = 0;
								lmp_timeout = 0;
							}
						}
					#endif
				}

			}
#endif
			else if (opcode == LL_VERSION_IND) { // control packets
				//extern u32 timer_tick;
				//timer_tick = 0;
				if (pc->remote_version) {
					pc->remote_version = 0;

					if (hci_eventMask & HCI_EVT_MASK_READ_REMOTE_VERSION_INFORMATION_COMPLETE) {
						p8[0] = 0; // status: success
						p8[1] = BLM_CONN_HANDLE; // handle
						p8[2] = BLM_CONN_HANDLE >> 8; // handle
						memcpy(p8 + 3, pll->dat, 5);

						blc_hci_send_event( HCI_FLAG_EVENT_BT_STD | HCI_EVT_READ_REMOTE_VER_INFO_COMPLETE, p8, 8); //read remote version complete
					}
				}
				else {
					#if(BQB_5P0_TEST_ENABLE)
						if(blm_crypt_busy == CRYPTE_AFTER_SEND_LL_ENC_REQ)
						{
							ll_req_pending |= LL_VERSION_IND_PENDING;
						}
						else if(blm_crypt_busy == CRYPTE_AFTER_REC_LL_ENC_RSP)
						{
							pc->crypt.mic_fail = 1;
							blm_ll_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
						}
						else
						{
							dat16[0] = 0x0603; //type, len
							dat16[1] = LL_VERSION_IND | (BLUETOOTH_VER << 8);
							dat16[2] = VENDOR_ID;
							dat16[3] = BLUETOOTH_VER_SUBVER;
							blm_push_fifo(idx, (u8 *) dat16);
						}
					#else
						if(blm_crypt_busy == CRYPTE_AFTER_REC_LL_ENC_RSP)
						{
							pc->crypt.mic_fail = 1;
							blm_ll_enc_proc_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
							return 0;
						}
						else if(!blt_ll_version_ind_rsp_flag)
						{
							#if (!ENCRYPT_START_SEND_PROTECT_EN)
								if(blm_crypt_busy == CRYPTE_AFTER_SEND_LL_ENC_REQ)
								{
									ll_req_pending |= LL_VERSION_IND_PENDING;
								}
								else
							#endif
								{
									dat16[0] = 0x0603; //type, len
									dat16[1] = LL_VERSION_IND | (BLUETOOTH_VER << 8);
									dat16[2] = VENDOR_ID;
									dat16[3] = BLUETOOTH_VER_SUBVER;
									blm_push_fifo(idx, (u8 *) dat16);
									blt_ll_version_ind_rsp_flag = 1;
								}
						}
					#endif
				}
			}
			else if (opcode == LL_UNKNOWN_RSP) {
				if (pll->dat[0] == LL_FEATURE_REQ) {
					//						if(blm_pconn->remoteFeatureReq){
					//							blm_pconn->remoteFeatureReq = 0;
					//							blm_pconn->ll_remoteFeature = 0;
					//							bls_ll_send_read_remote_feature_event(HCI_ERR_UNSUPPORTED_REMOTE_FEATURE);
					//						}
				}
#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
				else if (pll->dat[0] == LL_LENGTH_REQ) {
					bltData.connRemoteMaxTxOctets = bltData.connRemoteMaxRxOctets
							= MAX_OCTETS_DATA_LEN_27;
					bltData.connEffectiveMaxTxOctets
							= bltData.connEffectiveMaxRxOctets
									= MAX_OCTETS_DATA_LEN_27;
				}
#endif
			}
			else if (opcode == LL_SLAVE_FEATURE_REQ) { // control packets
				#if (!ENCRYPT_START_SEND_PROTECT_EN)//(BQB_5P0_TEST_ENABLE)
					if(blm_crypt_busy == CRYPTE_AFTER_SEND_LL_ENC_REQ)
					{
						ll_req_pending |= LL_FEATURE_IND_PENDING;
					}
					else
					{
						dat16[0] = 0x0903; //type, len
						dat16[1] = LL_FEATURE_RSP 		| LL_FEATURE_BYTE_0 <<8; //feature
						dat16[2] = LL_FEATURE_BYTE_1	| LL_FEATURE_BYTE_2 <<8;
						dat16[3] = LL_FEATURE_BYTE_3	| LL_FEATURE_BYTE_4 <<8;
						dat16[4] = LL_FEATURE_BYTE_5	| LL_FEATURE_BYTE_6 <<8;
						dat16[5] = LL_FEATURE_BYTE_7;
						blm_push_fifo(idx, (u8 *) dat16);
					}
				#else
					dat16[0] = 0x0903; //type, len
					dat16[1] = LL_FEATURE_RSP 		| LL_FEATURE_BYTE_0 <<8; //feature
					dat16[2] = LL_FEATURE_BYTE_1	| LL_FEATURE_BYTE_2 <<8;
					dat16[3] = LL_FEATURE_BYTE_3	| LL_FEATURE_BYTE_4 <<8;
					dat16[4] = LL_FEATURE_BYTE_5	| LL_FEATURE_BYTE_6 <<8;
					dat16[5] = LL_FEATURE_BYTE_7;
					blm_push_fifo(idx, (u8 *) dat16);
				#endif
			}
			else if (opcode == LL_FEATURE_RSP) {
				//extern u32 timer_tick;
				//timer_tick = 0;
				pc->ll_remoteFeature = pll->dat[0] | (pll->dat[1]<<8) | (pll->dat[2]<<16) | (pll->dat[3]<<24);

				if (hci_le_eventMask & HCI_LE_EVT_MASK_READ_REMOTE_FEATURES_COMPLETE) {
					p8[0] = HCI_SUB_EVT_LE_READ_REMOTE_USED_FEATURES_COMPLETE; // sub code
					p8[1] = 0; // status: success
					dat16[1] = idx | BLM_CONN_HANDLE; // handle
					memcpy(p8 + 4, pll->dat, 8);

					blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, p8, 12);
				}
			}
			else if (opcode == LL_TERMINATE_IND) {
				pc->conn_terminate_reason = pll->dat[0];
				pc->slave_terminate_conn_flag = 1;
				#if 1//(BQB_5P0_TEST_ENABLE)
					blm_crypt_busy = CRYPTE_IDLYT;
				#endif
			}
#if (LL_FEATURE_ENABLE_LE_PING)
			else if (opcode == LL_PING_REQ ){
				dat16[0] = 0x0103;		//type, len
				dat16[1] = LL_PING_RSP;
				blm_push_fifo (idx, (u8 *)dat16);
			}
#endif
#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
			else if (opcode == LL_LENGTH_REQ || opcode == LL_LENGTH_RSP) { // data length request

			#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
					u16 connRemoteMaxRxOctets = pll->dat[0] | (pll->dat[1] << 8);
					u16 connRemoteMaxTxOctets = pll->dat[4] | (pll->dat[5] << 8);

					if(connRemoteMaxRxOctets < 27 || connRemoteMaxRxOctets > 251 || connRemoteMaxTxOctets < 27 || connRemoteMaxTxOctets > 251){
						// unknown rsp
						dat16[0] = 0x0203; //type, len
						dat16[1] = LL_UNKNOWN_RSP | (opcode << 8); //feature
						blm_push_fifo(idx, (u8 *) dat16);
						return 0;
					}
					bltData.connRemoteMaxRxOctets = connRemoteMaxRxOctets;
					bltData.connRemoteMaxTxOctets = connRemoteMaxTxOctets;

					u16 new_effectiveMaxRx = min(bltData.supportedMaxRxOctets, bltData.connRemoteMaxTxOctets);
					u16 new_effectiveMaxTx = min(bltData.supportedMaxTxOctets, bltData.connRemoteMaxRxOctets);

					//when master send LL_LENGTH_REQ and(connMaxTxOctets < supportedMaxTxOctets) or (connMaxRxOctets < supportedMaxRxOctets)
					//slave will response with LL_LENGTH_RSP,
					//if we get effect Tx/Rx using min(bltData.supportedMaxRxOctets, bltData.connRemoteMaxTxOctets),
					//then we may have an mismatch effect Tx/Rx value.
					if(opcode == LL_LENGTH_RSP){
						 new_effectiveMaxRx = min(bltData.connMaxRxOctets, bltData.connRemoteMaxTxOctets);
						 new_effectiveMaxTx = min(bltData.connMaxTxOctets, bltData.connRemoteMaxRxOctets);
					}

					int change = (bltData.connEffectiveMaxRxOctets != new_effectiveMaxRx) || \
								 (bltData.connEffectiveMaxTxOctets != new_effectiveMaxTx);

					bltData.connEffectiveMaxRxOctets = new_effectiveMaxRx;
					bltData.connEffectiveMaxTxOctets = new_effectiveMaxTx;

					if (opcode == LL_LENGTH_REQ) {
						blt_ll_exchangeDataLength(LL_LENGTH_RSP, bltData.supportedMaxTxOctets);
					}
					bltData.connMaxTxRxOctets_req = DATA_LENGTH_REQ_DONE;

					if ( change && (hci_le_eventMask & HCI_LE_EVT_MASK_DATA_LENGTH_CHANGE) ) {
						u8 result[11];
						hci_le_dataLengthChangeEvt_t *pEvt =
								(hci_le_dataLengthChangeEvt_t *) result;

						pEvt->subEventCode = HCI_SUB_EVT_LE_DATA_LENGTH_CHANGE;
						pEvt->connHandle = BLM_CONN_HANDLE;
						pEvt->maxTxOct = bltData.connEffectiveMaxTxOctets;
						pEvt->maxTxtime = LL_PACKET_OCTET_TIME(bltData.connEffectiveMaxTxOctets);
						pEvt->maxRxOct = bltData.connEffectiveMaxRxOctets;
						pEvt->maxRxtime = LL_PACKET_OCTET_TIME(bltData.connEffectiveMaxRxOctets);

						blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, result, 11);
					}
			#else
					// unknown rsp
					dat16[0] = 0x0203; //type, len
					dat16[1] = LL_UNKNOWN_RSP | (opcode << 8); //feature
					blm_push_fifo(idx, (u8 *) dat16);
			#endif

			}
#endif
			else {//((opcode == LL_CONNECTION_PARAM_REQ) || (opcode == LL_FEATURE_REQ) || (opcode > LL_CMD_MAX) )
				// unknown rsp
				dat16[0] = 0x0203; //type, len
				dat16[1] = LL_UNKNOWN_RSP | (opcode << 8); //feature
				blm_push_fifo(idx, (u8 *) dat16);
			}
		}
		//------------- LL L2CAP single packet ---------------------------------
		else if (type == 2 || type == 1) {
			#if 1//(BQB_5P0_TEST_ENABLE)

				if(blm_crypt_busy == CRYPTE_AFTER_SEND_LL_ENC_REQ && lmp_tick)
				{
					if(lmp_timeout != LMP_PROCEDURE_RESPONSE_TIMEOUT)
						lmp_timeout = LMP_PROCEDURE_RESPONSE_TIMEOUT;
				}
			#if(!ENCRYPT_START_SEND_PROTECT_EN)
				if(blm_crypt_busy == CRYPTE_AFTER_REC_LL_ENC_RSP)
				{
					pc->crypt.mic_fail = 1;
					blm_ll_enc_proc_disconnect(BLM_CONN_HANDLE, HCI_ERR_CONN_TERM_MIC_FAILURE);
					return 0;
				}
			#endif

				if (blc_l2cap_handler) {
					blc_l2cap_handler(BLM_CONN_HANDLE, (raw_pkt+DMA_RFRX_OFFSET_HEADER));
				}
			#else
				if (blc_l2cap_handler) {
					blc_l2cap_handler(BLM_CONN_HANDLE, (raw_pkt+DMA_RFRX_OFFSET_HEADER));
				}
			#endif
		}


	}


	return 0;
}



int blt_send_connection_establish_event(void)
{
	blti.conn_established = 0;

	if (hci_le_eventMask & HCI_LE_EVT_MASK_CONNECTION_ESTABLISH)
	{
		u8 p8[24];
		event_connection_complete_t *pc = (event_connection_complete_t *) p8;

		pc->subcode = HCI_SUB_EVT_LE_CONNECTION_ESTABLISH; // sub code
		pc->status = BLE_SUCCESS; // status: success
		pc->handle = BLM_CONN_HANDLE; // handle
		pc->role = LL_ROLE_MASTER; // master role
		pc->peer_adr_type = pkt_init.rxAddr; // peer address type
		memcpy(pc->mac, pkt_init.advA, 6); // peer address
		pc->interval = pkt_init.interval; // interval
		pc->latency = pkt_init.latency; // latency
		pc->timeout = pkt_init.timeout;
		pc->accuracy = 0; // clock accuracy (only valid for slave)

		blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, p8, 19);
	}

	return 0;
}




int blt_master_main_loop_post(void) {
	u16 dat16[32];
	u8 *p8 = (u8 *) dat16;
	event_connection_complete_t *pc = (event_connection_complete_t *) p8;

	if(blti.conn_established)    //telink private
	{
		blt_send_connection_establish_event();
	}


	if (blm_create_connection & BLM_CONN_HANDLE_CANCEL)
	{
		if (hci_le_eventMask & HCI_LE_EVT_MASK_CONNECTION_COMPLETE)
		{

			pc->subcode = HCI_SUB_EVT_LE_CONNECTION_COMPLETE; // sub code
			pc->status = HCI_ERR_UNKNOWN_CONN_ID; // status: fail
			pc->handle = BLM_CONN_HANDLE; // handle
			pc->role = LL_ROLE_MASTER;
			pc->peer_adr_type = blti.conn_advType; // peer address type
			memcpy(pc->mac, blti.conn_mac, 6); // peer address
			pc->interval = pkt_init.interval; // interval
			pc->latency = pkt_init.latency; // latency
			pc->timeout = pkt_init.timeout;
			pc->accuracy = 0; // clock accuracy (only valid for slave)

			blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, p8, 19);
		}

		blm_create_connection = 0;
	}
	else if (blm_create_connection & BLM_CONN_HANDLE)
	{

		if (hci_le_eventMask & HCI_LE_EVT_MASK_CONNECTION_COMPLETE) {
			pc->subcode = HCI_SUB_EVT_LE_CONNECTION_COMPLETE; // sub code
			pc->status = BLE_SUCCESS; // status: success
			pc->handle = BLM_CONN_HANDLE; // handle
			pc->role = LL_ROLE_MASTER; // master role
			pc->peer_adr_type = pkt_init.rxAddr; // peer address type
			memcpy(pc->mac, pkt_init.advA, 6); // peer address
			pc->interval = pkt_init.interval; // interval
			pc->latency = pkt_init.latency; // latency
			pc->timeout = pkt_init.timeout;
			pc->accuracy = 0; // clock accuracy (only valid for slave)

			blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, p8, 19);
		}


		blm_pconn->crypt.enable = 0;
		blm_pconn->crypt.st = MASTER_LL_ENC_OFF;

		blm_create_connection = 0;

	}


	if (blm_encryption_st & BLM_CONN_ENC_CHANGE)
	{
		if (hci_eventMask & HCI_EVT_MASK_ENCRYPTION_CHANGE)
		{
			event_enc_change_t* pc = (event_enc_change_t*) p8;
			pc->status = blm_pconn->crypt.mic_fail;  //BLE_SUCCESS
			pc->handle = BLM_CONN_HANDLE; // handle
			pc->enc_enable = blm_pconn->crypt.enable;

			blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_ENCRYPTION_CHANGE, p8, 4);
		}

		blm_encryption_st = 0;
	}
	else if (blm_encryption_st & BLM_CONN_ENC_REFRESH)
	{
		if (hci_eventMask & HCI_EVT_MASK_ENCRYPTION_CHANGE)
		{

			event_enc_refresh_t* pc = (event_enc_refresh_t*) p8;
			pc->status = BLE_SUCCESS; // status: success
			pc->handle = BLM_CONN_HANDLE; // handle

			blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_ENCRYPTION_KEY_REFRESH, p8, 3);
		}
		blm_encryption_st = 0;
	}



	if( blti.create_conn_startTick && clock_time_exceed(blti.create_conn_startTick, blm_timeout_connectDevice) )
	{
		if(bltParam.blt_state == BLS_LINK_STATE_INIT )
		{
			if(blts.scan_en){
				blc_ll_setScanEnable(1, blts.filter_dup);
			}
			else{
				blc_ll_setInitEnable(0);
			}
		}

		if (blm_create_connection == BLE_MASTER_CONNECTION_REQ)
		{
			blm_create_connection = 0;
		}

		blti.create_conn_startTick = 0;
	}



	//---------- disconnect ------------------------------------------
	if (blm_disconnect) {

		if (hci_eventMask & HCI_EVT_MASK_DISCONNECTION_COMPLETE)
		{
			p8[0] = 0; // status: OK
			p8[1] = blm_disconnect; // handle
			p8[2] = blm_disconnect >> 8; // handle
			p8[3] = blm[blm_disconnect & 7].conn_terminate_reason; // reason
			blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_DISCONNECTION_COMPLETE, p8, 4);
		}

		master_connecting_tick_flag = 0;
		blm_disconnect = 0;
		blt_ll_version_ind_rsp_flag = 0;

		blc_ll_setEncryptionBusy (0);

		bltData.connEffectiveMaxTxOctets = MAX_OCTETS_DATA_LEN_27;
		bltData.connEffectiveMaxRxOctets = MAX_OCTETS_DATA_LEN_27;
	}

	//---------- connection update complete -----------------------------
	if (blm_conn_update_complete)
	{
		if ((blm_conn_update_complete==BLM_CONN_HANDLE) && (hci_le_eventMask & HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE))
		{
			event_connection_update_t *pc = (event_connection_update_t *) p8;
			st_ll_conn_master_t *pm = &blm[blm_conn_update_complete & 7];
			pc->subcode = HCI_SUB_EVT_LE_CONNECTION_UPDATE_COMPLETE; // sub code
			pc->status = BLE_SUCCESS; // status: success
			pc->handle = blm_conn_update_complete; // handle
			pc->interval = pm->conn_interval; // interval
			pc->latency = pm->conn_latency; // latency
			pc->timeout = pm->conn_timeout/10000;

			blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, p8, 10);
		}

		blm_conn_update_complete = 0;
	}

	//------------security handle proc--------------------------------------------
	blm_ll_security_proc();




	if (blm_pconn->slave_terminate_conn_flag > 2) {
		blm_pconn->slave_terminate_conn_flag = 0;
		blm_pconn->conn_terminate_pending = BLM_CONN_SLAVE_TERMINATE;
	}

#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
	if (bltData.connMaxTxRxOctets_req == DATA_LENGTH_REQ_PENDING) {
		if (0) {
			blt_ll_exchangeDataLength(LL_LENGTH_REQ, bltData.connMaxTxOctets);
		}
	}
#endif


#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	if(blt_conn_phy.phy_update_pending && !blc_ll_isEncryptionBusy() && blttcon.conn_update==0){ //
		u8 phy_update_dat[8];
		rf_pkt_ll_phy_update_ind_t *pUpdt = (rf_pkt_ll_phy_update_ind_t* )phy_update_dat;
		pUpdt->llid = LLID_CONTROL;
		pUpdt->rf_len = 5;
		pUpdt->opcode = LL_PHY_UPDATE_IND;
		pUpdt->m_to_s_phy = pUpdt->s_to_m_phy = blt_conn_phy.conn_update_phy;

		u16 connInst_next  = blttcon.conn_inst + 10 + blm_pconn->conn_latency;
		pUpdt->instant0 = U16_LO(connInst_next);
		pUpdt->instant1 = U16_HI(connInst_next);



		if (blm_push_fifo(BLM_CONN_HANDLE | HANDLE_STK_FLAG, phy_update_dat)) {  //connHandle no use, due to single connection
			blt_conn_phy.phy_update_pending = 0;
			blttcon.conn_update = 0x03;
			blttcon.conn_inst_next = connInst_next;
		}
	}
#endif






	return 0;
}

//////////////////////////////////////////////
// LE link control
//////////////////////////////////////////////


void ble_set_debug_adv_channel(u8 chn) {
	blm_debug_adv_channel = chn;
}

u16 blm_get_interval(u16 min, u16 max) {
	u16 interval = 32;
	if (min <= 32 && 32 <= max) {
		interval = 32;
	} else if (max < 32) {
		interval = min > 8 ? 16 : 8;
	} else if (min > 32) {
		interval = max & 0xffe0; //multiple 32 (x 40ms)
	}
	return interval;
}

int hci_handle_valid(u16 handle) {
	if (!(handle & BLM_CONN_HANDLE)) {
		return 0;
	}
	return 1;
}

ble_sts_t blm_ll_updateConnection(u16 connHandle, u16 conn_min, u16 conn_max,
		u16 conn_latency, u16 timeout, u16 ce_min, u16 ce_max) {
	u8 pkt_conn_para_update[] = { 0x03, //llid
			0x0c, //rf_len
			0x00, //opcode
			BLM_WINSIZE, //winsize		7
			BLM_WINOFFSET, 0, //winoffset 	4
			0, 0x00, //interval		10
			0, 0x00, //latency		12
			0, 0x00, //timeout		14
			0, 0x00, //inst			16
			0, 0, 0, 0 //add 4 byte for encryption
			};
	u16 *pw = (u16 *) pkt_conn_para_update;

	//if (!hci_handle_valid(handle))
	if (0) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}

	st_ll_conn_master_t *pm = &blm[connHandle & 7];

	//-----   decision on interval --------------------
	u16 interval = conn_min;

	//pw[2] = (slot * blm_conn_slot_duration) - 3;			// ??
	pw[3] = interval;
	pw[4] = conn_latency;
	pw[5] = timeout;

	if (pm->conn_latency) { //slave may not rcvd this req when in long suspend
		pw[6] = blttcon.conn_inst + pm->conn_latency + 10;
	} else {
		pw[6] = blttcon.conn_inst + 10;
	}

	pm->conn_latency_next = conn_latency;
	pm->conn_interval_next = interval;
	pm->conn_timeout_next = timeout * 10000;
	blttcon.conn_inst_next = pw[6];

	blm_push_fifo (connHandle | HANDLE_STK_FLAG, pkt_conn_para_update);

	blttcon.conn_update = 2;

	return BLE_SUCCESS;
}

ble_sts_t blm_ll_setHostChannel(u16 handle, u8 * map) {
	u8 pkt_chn_map_req[] =
			{ 0x03, 0x08, 0x01, 0, 0x00, 0x00, 0x00, 0x00, 0, 0 };

	st_ll_conn_master_t *pm = &blm[0];

	if (pm->conn_latency) { //slave may not rcvd this req when in long suspend
		blttcon.conn_inst_next = blttcon.conn_inst + pm->conn_latency + 10;
	} else {
		blttcon.conn_inst_next = blttcon.conn_inst + 10;
	}

	memcpy(blttcon.conn_chn_map_next, map, 5);
	memcpy(pkt_chn_map_req + 3, map, 5);

	memcpy(pkt_chn_map_req + 8, &blttcon.conn_inst_next, 2);
	//check cmd push success
	if(blm_push_fifo(BLM_CONN_HANDLE, pkt_chn_map_req)){
		blttcon.conn_update = 1;
	}
	else{
		return LL_ERR_TX_FIFO_NOT_ENOUGH;
	}

	return BLE_SUCCESS;
}

ble_sts_t blm_ll_readChannelMap(u16 handle, u8 * map) {
	if (!hci_handle_valid(handle)) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
	handle &= 7;
//	st_ll_conn_master_t *pm = &blm[handle];

	memcpy(map, blttcon.conn_chn_map, 5);
	return BLE_SUCCESS;
}

ble_sts_t blm_ll_readRemoteFeature(u16 handle) {
	if (!hci_handle_valid(handle)) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
	handle &= 7;

	u16 dat[6];
	dat[0] = 0x0903; //type, len
	dat[1] = LL_FEATURE_REQ 	| LL_FEATURE_BYTE_0 << 8; //feature
	dat[2] = LL_FEATURE_BYTE_1	| LL_FEATURE_BYTE_2 << 8;
	dat[3] = LL_FEATURE_BYTE_3	| LL_FEATURE_BYTE_4 << 8;
	dat[4] = LL_FEATURE_BYTE_5	| LL_FEATURE_BYTE_6 << 8;
	dat[5] = LL_FEATURE_BYTE_7;
	blm_push_fifo(handle, (u8 *) dat);

	return BLE_SUCCESS;
}

ble_sts_t blm_ll_readRemoteVersion(u16 handle) {
	if (!hci_handle_valid(handle)) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
	handle &= 7;

	u16 dat[4];
	dat[0] = 0x0603; //type, len
	dat[1] = LL_VERSION_IND | (BLUETOOTH_VER << 8);
	dat[2] = VENDOR_ID;
	dat[3] = BLUETOOTH_VER_SUBVER;
	blm[handle].remote_version = 1;
	blm_push_fifo(handle | HANDLE_STK_FLAG, (u8 *) dat);
	return BLE_SUCCESS;
}

ble_sts_t blm_ll_enc_proc_disconnect(u16 handle, u8 reason) {//
	if (!hci_handle_valid(handle)) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
//	handle &= 7;
//	blm[handle].conn_terminate_reason = reason; //terminate reason
	lmp_tick = 0;
	lmp_timeout = 0;
	blm_crypt_busy = CRYPTE_IDLYT;

	blm_disconnect = BLM_CONN_HANDLE;
	blm_pconn->conn_terminate_reason = reason;
	blc_ll_setIdleState();
	return BLE_SUCCESS;
}

ble_sts_t blm_ll_disconnect(u16 handle, u8 reason) {
	u8 pkt_terminate[] = { 0x03, 0x02, 0x02, 0x13 }; //reason 13
	if (!hci_handle_valid(handle)) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
	handle &= 7;

	pkt_terminate[3] = reason;
	blm_push_fifo(handle, pkt_terminate);

#if(BQB_5P0_TEST_ENABLE) //for LL/CON/MAS/BI-02-C [Master T_Terminate Timer]
	blm_pconn->master_teminate_time = clock_time();
#endif
	blm_pconn->master_terminate_conn_flag = 1;
	blm[handle].conn_terminate_reason = reason; //terminate reason
	lmp_tick = 0;
	lmp_timeout = 0;
	blm_crypt_busy = CRYPTE_IDLYT;
	return BLE_SUCCESS;
}


//------------- HCI interface --------------------------------

ble_sts_t blm_hci_reset(void) {


	bltParam.scan_hci_cmd =0;

	blm_pconn->slave_terminate_conn_flag = 0;
	blm_pconn->master_terminate_conn_flag = 0;

	if (blts.scan_en) {
		blc_ll_setScanEnable(0, 0);
	}

	if(blti.init_en){
		blc_ll_setInitEnable(0);
	}


	if (blm_create_connection) {
		blm_create_connection = 0;
	}

	if (blm_disconnect) {
		blm_disconnect = 0;
	}

	if (bltParam.ble_state == BLE_STATE_BTX_S || bltParam.ble_state == BLE_STATE_BTX_E) {
		bltParam.ble_state = 0;

		systimer_clr_irq_status();
		systimer_irq_disable();
		CLEAR_ALL_RFIRQ_STATUS; // clear all interrupt flag
	}

	return BLE_SUCCESS;
}


ble_sts_t blm_hci_receiveHostACLData(u16 connHandle, u8 PB_Flag, u8 BC_Flag, u8 *pData)
{
#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
	if (1) { //IS_LL_CONNECTION_VALID(connHandle)

		u8 len = pData[0]; //core42 dataLen  max 251

		int n = min(len, bltData.connEffectiveMaxTxOctets);
		pData[0] = PB_Flag == HCI_CONTINUING_PACKET ? 1 : 2; //llid
		pData[1] = n;

		blm_push_fifo(BLM_CONN_HANDLE, pData);

		for (int i = n; i < len; i += bltData.connEffectiveMaxTxOctets) {
			n
					= len - i > bltData.connEffectiveMaxTxOctets ? bltData.connEffectiveMaxTxOctets
							: len - i;
			pData[0] = 1; //llid, fregment pkt
			pData[1] = n;
			memcpy(pData + 2, pData + 2 + i, n);
			blm_push_fifo(BLM_CONN_HANDLE, pData);
		}

		return BLE_SUCCESS;
	} else {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}

#else
	return BLE_SUCCESS;
#endif
}

#endif   //end of   LL_MASTER_MUL TI_CONNECTION

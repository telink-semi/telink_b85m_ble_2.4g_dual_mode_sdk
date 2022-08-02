/********************************************************************************************************
 * @file	ll_slave.c
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


#ifndef     DEBUG_TERMINATE_CNT_EN
#define		DEBUG_TERMINATE_CNT_EN			0
#endif


#define    LL_SN_NESN_MANAGE_BY_SOFTWARE				1   //when deep retention back, must use this


// Hop & SCA mask
#define 	LL_CONNECT_REQ_HOP_MASK                     0x1F
#define 	LL_CONNECT_REQ_SCA_MASK                     0xE0

#define		BLT_BRX_TIMING_STATUS_LOST					0
#define		BLT_BRX_TIMING_STATUS_INIT					1
#define		BLT_BRX_TIMING_STATUS_OK					2


#define 	CONN_LONG_TOLERANCE_TIME					(1500 * SYSTEM_TIMER_TICK_1US)


#if (TRY_FIX_ERR_BY_ADD_BRX_WAIT)
    _attribute_data_retention_ //must
	int		CONN_SHORT_TOLERANCE_TIME =	(500 * SYSTEM_TIMER_TICK_1US);
#else
	#define		CONN_SHORT_TOLERANCE_TIME		(500 * SYSTEM_TIMER_TICK_1US)
#endif

#define 	CONN_BRX_WINDOW					(1000 * SYSTEM_TIMER_TICK_1US)


_attribute_data_retention_	_attribute_aligned_(4)	st_ll_conn_slave_t		bltc;
_attribute_data_retention_	u8 restart_encryption = 0;
_attribute_data_retention_	u8 bls_peer_start_enc_flag = 0;

extern u8 blc_tlkEvent_data[];
extern u8 blt_ll_version_ind_rsp_flag;
extern blt_event_callback_t		blt_p_event_callback;
extern ll_module_adv_callback_t	   ll_module_adv_cb;
extern ll_module_adv_callback_t	   ll_module_advSlave_cb;


typedef struct{
	s8		brx_pkt_rcvd;
	s8		brx_pkt_miss;
	u8		blt_brx_synced ;
	u8 		rsvd;
	u32		last_brx;
	int		connection_offset;
	int		brx_ref;
}st_ll_timing_t;

_attribute_data_retention_	_attribute_aligned_(4) st_ll_timing_t bls_timing;

extern u32 lmp_tick;
extern u32 lmp_timeout;



int 	blt_connect (u8 * raw_pkt, bool aux_conn);
int 	blt_ll_channel_conn_update (u8 *pkt);
void 	blt_brx_timing_update (u32 t, int st);

void 	irq_blc_slave_tx(void);
int  	irq_blc_slave_rx_data(u8 *raw_pkt, u32 tick_now);
void 	irq_slave_system_timer(void);
int		blt_slave_main_loop_data (u8 *raw_pkt);
int  	blt_slave_main_loop_post (void);

#if (MCU_CORE_TYPE == MCU_CORE_9518)
	#if PM_32k_RC_CALIBRATION_ALGORITHM_EN
	_attribute_data_retention_	u8 	conn_new_interval_flag  		= 0;
		extern  u32 ble_first_rx_tick_last;
		extern  u32 ble_first_rx_tick_pre;
		extern 	u32 ble_actual_conn_interval_tick;
	#endif
#endif




///// slave module ////

extern ll_irq_tx_callback_t					ll_irq_tx_cb;
extern ll_irq_rx_data_callback_t			ll_irq_rx_data_cb;
extern ll_irq_rx_post_callback_t			ll_irq_rx_post_cb;
extern ll_irq_systemTick_conn_callback_t	ll_irq_systemTick_conn_cb;
extern blc_main_loop_data_callback_t		blc_main_loop_data_cb;
extern blc_main_loop_post_callback_t		blc_main_loop_post_cb;

extern ll_enc_done_callback_t				ll_encryption_done_cb;
extern ll_enc_pause_callback_t				ll_encryption_pause_cb;

extern ll_conn_complete_handler_t			ll_connComplete_handler;
extern ll_conn_terminate_handler_t			ll_connTerminate_handler;

void blc_ll_initSlaveRole_module(void)
{
	ll_adv2conn_cb = blt_connect;   //adv -> slave


	ll_irq_tx_cb = irq_blc_slave_tx;
	ll_irq_rx_data_cb = irq_blc_slave_rx_data;

	ll_irq_systemTick_conn_cb = irq_slave_system_timer;

	blc_main_loop_data_cb = blt_slave_main_loop_data;
	blc_main_loop_post_cb = blt_slave_main_loop_post;

#if (MCU_CORE_TYPE == MCU_CORE_9518)
	bltParam.acl_slave_en = 1;
#endif

	ll_push_tx_fifo_handler = bls_ll_pushTxFifo;

	bltc.connHandle = BLE_INVALID_CONNECTION_HANDLE;

#if (LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN)
	bltParam.tick_LE_Authenticated_Payload = 0;
	bltParam.to_us_LE_Authenticated_Payload = 30000000;  //default 30 S
#endif

	if(blt_miscParam.pad32k_en){
		blc_pm_modefy_brx_early_set(150);//TODO: Total SCA:150ppm: windowWidening = ((masterSCA+slaveSCA)/1000000)*(futureAnchorTime - lastAnchorTime)
	}
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if (MCU_CORE_TYPE == MCU_CORE_9518)
// *((u32 *)blt_tx_empty_packet) = RF_TX_PAKET_DMA_LEN(2) = 0x00 80 00 01
//01 00 80 00 01 00
u8					blt_tx_empty_packet[6] = {0x01, 0x00, 0x80, 0x00, 0x01, 0x00};   //never change, so no need retention
#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
u8					blt_tx_empty_packet[6] = {2, 0, 0, 0, 1, 0};   //never change, so no need retention
#endif




_attribute_data_retention_	ble_crypt_para_t 	blc_cyrpt_para = {0,};


#define TERMINTE_CMD_IDLE							0
#define TERMINTE_CMD_IN_FIFO						1


/*
 * API for register call for get ltk from hci or Telink_host directly.
 * */

_attribute_data_retention_	blt_LTK_req_callback_t blt_ltk_request = NULL;   // shall init to sm callback
void blc_ll_registerLtkReqEvtCb(blt_LTK_req_callback_t evtCbFunc)
{
	blt_ltk_request = evtCbFunc;
}




_attribute_data_retention_
rf_packet_ll_terminate_t	pkt_slave_terminate = {
		0x03,						// type
		2,							//rf_len
		LL_TERMINATE_IND,
		HCI_ERR_REMOTE_USER_TERM_CONN,
};





ble_sts_t	bls_hci_receiveHostACLData(u16 connHandle, u8 PB_Flag, u8 BC_Flag, u8 *pData )
{
#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
	if( IS_LL_CONNECTION_VALID(connHandle) ){
		u8 len = pData[0];  //core42 dataLen  max 251

		int n = min(len, bltData.connEffectiveMaxTxOctets);
		pData[0] = PB_Flag == HCI_CONTINUING_PACKET ? 1 : 2;         //llid
		pData[1] = n;
		bls_ll_pushTxFifo (BLS_CONN_HANDLE, pData);

		for (int i=n; i<len; i+=bltData.connEffectiveMaxTxOctets)
		{
			n = len - i > bltData.connEffectiveMaxTxOctets ? bltData.connEffectiveMaxTxOctets : len - i;
			pData[0] = 1; //llid, fregment pkt
			pData[1] = n;
			smemcpy (pData + 2, pData + 2 + i, n);
			bls_ll_pushTxFifo (BLS_CONN_HANDLE, pData);
		}

		return BLE_SUCCESS;
	}
	else{
		return HCI_ERR_UNKNOWN_CONN_ID;
	}

#else
	if( IS_LL_CONNECTION_VALID(connHandle) ){
		pData[1] = pData[0];
		pData[0] = PB_Flag == HCI_CONTINUING_PACKET ? 1 : 2;         //llid
		bls_ll_pushTxFifo (BLS_CONN_HANDLE, pData);
		return BLE_SUCCESS;
	}
	else{
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
#endif
}








ble_sts_t  bls_hci_reset(void)
{

	bltParam.adv_hci_cmd = 0;

	if(bltParam.blt_state == BLS_LINK_STATE_CONN){
		bls_ll_terminateConnection(HCI_ERR_CONN_TERM_BY_LOCAL_HOST);

		bltParam.adv_en = 0;
	}



	return BLE_SUCCESS;
}





ble_sts_t  bls_ll_enc_proc_disConnect (u8 reason)
{

	if(bltParam.blt_state != BLS_LINK_STATE_CONN){
		return LL_ERR_CONNECTION_NOT_ESTABLISH;
	}
	lmp_tick = 0;
	lmp_timeout = 0;
	bls_peer_start_enc_flag = 0;
	pkt_slave_terminate.reason = reason;
	bltc.conn_terminate_pending = 2;
	bltc.conn_terminate_reason = reason;
	blt_terminate_proc();
	return BLE_SUCCESS;
}





ble_sts_t  bls_ll_terminateConnection (u8 reason)
{

	if(bltParam.blt_state != BLS_LINK_STATE_CONN){
		return LL_ERR_CONNECTION_NOT_ESTABLISH;
	}

	pkt_slave_terminate.reason = reason;

	if(bltc.conn_slave_terminate == TERMINTE_CMD_IDLE && bls_ll_pushTxFifo(BLS_CONN_HANDLE | HANDLE_STK_FLAG, (u8*)&pkt_slave_terminate) ){
		bltc.conn_slave_terminate = TERMINTE_CMD_IN_FIFO;
		bltc.conn_terminate_pending = 1;
		bltc.conn_slaveTerminate_time = clock_time();


		return BLE_SUCCESS;
	}
	else{
		return HCI_ERR_CONTROLLER_BUSY;  //err
	}
}


ble_sts_t  blt_disconnect(u16 connHandle, u8 reason)
{
	if( IS_LL_CONNECTION_VALID(connHandle) ){
		return bls_ll_terminateConnection(reason);
	}
	else{
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
}






u16	bls_ll_getConnectionInterval(void)
{
	if(bltParam.blt_state == BLS_LINK_STATE_CONN){
		return  bltc.conn_interval/SYSTEM_TIMER_TICK_1250US;
	}
	else{
		return 0;
	}
}

u16	bls_ll_getConnectionLatency(void)
{
	if(bltParam.blt_state == BLS_LINK_STATE_CONN){
		return bltc.conn_latency;
	}
	else{
		return 0;
	}
}

u16	bls_ll_getConnectionTimeout(void)
{
	if(bltParam.blt_state == BLS_LINK_STATE_CONN){
		return bltc.conn_timeout/(10000 * SYSTEM_TIMER_TICK_1US);
	}
	else{
		return 0;
	}
}



extern void aes_ll_ccm_encryption_init (u8 *ltk, u8 *skdm, u8 *skds, u8 *ivm, u8 *ivs, ble_crypt_para_t *pd);
extern void aes_ll_ccm_encryption(u8 *pkt, int master, ble_crypt_para_t *pd);
extern int  aes_ll_ccm_decryption(u8 *pkt, int master, ble_crypt_para_t *pd);


#if(MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
	bool    bls_ll_pushTxFifo (int handle, u8 *p)
	{
		u8 r = irq_disable();

		int n = ((reg_dma_tx_wptr - reg_dma_tx_rptr) & 15 )  +  ( (blt_txfifo.wptr - blt_txfifo.rptr) & 31 ) ;

		irq_restore(r);



		if ( n >= ((handle & HANDLE_STK_FLAG) ? blt_txfifo.num : blt_txfifo.num - 2) ) {
			return 0;
		}
		u8 *pd = (u8 *)blt_txfifo_b + (blt_txfifo.wptr & (blt_txfifo.num-1)) * blt_txfifo.size;
		memcpy (pd + 4, p, p[1] + 2);


		if (blc_cyrpt_para.enable)
		{
			bltc.conn_enc_dec_busy = 1;
			//16m, payload:251byte, AES_CCM need 1.498ms
			aes_ll_ccm_encryption ( pd + 4, blc_cyrpt_para.enable == 2, &blc_cyrpt_para);
			bltc.conn_enc_dec_busy = 0;

			#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE) ///// proc master update_req in irq ///////
				if (bltc.conn_pkt_dec_pending)
				{
					blt_ll_channel_conn_update (bltc.conn_pkt_dec_pending);
					bltc.conn_pkt_dec_pending = 0;
				}
			#endif
		}

	#if (1) // support RF RX/TX MAX data Length: 251byte
		pd[0] = (pd[5] + 2);    //DMA length low value
		pd[1] = (pd[5] + 2)>>8; //DMA length high value
	#else
		pd[0] = (pd[5] + 2);
	#endif

		blt_txfifo.wptr++;
		return 1;
	}
#elif(MCU_CORE_TYPE == MCU_CORE_9518)
	bool    bls_ll_pushTxFifo (int handle, u8 *p)
	{

		u32 r = irq_disable();

		int hw_fifo_n = ((reg_dma_tx_wptr - reg_dma_tx_rptr) & 31 );
		int	sw_fifo_n = ((blt_txfifo.wptr - blt_txfifo.rptr) & 31 );
		int total_fifo_n = sw_fifo_n + hw_fifo_n;


		irq_restore(r);

		//consider whether need leave one space for empty packet
		int empty_space = hw_fifo_n ? 0 : 1;


		if ( total_fifo_n >= ((handle & HANDLE_STK_FLAG) ? (blt_txfifo.num - empty_space) : (blt_txfifo.num - empty_space - BLE_STACK_USED_TX_FIFIO_NUM)) ) {
			return 0;
		}


		u8 *pd = (u8 *)blt_txfifo.p_base + ( blt_txfifo.wptr & blt_txfifo.mask) * blt_txfifo.size;
		smemcpy (pd + 4, p, p[1] + 2);


		if (blc_cyrpt_para.enable)
		{
			bltc.conn_enc_dec_busy = 1;
			//16m, payload:251byte, AES_CCM need 1.498ms
			aes_ll_ccm_encryption ( pd + 4, blc_cyrpt_para.enable == 2, &blc_cyrpt_para);
			bltc.conn_enc_dec_busy = 0;

			#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE) ///// proc master update_req in irq ///////
				if (bltc.conn_pkt_dec_pending)
				{
					blt_ll_channel_conn_update (bltc.conn_pkt_dec_pending);
					bltc.conn_pkt_dec_pending = 0;
				}
			#endif
		}


		*(u32 *)pd = RF_TX_PAKET_DMA_LEN(pd[5] + 2);

		blt_txfifo.wptr += 1;

	#if (TX_FIFO_DBG_EN)
		r = irq_disable();

		log_b16_irq (TX_FIFO_DBG_EN, SL16_tf_hw_push, reg_dma_tx_wptr<<8 | reg_dma_tx_rptr);
		log_b16_irq (TX_FIFO_DBG_EN, SL16_tf_sw_push, blt_txfifo.wptr<<8 | blt_txfifo.rptr);
		log_tick_irq(TX_FIFO_DBG_EN, SLEV_txFifo_push);

		irq_restore(r);
	#endif

		return 1;
	}
#endif




#if(MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
_attribute_ram_code_
void blt_push_fifo_hw ()
{
#if (LL_SN_NESN_MANAGE_BY_SOFTWARE)
	blt_restore_snnesn ();
	blt_bb.save_flg = 0;
	blt_restore_dma_tx_rptr();
#endif

#if (DEEP_RET_ENTRY_CONDITIONS_OPTIMIZATION_EN)
	//deep ret back condition:
	//	MCU core register will power off during deep retention condition, after deep ret up,the DMA
	//	TX read and write pointer register value is 0, so special consideration is required here!!!
	int n = 0;

	if( pmParam.is_deepretn_back)
	{
		//here: reg_dma_tx_rptr = reg_dma_tx_wptr = blt_bb.blt_dma_tx_rptr = 0

		if(blt_bb.dma_tx_rcvry_dat_depth){
			n = blt_bb.dma_tx_data_depth;
			for(int i = 0; i< n; i++){
				reg_dma_tx_fifo = blt_bb.dma_tx_ptr[i];
			}

			blt_bb.blt_dma_tx_rptr = (blt_bb.dma_tx_data_depth-blt_bb.dma_tx_rcvry_dat_depth)&15;
			reg_dma_tx_rptr = (BIT(6) | blt_bb.blt_dma_tx_rptr);//restore tx_rptr

		}
	}
	else //when suspend or normal conditions
	{
		#if (FIX_HW_CRC24_EN)
			blt_restore_dma_tx_rptr();
		#endif

		n = (reg_dma_tx_wptr - reg_dma_tx_rptr) & 15;
	}

	u16 tx_ptr_tmp  = 0;
	if (n == 0 && blt_txfifo.rptr != blt_txfifo.wptr)
	{
		tx_ptr_tmp = (u16)(u32) blt_tx_empty_packet;
		reg_dma_tx_fifo = tx_ptr_tmp;
		blt_bb.dma_tx_ptr[n] = tx_ptr_tmp;
		n = 1;
	}
	while (blt_txfifo.rptr != blt_txfifo.wptr && n < 7)
	{
		tx_ptr_tmp = (u16)(u32)(blt_txfifo_b + (blt_txfifo.rptr++ & (blt_txfifo.num-1)) * blt_txfifo.size);
		reg_dma_tx_fifo = tx_ptr_tmp;
		blt_bb.dma_tx_ptr[n] = tx_ptr_tmp;
		n++;
	}

	blt_bb.dma_tx_data_depth = n;

#else


	int n = (reg_dma_tx_wptr - reg_dma_tx_rptr) & 15;
	if (n == 0 && blt_txfifo.rptr != blt_txfifo.wptr)
	{
		reg_dma_tx_fifo = (u16)(u32) blt_tx_empty_packet;
		n = 1;
	}
	while (blt_txfifo.rptr != blt_txfifo.wptr && n < 7)
	{
		reg_dma_tx_fifo = (u16)(u32)(blt_txfifo_b + (blt_txfifo.rptr++ & (blt_txfifo.num-1)) * blt_txfifo.size);
		n++;
	}
#endif
}

#elif(MCU_CORE_TYPE == MCU_CORE_9518)
_attribute_ram_code_
void blt_push_fifo_hw ()
{
#if (LL_SN_NESN_MANAGE_BY_SOFTWARE)
	blt_restore_snnesn ();
	blt_bb.save_flg = 0;
	blt_restore_dma_tx_ptr();
#endif


	log_b16_irq (TX_FIFO_DBG_EN, SL16_tf_hw_load1, reg_dma_tx_wptr<<8 | reg_dma_tx_rptr);
	log_b16_irq (TX_FIFO_DBG_EN, SL16_tf_sw_load1, blt_txfifo.wptr<<8 | blt_txfifo.rptr);


	int hw_fifo_n = (reg_dma_tx_wptr - reg_dma_tx_rptr) & 31;
	int sw_fifo_n = (blt_txfifo.wptr - blt_txfifo.rptr) & 31;


//	//TODO debug, will delete later
//	if(sw_fifo_n >15 ){
//		write_sram8(0x80000, 0x11);
//		while(1);
//		write_sram8(0x80000, 0x22);
//	}


	if (hw_fifo_n == 0 && sw_fifo_n != 0)
	{
		//insert empty packet
		//1. empty packet no need encryption
		//2. empty packet dma_len has already calculated

		u8 *p = (u8 *)blt_txfifo.p_base + ((blt_txfifo.rptr - 1) & blt_txfifo.mask) * blt_txfifo.size;
		smemcpy(p, blt_tx_empty_packet, 6);
		reg_dma_tx_rptr = (FLD_DMA_RPTR_SET | ((blt_txfifo.rptr - 1) & 31));
		reg_dma_tx_wptr = reg_dma_tx_rptr + 1;  // & 31

		log_event_irq(TX_FIFO_DBG_EN, SLEV_txFifo_empty);
	}

	reg_dma_tx_wptr += sw_fifo_n;
	blt_txfifo.rptr += sw_fifo_n;



	log_b16_irq (TX_FIFO_DBG_EN, SL16_tf_hw_load2, reg_dma_tx_wptr<<8 | reg_dma_tx_rptr);
	log_b16_irq (TX_FIFO_DBG_EN, SL16_tf_sw_load2, blt_txfifo.wptr<<8 | blt_txfifo.rptr);
	log_tick_irq(TX_FIFO_DBG_EN, SLEV_timestamp);

}
#endif


#if (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
	_attribute_data_retention_ static u32 bls_brxDisableNearestBrxT;
	_attribute_data_retention_	u8 brx_disable_threhold = 3;

	u8 set_disBrxRequest_thresold(u8 thres)
	{
		if( (thres>0)&&(thres<7) )
		{
			brx_disable_threhold = thres;
			return 0;
		}
		return 1;
	}

	int		bls_ll_requestConnBrxEventDisable(void)  //ms
	{
		//conn state
		if(bltParam.blt_state != BLS_LINK_STATE_CONN){  //none conn state
			return U16_MAX;
		}

		irq_disable();

		int ret = 0;

		#if (0) //connection establishment is less than 2.5s, then return 0.
			if(0 && !clock_time_exceed(bls_ll_getConnectionCreateTime(), 25000)){

			}
			else
		#endif
		//during brx_working stage: return 0.
		if(bltParam.blt_busy || bltParam.ble_state == BLE_STATE_BRX_S || (1 && bls_timing.brx_pkt_rcvd < brx_disable_threhold)){  //brx working period or connection unstable

		}
		else if(blttcon.conn_update){  //update req
			//plus 2 interval for boundary condition compensation by tuyf 180607
			if( (s32)(blttcon.conn_inst_next - blttcon.conn_inst - 2) > 0 && \
				((u32) ( systimer_get_irq_capture() - (clock_time() + (SYSTEM_TIMER_TICK_1US<<9)))<BIT(30) )	){
				ret = (blttcon.conn_inst_next - blttcon.conn_inst - 2 ) * (bltc.conn_interval/(1000 * SYSTEM_TIMER_TICK_1US));
				//situation: interval = 1s and inst_next-inst is 8, it will return 6s.connection can disconnect due to timeout.
				s32 tmpHalfTimeOut = (bltc.conn_timeout>>1)/SYSTEM_TIMER_TICK_1MS;
				ret = min(ret, tmpHalfTimeOut);
			}
		}
		else{
			u32 delta_tick = (u32) ( systimer_get_irq_capture() - (clock_time() + (SYSTEM_TIMER_TICK_1US<<9)) ); //512 us bandwith
			if(delta_tick < BIT(30)){
				ret =   bltc.conn_timeout/(2000 * SYSTEM_TIMER_TICK_1US);  //timeout/2
			}
		}

		irq_enable();

		return ret;
	}

	void	bls_ll_disableConnBrxEvent(void)
	{
		irq_disable();

		systimer_irq_disable();

		bls_brxDisableNearestBrxT = reg_system_tick_irq;

		reg_system_tick_irq ^= BIT(30);

		irq_enable();
	}

	void	bls_ll_restoreConnBrxEvent(void)
	{
		irq_disable();

		u32 tick_now = clock_time();
		u32 delta_T = (u32)(tick_now - bls_brxDisableNearestBrxT);
		u32 jump_num = 0;

		if(delta_T > BIT(30)){
			//restoreBrxTick - disableBrxTick < T(one interval), do nothing
			reg_system_tick_irq = bls_brxDisableNearestBrxT;
			systimer_irq_enable();
			bltPm.latency_off = 1;
		}
		else if(bltParam.blt_state == BLS_LINK_STATE_CONN){
			//extend the BRX window to the maximum
			bltc.conn_tolerance_time = bltc.interval_level * CONN_LONG_TOLERANCE_TIME + CONN_SHORT_TOLERANCE_TIME;
			bltc.conn_duration = CONN_BRX_WINDOW + bltc.conn_tolerance_time * 2;

			bls_timing.brx_ref = CONN_LONG_TOLERANCE_TIME;
			bls_timing.brx_pkt_miss = bltc.interval_level;
			if (bls_timing.brx_pkt_rcvd > 1) {
				bls_timing.brx_pkt_rcvd -= 2;
			}

			u32 interval_cycle = delta_T/bltc.conn_interval;
			u32 interval_mod = delta_T - interval_cycle*bltc.conn_interval;

			if(interval_mod >= bltc.conn_interval){
				jump_num = interval_cycle + 2;
			}
			else{
				jump_num = interval_cycle + 1;
			}

			bltc.connExpectTime += (bltc.conn_interval*jump_num);
			blttcon.conn_inst += jump_num;
			blttcon.chn_idx = (blttcon.chn_idx+jump_num)%37;

			//Here: clear the system timer interrupt flag first, and then set the tick value.
			//On the contrary, there is a problem that the system clock value reg_system_tick_irq is just set near the dangerous area near the time
			//series recovery point, and the tick that reaches the system clock setting is generated at this time. The system clock flag in reg_irq_src
			//is set, which is just cleared. Even if reg_irq_mask turns on the system timer is set, no interrupt will be generated.
			//systimer_clr_irq_status();
			systimer_set_irq_capture(bltc.connExpectTime - bltc.conn_tolerance_time);
			systimer_irq_enable();


			bltPm.latency_off = 1;
		}
		irq_enable();
	}
#endif


/////////////////////////////////////////////////////////////////
//		link layer security
/////////////////////////////////////////////////////////////////
_attribute_data_retention_	u32 enc_skdm[2] = {0xE0F10213, 0xACBDCEDF}; // 0xACBDCEDF E0F10213 (MSO to LSO)
_attribute_data_retention_	u32	enc_ivm     = 0xBADCAB24;
_attribute_data_retention_	u32 enc_skds[2] = {0xae86ede8, 0x00e3635a}; // 0x00e3635a ae86ede8 (MSO to LSO)
_attribute_data_retention_	u32 enc_ivs     = 0x92ad77e1;
_attribute_data_retention_	u32 enc_ltk[4]  = {0};

_attribute_data_retention_	u8 	blt_smp_empty_pkt_entry = 0;




void			bls_ll_pushEncPkt (int type)
{
	u8			pkt_enc[28];

	pkt_enc[0] = 0x03;

	if (type == LL_ENC_RSP)
	{
		pkt_enc[1] = 0x0d;
		smemcpy(pkt_enc + 3, enc_skds, 8);
		smemcpy(pkt_enc + 11, &enc_ivs, 4);
	}
#if 0
	else if (type == LL_ENC_REQ)
	{
		pkt_enc[1] = 0x17;
		smemcpy (pkt_enc + 3, enc_rand, 8);
		smemcpy (pkt_enc + 11, &enc_ediv, 2);
		smemcpy (pkt_enc + 13, enc_skdm, 8);
		smemcpy (pkt_enc + 21, &enc_ivm, 4);
	}
#endif
	else if (type == LL_REJECT_IND)
	{
		pkt_enc[1] = 0x2;
		pkt_enc[3] = HCI_ERR_PIN_KEY_MISSING;					//PIN missing
	}
#if (LL_FEATURE_ENABLE_EXTENDED_REJECT_INDICATION)
	else if (type == LL_REJECT_IND_EXT)
	{
		pkt_enc[1] = 0x3;
		pkt_enc[3] = LL_ENC_REQ;    //Attention: Now support "LL_ENC_REQ" reject only, maybe some other case in future
		pkt_enc[4] = HCI_ERR_PIN_KEY_MISSING;					//PIN missing
	}
#endif
	else
	{
		pkt_enc[1] = 1;
	}

	pkt_enc[2] = type;

	bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, pkt_enc);
}



void  bls_ll_security_proc (void)
{
#if (MCU_CORE_TYPE == MCU_CORE_9518)
	if( blt_ll_getRealTxFifoNumber () != 0 )
#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
	if( blc_ll_getTxFifoNumber() != 0 )
#endif
	{

	}
	else if (blc_cyrpt_para.st == SLAVE_LL_ENC_RSP_T)
	{
		blc_cyrpt_para.st = SLAVE_LL_ENC_START_REQ_T;
		bls_ll_pushEncPkt (LL_START_ENC_REQ);


		lmp_timeout = LMP_PROCEDURE_RESPONSE_TIMEOUT;
		lmp_tick = clock_time()|1;

		bltc.conn_pkt_rcvd = 0;
		blc_cyrpt_para.enable = 1;												//
		aes_ll_ccm_encryption_init ( (u8*)enc_ltk, (u8*)enc_skdm, (u8*)enc_skds, (u8*)&enc_ivm, (u8*)&enc_ivs, &blc_cyrpt_para);
	}
	else if (blc_cyrpt_para.st == SLAVE_LL_REJECT_IND_T)
	{
		blc_cyrpt_para.st = SLAVE_LL_ENC_OFF;
		blt_smp_empty_pkt_entry = 0;
		blc_ll_setEncryptionBusy (0);   //no ltk match, must clear enc busy flag

		u8 reject_cmd;
		if( LL_FEATURE_ENABLE_EXTENDED_REJECT_INDICATION && (bltc.ll_remoteFeature & LL_FEATURE_MASK_EXTENDED_REJECT_INDICATION) ){
			reject_cmd = LL_REJECT_IND_EXT;
		}
		else{
			reject_cmd = LL_REJECT_IND;
		}
		bls_ll_pushEncPkt (reject_cmd);
		bls_peer_start_enc_flag = 0;
		blt_p_event_callback (BLT_EV_FLAG_LL_REJECT_IND, (u8 *)&reject_cmd, 1);
	}
	else if (blc_cyrpt_para.st == SLAVE_LL_ENC_START_RSP_T)
	{
		blt_smp_empty_pkt_entry = 0;
	}
}

/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
int blt_ll_channel_conn_update (u8 *pkt)
{
	int st = 0;
	u32 bak;
	if (blc_cyrpt_para.enable)
	{
		bak = blc_cyrpt_para.dec_pno;
		blc_cyrpt_para.dec_pno = bltc.conn_pkt_rcvd_no;
		st = aes_ll_ccm_decryption (pkt, 1, &blc_cyrpt_para);
		blc_cyrpt_para.dec_pno = bak;							//restore orignal value

	}

#if 0
	u8 pkt_notify[] = {2, 11, 7, 0, 4, 0, 0x1b, 0x0d, 0x00,   0x01, 0x02, 0x03, 0x4};
	pkt_notify[9] = bltc.conn_pkt_rcvd_no;
	pkt_notify[10] = bak;
	bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, pkt_notify);
#endif
	if (!st)
	{
		rf_packet_ll_control_t *pll = (rf_packet_ll_control_t *)pkt;

		s16 diff_inst;

		if (pll->opcode == LL_CHANNEL_MAP_REQ)	//update channel map
		{

			blttcon.conn_inst_next = *(u16 *)(pkt + 8);
			diff_inst = blttcon.conn_inst_next - blttcon.conn_inst;
			if(diff_inst > 0){
				bltc.master_not_ack_slaveAckUpReq = 1;
				blttcon.conn_update = 1;
				smemcpy (blttcon.conn_chn_map_next, pkt + 3, 5);
				blc_tlkEvent_pending |= EVENT_MASK_CHN_MAP_REQ;
			}
		}
		else if (pll->opcode == LL_CONNECTION_UPDATE_REQ)
		{
			rf_packet_ll_updateConnPara_t *pUpdate = (rf_packet_ll_updateConnPara_t *)pkt;

			blttcon.conn_inst_next = pUpdate->instant;
			diff_inst = blttcon.conn_inst_next - blttcon.conn_inst;
			if(diff_inst > 0){
				bltc.master_not_ack_slaveAckUpReq = 1;
				blttcon.conn_update = 2;
				bltc.conn_winsize_next = pUpdate->winSize;
				bltc.conn_offset_next   = pUpdate->winOffset;
				bltc.conn_interval_next = pUpdate->interval;
				bltc.conn_latency_next = pUpdate->latency;
				bltc.conn_timeout_next = pUpdate->timeout;

				smemcpy(blc_tlkEvent_data, pkt, 14);
				blc_tlkEvent_pending |= EVENT_MASK_CONN_PARA_REQ;
			}
		}
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
		else if (pll->opcode == LL_PHY_UPDATE_IND )	//LL Control Opcode: 0x18
		{
			rf_pkt_ll_phy_update_ind_t* pUpdt = (rf_pkt_ll_phy_update_ind_t *)(pkt);


			u8 comm_phy = pUpdt->m_to_s_phy & pUpdt->s_to_m_phy;

			u16 conn_inst_next = pUpdt->instant0 | (pUpdt->instant1<<8);
			s16 diff_inst = conn_inst_next - blttcon.conn_inst;
			if(diff_inst > 0){
				bltc.master_not_ack_slaveAckUpReq = 1;
				blttcon.conn_update = 3;
				blttcon.conn_inst_next = conn_inst_next;
				if(comm_phy & PHY_PREFER_1M){
					blt_conn_phy.conn_next_phy = BLE_PHY_1M;
				}
				else if(comm_phy & PHY_PREFER_2M){
					blt_conn_phy.conn_next_phy = BLE_PHY_2M;
				}
				else if(comm_phy & PHY_PREFER_CODED){
					blt_conn_phy.conn_next_phy = BLE_PHY_CODED;
				}
				else{  //no PHY Update
					blt_conn_phy.conn_next_phy = blt_conn_phy.conn_cur_phy;
				}
			}
			else //todo wrap around
			{
				//bls_ll_terminateConnection(HCI_ERR_INSTANT_PASSED);
				//hci_disconnectionComplete_evt(0,bltc.connHandle,HCI_ERR_INSTANT_PASSED);
			}
		}
#endif

		else{
			//debug
		}
	}
	else
	{
		blc_cyrpt_para.mic_fail = 1;
	}
	return st;
}



void bls_ll_send_read_remote_feature_event(u8 status)
{
	if(hci_le_eventMask & HCI_LE_EVT_MASK_READ_REMOTE_FEATURES_COMPLETE){
		u8 remoteFeature[8] = {0};
		smemcpy(remoteFeature, (u8*)&bltc.ll_remoteFeature, 4);

		u8 result[12];
		hci_le_readRemoteFeaturesCompleteEvt_t *pEvt = (hci_le_readRemoteFeaturesCompleteEvt_t *)result;

		pEvt->subEventCode = HCI_SUB_EVT_LE_READ_REMOTE_USED_FEATURES_COMPLETE;
		pEvt->status = status;
		pEvt->connHandle = bltc.connHandle;
		smemcpy(pEvt->feature, remoteFeature, LL_FEATURE_SIZE);

		blc_hci_send_event (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, result, 12);
	}
}

int hci_slave_handle_valid(u16 handle) {
	if (!(handle & BLS_CONN_HANDLE)) {
		return 0;
	}
	return 1;
}

/**
 * @brief     for user to send LL_VERSION_IND.
 * @param     connHandle: BLS_CONN_HANDLE indicate slave role;
 * @return    status, 0x00 : succeed
 * 					  other: failed
 */
ble_sts_t 	bls_ll_readRemoteVersion(u16 connHandle)
{
	if (!hci_slave_handle_valid(connHandle)) {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}

	rf_packet_version_ind_t versionInd;

	versionInd.type		= LLID_CONTROL;
	versionInd.rf_len	= 6;
	versionInd.opcode	= LL_VERSION_IND;
	versionInd.versNr	= BLUETOOTH_VER;
	versionInd.compId	= VENDOR_ID;
	versionInd.subVersNr= BLUETOOTH_VER_SUBVER;

	bls_ll_pushTxFifo(connHandle|HANDLE_STK_FLAG, (u8*)&versionInd); //ll_push_tx_fifo_handler :blm_push_fifo   ;;; bls_ll_pushTxFifo

	bltc.remoteVersionFlag = 1;
	return BLE_SUCCESS;
}



int blt_ll_packet_proc (u8 *p)
{


	u16 dat[8];
//	u8 *d8 = (u8 *)dat;
	//////////// decrypt packet if security on ////////////////////////
	if(p[5]) {
		if (blc_cyrpt_para.enable)
		{

			#if (LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN)
				if(bltParam.enable_LE_Authenticated_Payload){
					bltParam.tick_LE_Authenticated_Payload = clock_time() | 1;
				}
			#endif

			bltc.conn_enc_dec_busy = 1;
			int st = aes_ll_ccm_decryption (p + 4, 1, &blc_cyrpt_para);
			bltc.conn_enc_dec_busy = 0;

			#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE) /////// proc master update_req in irq ///////
				if (bltc.conn_pkt_dec_pending)
				{
					st = blt_ll_channel_conn_update (bltc.conn_pkt_dec_pending);
					bltc.conn_pkt_dec_pending = 0;
				}
			#endif


			//decrypt packet from master
			if( st )  //return 0 is OK
			{
				blc_cyrpt_para.mic_fail = 1;
				if(bls_peer_start_enc_flag){
					bls_ll_enc_proc_disConnect(HCI_ERR_CONN_TERM_MIC_FAILURE);
				}
				else{
					bls_ll_terminateConnection(HCI_ERR_CONN_TERM_MIC_FAILURE);
				}
				return 0;
			}
		}
	}


	//////////////////////////////////////
	if (p[5])
	{
		u8 opcode = p[6];
		rf_packet_ll_control_t *pll = (rf_packet_ll_control_t *)(p + 4);
		u8 type = p[4]&3;
		if(bls_peer_start_enc_flag){//master enter enc procedure.

			if((type == LLID_CONTROL) && ((opcode == LL_TERMINATE_IND) || (opcode == LL_ENC_REQ) || \
			                              (opcode == LL_START_ENC_RSP) || (opcode == LL_REJECT_IND_EXT) || (opcode == LL_REJECT_IND))){

			}

			else{//unexpected pdu,slave shall terminate.
				blc_cyrpt_para.mic_fail = 1;
				bls_ll_enc_proc_disConnect(HCI_ERR_CONN_TERM_MIC_FAILURE);
				return 0;
			}
		}

		///////////////////////////////////////////////////////////////////
		if (type == LLID_CONTROL)					//LL control
		{

			extern u8 cmd_length_array[];
			if(pll->rf_len != cmd_length_array[opcode])
			{
				blt_ll_unknown_rsp(BLS_CONN_HANDLE | HANDLE_STK_FLAG, opcode);
				return 0;
			}


#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE)			// LL Control Opcode: 0x00/0x01
			if (opcode == LL_CONNECTION_UPDATE_REQ){

			}
			else if(opcode == LL_CHANNEL_MAP_REQ){

			}

#else
			if (opcode == LL_CONNECTION_UPDATE_REQ)
			{
				rf_packet_ll_updateConnPara_t *pUpdate = (rf_packet_ll_updateConnPara_t *)(p+DMA_RFRX_OFFSET_HEADER);
				blttcon.conn_inst_next = pUpdate->instant;
				s16 diff_inst = blttcon.conn_inst_next - blttcon.conn_inst;
				if(diff_inst > 0){
					bltc.master_not_ack_slaveAckUpReq = 1;
					blttcon.conn_update = 2;
					bltc.conn_winsize_next = pUpdate->winSize;
					bltc.conn_offset_next   = pUpdate->winOffset;
					bltc.conn_interval_next = pUpdate->interval;
					bltc.conn_latency_next = pUpdate->latency;
					bltc.conn_timeout_next = pUpdate->timeout;

					smemcpy(blc_tlkEvent_data, p + 4, 14);
					blc_tlkEvent_pending |= EVENT_MASK_CONN_PARA_REQ;
				}
				#if 0
				else //todo wrap around
				{
					bltc.conn_terminate_pending = 2;  //slave can disconnect now
					bltc.conn_terminate_reason = HCI_ERR_INSTANT_PASSED;
					hci_disconnectionComplete_evt(0,bltc.connHandle,HCI_ERR_INSTANT_PASSED);
				}
				#endif
			}
			else if (opcode == LL_CHANNEL_MAP_REQ)	//update channel map
			{
				blttcon.conn_update = 1;
				blttcon.conn_inst_next = *(u16 *)(p + 12);

				s16 diff_inst = blttcon.conn_inst_next - blttcon.conn_inst;
				if(diff_inst > 0){
					bltc.master_not_ack_slaveAckUpReq = 1;
					blttcon.conn_update = 1;
					smemcpy (blttcon.conn_chn_map_next, p + 7, 5);
					blc_tlkEvent_pending |= EVENT_MASK_CHN_MAP_REQ;
				}
				#if 0
				else //todo wrap around
				{
					bls_ll_terminateConnection(HCI_ERR_INSTANT_PASSED);
				}
				#endif
			}
#endif  // end of BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE
			else if (opcode == LL_TERMINATE_IND ){ // termination			//LL Control Opcode: 0x02
				bltc.conn_master_terminate = 1;  //mark
				bltc.conn_terminate_reason = p[7];  //ErrorCode
				bltc.conn_terminate_pending = 1;
			}
#if (LL_FEATURE_ENABLE_LE_ENCRYPTION)	//LL Control Opcode: 0x03/0x04/0x05/0x06,  0x0A/0x0B
			else if (opcode == LL_ENC_REQ)					//ll_enc_req rx =>  tx (rsp) => tx(start)
			{
				bls_peer_start_enc_flag = 1;

				if (blc_cyrpt_para.st == SLAVE_LL_ENC_OFF || blc_cyrpt_para.st == SLAVE_LL_ENC_PAUSE_RSP)
				{
					if(blc_cyrpt_para.st == SLAVE_LL_ENC_PAUSE_RSP){
						restart_encryption = 1;
					}
					else{
						restart_encryption = 0;
					}

					//SDKs and IVs random generate every new pair
					generateRandomNum(8, (u8 *)enc_skds);
					generateRandomNum(4, (u8 *)&enc_ivs);

					bls_ll_pushEncPkt (LL_ENC_RSP);

					smemcpy (enc_skdm, pll->dat + 10, 8);
					smemcpy (&enc_ivm, pll->dat + 18, 4);

					//hci event callback  or  direct call sm functions
					u8* randomNumber = pll->dat;
					u16 ediv = pll->dat[8] | (pll->dat[9]<<8);



					if(blt_ltk_request){
						blt_ltk_request (bltc.connHandle, randomNumber, ediv);
					}

					if(hci_le_eventMask & HCI_LE_EVT_MASK_LONG_TERM_KEY_REQUEST)
					{
						u8 result[13];
						hci_le_longTermKeyRequestEvt_t *pEvt = (hci_le_longTermKeyRequestEvt_t *)result;

						pEvt->subEventCode = HCI_SUB_EVT_LE_LONG_TERM_KEY_REQUESTED;
						pEvt->connHandle = bltc.connHandle;
						pEvt->ediv = ediv;
						smemcpy(pEvt->random, randomNumber, 8);

						blc_hci_send_event (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, result, 13);
					}


					blt_smp_empty_pkt_entry = 1;
					blc_ll_setEncryptionBusy (1);
				}
				else		// error
				{

				}
			}
			else if (opcode == LL_ENC_RSP)				//ll_enc_rsp tx
			{
				//pll->dat[0] is feature[0]
			}
			else if (opcode == LL_START_ENC_REQ)				//ll_start_enc_req tx
			{

			}
			else if (opcode == LL_START_ENC_RSP)				//ll_start_enc_rsp rx => tx (rsp)
			{
				if (blc_cyrpt_para.st == SLAVE_LL_ENC_START_REQ_T)
				{
					bls_peer_start_enc_flag = 0;
					bls_ll_pushEncPkt (LL_START_ENC_RSP);
					lmp_tick = 0;
					lmp_timeout = 0;


					blc_cyrpt_para.st = SLAVE_LL_ENC_START_RSP_T;  //should set this in encryption change event by host


					//encBusyFlag clear before eventCallback to prevent one situation: user can not push packet to TX FIFO
					//due to encBusyFlag check error in some user API(pushNotify/push connParam update/...)  (problem raised by zhitao.wu)
					blc_ll_setEncryptionBusy (0);

					if(ll_encryption_done_cb){
						ll_encryption_done_cb(BLS_CONN_HANDLE);
					}



					if(hci_eventMask & HCI_EVT_MASK_ENCRYPTION_CHANGE)
					{
						u8 result[4];
						if(restart_encryption)
						{
							event_enc_refresh_t* pe = (event_enc_refresh_t*) result;
							pe->status = BLE_SUCCESS;										// status: success
							pe->handle = bltc.connHandle;					// handle

							blc_hci_send_event (HCI_FLAG_EVENT_BT_STD | HCI_EVT_ENCRYPTION_KEY_REFRESH, result, 3);

						}
						else
						{
							event_enc_change_t* pe = (event_enc_change_t*) result;

							pe->status = BLE_SUCCESS;
							pe->handle = bltc.connHandle;
							pe->enc_enable = 1;

							blc_hci_send_event (HCI_FLAG_EVENT_BT_STD | HCI_EVT_ENCRYPTION_CHANGE, result, 4);

						}
					}




					#if (LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN)
						if( bltParam.enable_LE_Authenticated_Payload ){
							bltParam.tick_LE_Authenticated_Payload = clock_time() | 1;
						}
					#endif

				}
				else
				{

				}
			}
			else if (opcode == LL_PAUSE_ENC_REQ)				//ll_pause_enc_req rx => tx (rsp)
			{
				if (blc_cyrpt_para.st == SLAVE_LL_ENC_START_RSP_T)
				{
					blc_cyrpt_para.st = SLAVE_LL_ENC_PAUSE_RSP_T;
					//send response encrypted
					bls_ll_pushEncPkt (LL_PAUSE_ENC_RSP);

					/////////////////////////
					blc_cyrpt_para.enable = 0;

					#if (LL_PAUSE_ENC_REQ)
						blc_ll_setEncryptionBusy (0);

						if(ll_encryption_pause_cb){
							ll_encryption_pause_cb(BLS_CONN_HANDLE);
						}
					#endif
				}
				else
				{

				}
			}
			else if (opcode == LL_PAUSE_ENC_RSP)				//ll_pause_enc_rsp rx
			{
				if (blc_cyrpt_para.st == SLAVE_LL_ENC_PAUSE_RSP_T)
				{
					blc_cyrpt_para.st = SLAVE_LL_ENC_PAUSE_RSP;
				}
				else
				{

				}
			}
#endif
			else if(opcode == LL_UNKNOWN_RSP)     //LL Control Opcode: 0x07
			{
				if(pll->dat[0] == LL_SLAVE_FEATURE_REQ){
					if(bltc.remoteFeatureReq){
						bltc.remoteFeatureReq = 0;
						bltc.ll_remoteFeature = 0;
						bls_ll_send_read_remote_feature_event(HCI_ERR_UNSUPPORTED_REMOTE_FEATURE);
					}
				}
				#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
					else if(pll->dat[0] == LL_LENGTH_REQ){
						bltData.connRemoteMaxTxOctets = bltData.connRemoteMaxRxOctets = MAX_OCTETS_DATA_LEN_27;
						bltData.connEffectiveMaxTxOctets = bltData.connEffectiveMaxRxOctets = MAX_OCTETS_DATA_LEN_27;
						bltData.connRxDiff100 = 0;
						bltData.connTxDiff100 = 0;
						blt_p_event_callback (BLT_EV_FLAG_DATA_LENGTH_EXCHANGE, (u8 *)&bltData.connEffectiveMaxRxOctets, 12);
					}
				#endif
			}
			else if (opcode == LL_FEATURE_REQ || opcode == LL_FEATURE_RSP)  //LL Control Opcode: 0x08/0x09/0x0E
			{

				bltc.ll_remoteFeature = pll->dat[0] | (pll->dat[1]<<8) | (pll->dat[2]<<16) | (pll->dat[3]<<24);

				#if (LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN)
					if( LL_FEATURE_ENABLE_LE_PING && (bltc.ll_remoteFeature & LL_FEATURE_MASK_LE_PING) ){
						bltParam.enable_LE_Authenticated_Payload = 1;
					}
				#endif

				if(opcode == LL_FEATURE_RSP){ //feature_rsp
					if(bltc.remoteFeatureReq){
						bltc.remoteFeatureReq = 0;
						bls_ll_send_read_remote_feature_event(BLE_SUCCESS);
					}
				}
				else{  //feature_req
					dat[0] = 0x0903;			//type, len
					dat[1] = LL_FEATURE_RSP 	| LL_FEATURE_BYTE_0 << 8;				//feature
					dat[2] = LL_FEATURE_BYTE_1	| LL_FEATURE_BYTE_2 << 8;
					dat[3] = LL_FEATURE_BYTE_3	| LL_FEATURE_BYTE_4 << 8;
					dat[4] = LL_FEATURE_BYTE_5	| LL_FEATURE_BYTE_6 << 8;
					dat[5] = LL_FEATURE_BYTE_7;
					bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, (u8 *)dat);
				}

			}
			else if(opcode == LL_VERSION_IND )	//LL Control Opcode: 0x0C
			{
				if(bltc.remoteVersionFlag){

					bltc.remoteVersionFlag = 0;
					if (hci_eventMask & HCI_EVT_MASK_READ_REMOTE_VERSION_INFORMATION_COMPLETE) {
						u8 p8[8];
						p8[0] = 0; // status: success
						p8[1] = BLM_CONN_HANDLE; // handle
						p8[2] = BLM_CONN_HANDLE >> 8; // handle
						memcpy(p8 + 3, pll->dat, 5);

						blc_hci_send_event( HCI_FLAG_EVENT_BT_STD | HCI_EVT_READ_REMOTE_VER_INFO_COMPLETE, p8, 8); //read remote version complete
					}
				}else{
					if(!blt_ll_version_ind_rsp_flag)
					{
						dat[0] = 0x0603;			//type, len
						dat[1] = LL_VERSION_IND | (BLUETOOTH_VER << 8);
						dat[2] = VENDOR_ID;
						dat[3] = BLUETOOTH_VER_SUBVER;
						bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, (u8 *)dat);

						blt_ll_version_ind_rsp_flag = 1;
					}
				}
				blt_p_event_callback (BLT_EV_FLAG_VERSION_IND_REV, (u8*)pll->dat, 5); //rf_packet_version_ind_t
			}
			else if(opcode == LL_REJECT_IND )	//LL Control Opcode: 0x0D
			{
				bls_peer_start_enc_flag = 0;
			}
#if (LL_FEATURE_ENABLE_CONNECTION_PARA_REQUEST_PROCEDURE)
			else if (opcode == LL_CONNECTION_PARAM_REQ || opcode == LL_CONNECTION_PARAM_RSP )   //LL Control Opcode: 0x0F/0x10
			{

			}
#endif
#if (LL_FEATURE_ENABLE_EXTENDED_REJECT_INDICATION)
			else if (opcode == LL_REJECT_IND_EXT)   //LL Control Opcode: 0x11
			{
				bls_peer_start_enc_flag = 0;
			}
#endif
#if (LL_FEATURE_ENABLE_LE_PING)
			else if (opcode == LL_PING_REQ )	//LL Control Opcode: 0x12
			{
				dat[0] = 0x0103;		//type, len
				dat[1] = LL_PING_RSP;
				bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, (u8 *)dat);
			}
			else if (opcode == LL_PING_RSP )	//LL Control Opcode: 0x13
			{

			}
#endif
#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
			else if (opcode == LL_LENGTH_REQ ||  opcode == LL_LENGTH_RSP){ //LL Control Opcode: 0x14/0x15

					u16 connRemoteMaxRxOctets = pll->dat[0] | (pll->dat[1] << 8);
					u16 connRemoteMaxTxOctets = pll->dat[4] | (pll->dat[5] << 8);

					if(connRemoteMaxRxOctets < 27 || connRemoteMaxRxOctets > 251 || connRemoteMaxTxOctets < 27 || connRemoteMaxTxOctets > 251){
						// unknown rsp
						blt_ll_unknown_rsp(BLS_CONN_HANDLE | HANDLE_STK_FLAG, opcode);
						return 0;
					}
					bltData.connRemoteMaxRxOctets = connRemoteMaxRxOctets;
					bltData.connRemoteMaxTxOctets = connRemoteMaxTxOctets;

					u16 new_effectiveMaxRx = min(bltData.supportedMaxRxOctets, bltData.connRemoteMaxTxOctets);
					u16 new_effectiveMaxTx = min(bltData.supportedMaxTxOctets, bltData.connRemoteMaxRxOctets);

					//GaoQiu fixed bug
					//when slave send LL_LENGTH_REQ and(connMaxTxOctets < supportedMaxTxOctets) or (connMaxRxOctets < supportedMaxRxOctets)
					//master will response with LL__LENGTH_RSP,
					//if we get  effect Tx/Rx using min(bltData.supportedMaxRxOctets, bltData.connRemoteMaxTxOctets),
					//then we will make a mistake that remote master and slave may have an mismatch effect Tx/Rx value.
					if(opcode == LL_LENGTH_RSP){
						 new_effectiveMaxRx = min(bltData.connMaxRxOctets, bltData.connRemoteMaxTxOctets);
						 new_effectiveMaxTx = min(bltData.connMaxTxOctets, bltData.connRemoteMaxRxOctets);
					}

					int change = (bltData.connEffectiveMaxRxOctets != new_effectiveMaxRx) || \
							     (bltData.connEffectiveMaxTxOctets != new_effectiveMaxTx);

					bltData.connEffectiveMaxRxOctets = new_effectiveMaxRx;
					if(new_effectiveMaxRx > 100){
						bltData.connRxDiff100 = new_effectiveMaxRx - 100;
					}
					bltData.connEffectiveMaxTxOctets = new_effectiveMaxTx;
					if(new_effectiveMaxTx > 100){
						bltData.connTxDiff100 = new_effectiveMaxTx - 100;
					}

					if(opcode == LL_LENGTH_REQ){
						blt_ll_exchangeDataLength(LL_LENGTH_RSP, bltData.supportedMaxTxOctets);
					}
					bltData.connMaxTxRxOctets_req = DATA_LENGTH_REQ_DONE;

					if( change && (hci_le_eventMask & HCI_LE_EVT_MASK_DATA_LENGTH_CHANGE) ){
						u8 result[11];
						hci_le_dataLengthChangeEvt_t *pEvt = (hci_le_dataLengthChangeEvt_t *)result;

						pEvt->subEventCode = HCI_SUB_EVT_LE_DATA_LENGTH_CHANGE;
						pEvt->connHandle = bltc.connHandle;
						pEvt->maxTxOct = bltData.connEffectiveMaxTxOctets;
						pEvt->maxTxtime = LL_PACKET_OCTET_TIME(bltData.connEffectiveMaxTxOctets);
						pEvt->maxRxOct = bltData.connEffectiveMaxRxOctets;
						pEvt->maxRxtime = LL_PACKET_OCTET_TIME(bltData.connEffectiveMaxRxOctets);

						blc_hci_send_event (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, result, 11);
					}
				#if (MCU_CORE_TYPE == MCU_CORE_9518)
					reg_rf_rxtmaxlen = bltData.connEffectiveMaxRxOctets + 4; // Contain MIC(4) length
				#endif
					blt_p_event_callback (BLT_EV_FLAG_DATA_LENGTH_EXCHANGE, (u8 *)&bltData.connEffectiveMaxRxOctets, 12);
			}
#endif
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
			else if (opcode == LL_PHY_REQ || opcode == LL_PHY_UPDATE_IND)
			{

				if( !(LL_FEATURE_MASK_0 & (LL_FEATURE_ENABLE_LE_2M_PHY<<8 | LL_FEATURE_ENABLE_LE_CODED_PHY<<11) ) ){
					blt_ll_unknown_rsp(BLS_CONN_HANDLE | HANDLE_STK_FLAG, opcode);
				}
				else{

					if (opcode == LL_PHY_REQ )	//LL Control Opcode: 0x16
					{
						rf_pkt_ll_phy_req_rsp_t* pReq = (rf_pkt_ll_phy_req_rsp_t *)(p+DMA_RFRX_OFFSET_HEADER);

						u8 comm_phy = comm_phy = pReq->tx_phys & pReq->rx_phys;  //support symmetric PHYs only; support 1M/2M/Coded PHY all

						if(comm_phy==0)// if rx and tx have none common PHYs, select 1M PHY
						{
							comm_phy = BLE_PHY_1M;
						}

						dat[0] = 0x0303;
						dat[1] = LL_PHY_RSP | (comm_phy<<8);
						dat[2] = comm_phy;
						bls_ll_pushTxFifo (BLS_CONN_HANDLE|HANDLE_STK_FLAG, (u8*)dat);

					}
					else if (opcode == LL_PHY_RSP )	//LL Control Opcode: 0x17
					{
						// master never send LL_PHY_RSP, so slave will not receive it
					}
					else if (opcode == LL_PHY_UPDATE_IND )	//LL Control Opcode: 0x18
					{
#if (!BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE)
						rf_pkt_ll_phy_update_ind_t* pUpdt = (rf_pkt_ll_phy_update_ind_t *)(p+DMA_RFRX_OFFSET_HEADER);


						u8 comm_phy = pUpdt->m_to_s_phy & pUpdt->s_to_m_phy;

						u16 conn_inst_next = pUpdt->instant0 | (pUpdt->instant1<<8);
						s16 diff_inst = conn_inst_next - blttcon.conn_inst;
						if(diff_inst > 0){
							bltc.master_not_ack_slaveAckUpReq = 1;
							blttcon.conn_update = 3;
							blttcon.conn_inst_next = conn_inst_next;
							if(comm_phy & PHY_PREFER_1M){
								blt_conn_phy.conn_next_phy = BLE_PHY_1M;
							}
							else if(comm_phy & PHY_PREFER_2M){
								blt_conn_phy.conn_next_phy = BLE_PHY_2M;
							}
							else if(comm_phy & PHY_PREFER_CODED){
								blt_conn_phy.conn_next_phy = BLE_PHY_CODED;
							}
							else{  //no PHY Update
								blt_conn_phy.conn_next_phy = blt_conn_phy.conn_cur_phy;
							}
						}
						else //todo wrap around
						{
							//bls_ll_terminateConnection(HCI_ERR_INSTANT_PASSED);
							//hci_disconnectionComplete_evt(0,bltc.connHandle,HCI_ERR_INSTANT_PASSED);
						}
#endif
					}


				}

			}
#endif
#if (LL_FEATURE_ENABLE_MIN_USED_OF_USED_CHANNELS)
			else if (opcode == LL_MIN_USED_CHN_IND )	//LL Control Opcode: 0x19
			{

			}
#endif
			else {   //Control PDUs not supported, send "LL_UNKNOWN_RSP"
				blt_ll_unknown_rsp(BLS_CONN_HANDLE | HANDLE_STK_FLAG, opcode);
			}
		}
		//////////// L2CAP pay load ///////////////////////////////
		else if (type != 0)
		{
			if (blc_l2cap_handler)
			{
				blc_l2cap_handler (BLS_CONN_HANDLE, (p+DMA_RFRX_OFFSET_HEADER));
			}
		}
		// error: invliad LL type
		else
		{

		}

	}

	///////////////////////////////////////////////////////////////
	if(blt_smp_empty_pkt_entry){
		bls_ll_security_proc ();
	}


	return 1;
}

//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
#define 		CONN_INTERVAL_SML		1
#define			CONN_INTERVAL_MID		2
#define			CONN_INTERVAL_BIG		3

/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void blt_set_fsmTimeout_intLevel(u16 interval)
{

	if(interval < 8){  // < 10ms
		bltc.interval_level = CONN_INTERVAL_SML;
	}
	else if(interval < 80){ // < 100ms
		bltc.interval_level = CONN_INTERVAL_MID;
	}
	else{   //duration max 11000 us
		bltc.interval_level = CONN_INTERVAL_BIG;
	}


	//core_f2c will reset to 9999uS after every suspend/deep retention
	if(interval < 17){  // <= 16*1.25ms = 20mS
		bltc.conn_fsm_timeout = (interval-1)*1250;   // conn_interval - 1.25mS
	}
	else if(interval < 33){  // <= 32*1.25 = 40mS
		bltc.conn_fsm_timeout = (interval-4)*1250;   // conn_interval - 5 mS
	}
	else{  // > 40mS
		bltc.conn_fsm_timeout = interval*625;   	 // conn_interval/2
	}
}


/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void blt_brx_timing_update_init(void)
{

	bls_timing.brx_pkt_miss = 0;
	bls_timing.blt_brx_synced = 0;
	bls_timing.brx_pkt_rcvd = 0;
	bls_timing.connection_offset = 0;
	bltc.connExpectTime += bltc.conn_interval;
	bls_timing.brx_ref = CONN_LONG_TOLERANCE_TIME;
	bltc.conn_interval_adjust = 0;


	FSM_TIMEOUT_DISABLE;

}



_attribute_ram_code_
void blt_brx_timing_update (u32 t, int st)
{
	///////////////////////////////////////////////
	//	dc offset estimation & compensation
	//	timing tracking
	//
	///////////////////////////////////////////////


	if (BLT_BRX_TIMING_STATUS_OK == st)
	{
		bls_timing.brx_pkt_miss = 0;


		if (bls_timing.brx_pkt_rcvd < 6) {
			bls_timing.brx_pkt_rcvd ++;
		}
		else {	//connection stable
			bls_timing.brx_ref = CONN_SHORT_TOLERANCE_TIME;
		}


		if (!(bls_timing.blt_brx_synced & BIT(7))) {
			bls_timing.blt_brx_synced++;
		}
	#if (MCU_CORE_TYPE == MCU_CORE_9518)
		if(blt_miscParam.pad32k_en)
	#endif
		{
			if (bls_timing.blt_brx_synced > 1 && !bltPm.timer_wakeup)	//calculate DC offset
			{
				int td = bltc.conn_brx_tick - bls_timing.last_brx;
				if (td > (100000 * SYSTEM_TIMER_TICK_1US))
				{
					int offset = bls_timing.connection_offset * td / (int)(1024 * SYSTEM_TIMER_TICK_1US);
					int diff = (int)(t - bltc.connExpectTime + offset);

					//blt_push_notify (0x0a, t - bltc.conn_brx_tick, 4);
					diff = diff * (int)(1024 * SYSTEM_TIMER_TICK_1US) / td;

					bls_timing.connection_offset += (diff - bls_timing.connection_offset)>>1;
					bltc.conn_interval_adjust = bls_timing.connection_offset * (int)bltc.conn_interval / (int)(1024 * SYSTEM_TIMER_TICK_1US);
				}
			}
		}
		bltc.conn_tolerance_time = bls_timing.brx_ref;

	}
	else
	{
		bls_timing.blt_brx_synced = 0;
		if (bls_timing.brx_pkt_miss < bltc.interval_level)
		{
			bls_timing.brx_pkt_miss++;
		}

		if (bls_timing.brx_pkt_rcvd > 1) {
			bls_timing.brx_pkt_rcvd -= 2;
		}
		else {	//connection unstable
			bls_timing.brx_ref = CONN_LONG_TOLERANCE_TIME;
		}

		t = bltc.connExpectTime;

		bltc.conn_tolerance_time = bls_timing.brx_pkt_miss * bls_timing.brx_ref + CONN_SHORT_TOLERANCE_TIME;
	}

	bltc.connExpectTime = t + bltc.conn_interval;
	bltc.conn_duration = CONN_BRX_WINDOW + bltc.conn_tolerance_time * 2;

	bls_timing.last_brx = bltc.conn_brx_tick;

	bltPm.timer_wakeup = 0;
}


/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void bls_ll_setNewConnection_timing(u16 interval, u8 winsize, u16 winOffset)
{
	bltc.conn_new_param = 1;

	u16 diff = interval - winsize;

	if(diff < 4){
		//1 : 500  1000
		//2:  750  1500
		//3:  1000 2000    if(winoffset == 0)  750  1500
		bltc.conn_tolerance_time = (diff+1)*250 * SYSTEM_TIMER_TICK_1US;
		bltc.conn_duration =  (diff+1)*500 * SYSTEM_TIMER_TICK_1US + winsize * SYSTEM_TIMER_TICK_1250US;

		bltPm.conn_no_suspend = 1;
	}
	else{
		bltc.conn_tolerance_time = 2000 * SYSTEM_TIMER_TICK_1US;
		bltc.conn_duration =  4000 * SYSTEM_TIMER_TICK_1US + winsize * SYSTEM_TIMER_TICK_1250US;
	}


	if(winOffset == 0 && diff > 2){
		bltc.conn_tolerance_time = 750 * SYSTEM_TIMER_TICK_1US;
		bltc.conn_duration =  1500 * SYSTEM_TIMER_TICK_1US + winsize * SYSTEM_TIMER_TICK_1250US;
		bltPm.conn_no_suspend = 1;
	}

}

int blt_connect (u8 * raw_pkt, bool aux_conn)
{
	rf_packet_connect_t *pInit = (rf_packet_connect_t *)(raw_pkt + 0);

	bltParam.blt_state = BLS_LINK_STATE_CONN;

	bltc.conn_sn_master = 0x10;

	bltc.tick_1st_rx = 0; //kite/vulture no exist before. but it is not problem.

	reg_dma_tx_rptr = FLD_DMA_RPTR_CLR;			//reset rptr = wptr
	blt_txfifo.wptr = blt_txfifo.rptr = 0;	//reg_dma_tx_rptr;
	blt_bb.blt_dma_tx_rptr = reg_dma_tx_rptr & FLD_DMA_RPTR_MASK;
	
	bltc.conn_interval_n_1m25 = pInit->interval;
	bltc.conn_interval = pInit->interval * SYSTEM_TIMER_TICK_1250US;
	blt_set_fsmTimeout_intLevel(pInit->interval);

	/**
	 * The value of transmitWindowDelay shall be 1.25 ms when a a CONNECT_IND PDU is used,2.5ms
	 * when an AUX_CONNECT_REQ PDU is used on an LE Uncoded PHY,and 3.75ms when an AUX_CONNECT_REQ PDU
	 * is used on the LE Coded PHY.
	 */
#if(LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	bltc.connExpectTime = (pInit->winOffset + 1 + (aux_conn == TRUE) + (bltPHYs.cur_llPhy == BLE_PHY_CODED)) * SYSTEM_TIMER_TICK_1250US + blc_rcvd_connReq_tick - bltc.conn_interval;
#else
	bltc.connExpectTime = (pInit->winOffset + 1 + (aux_conn == TRUE)) * SYSTEM_TIMER_TICK_1250US + blc_rcvd_connReq_tick - bltc.conn_interval;
#endif
	bls_ll_setNewConnection_timing(pInit->interval, pInit->winSize, pInit->winOffset);
	bltc.conn_timeout = pInit->timeout * 10000 * SYSTEM_TIMER_TICK_1US;
	bltc.conn_latency = pInit->latency;
	blttcon.conn_tick = blc_rcvd_connReq_tick;
	bltc.conn_start_tick = blc_rcvd_connReq_tick;

#if	(BQB_5P0_TEST_ENABLE)
	bltc.conn_establish_pending_flag =1;
	bltc.conn_establish_pending_timeout = (6 - 1) * pInit->interval * 1250 - 5000;// TP/ENC/ADV/BI-02-C why master timeout is 5 intervel
#endif
	//bltc.role = LL_ROLE_SLAVE;
	bltc.connHandle = BLS_CONN_HANDLE;


	blc_cyrpt_para.mic_fail = 0;  //clear mic fail flag
	bltc.conn_stop_brx = 0;
	bltc.conn_pkt_dec_pending = 0;
	bltc.conn_enc_dec_busy  = 0;

	blt_brx_timing_update_init ();  //BLT_BRX_TIMING_STATUS_INIT


	blttcon.connHandle = BLS_CONN_HANDLE;


	if(aux_conn == TRUE)
	{
		bltParam.peer_chSel = 1;
	}
	else
	{
		bltParam.peer_chSel = pInit->chan_sel;
	}


	blt_ll_connect_common(blt_pconn, pInit);


	//rf baseband set
	reset_sn_nesn();

	blt_bb.sn_nesn = 0x10;

#if (FIX_HW_CRC24_EN)
	extern u32 revert_conn_crc;
	extern u32 reverse_32bit(volatile u32 x);
	revert_conn_crc = (reverse_32bit(blttcon.conn_crc) >> 8) & 0xffffff;
#endif

	blc_ll_resetInfoRSSI();


	bltc.ll_remoteFeature = 0;
	bltc.remoteVersionFlag = 0;

	blc_cyrpt_para.enable = 0;
	blc_cyrpt_para.st = SLAVE_LL_ENC_OFF;  //clear status

	bls_peer_start_enc_flag = 0;

	if(ll_connComplete_handler){
		ll_connComplete_handler(BLS_CONN_HANDLE, raw_pkt);
	}



#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
	bltData.connEffectiveMaxTxOctets = MAX_OCTETS_DATA_LEN_27;
	bltData.connEffectiveMaxRxOctets = MAX_OCTETS_DATA_LEN_27;
	bltData.connRxDiff100 = 0;
	bltData.connTxDiff100 = 0;
	#if (MCU_CORE_TYPE == MCU_CORE_9518)
		if(bltParam.maxRxOct != MAX_OCTETS_DATA_LEN_27 || bltParam.maxTxOct != MAX_OCTETS_DATA_LEN_27){
			bltData.connMaxTxRxOctets_req = DATA_LENGTH_REQ_PENDING;
		}
		else{
			bltData.connMaxTxRxOctets_req = 0;
		}
	#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
		if(bltData.connInitialMaxTxOctets > MAX_OCTETS_DATA_LEN_27){
			bltData.connMaxTxRxOctets_req = DATA_LENGTH_REQ_PENDING;
			bltData.connMaxTxOctets = bltData.connInitialMaxTxOctets;
		}
		else{
			bltData.connMaxTxRxOctets_req = 0;
			bltData.connMaxTxOctets = MAX_OCTETS_DATA_LEN_27;
		}
	#endif
#endif


#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	//If the conn is established by AUX_CONN_REQ pkt, you need to keep the connPHY and AUX_CONN_REQ pkt using the same PHY.
	if(aux_conn == TRUE){
		blt_conn_phy.conn_cur_phy = bltPHYs.cur_llPhy;
		blt_conn_phy.conn_cur_CI  = bltPHYs.cur_CI;
	}
#endif
#if (MCU_CORE_TYPE == MCU_CORE_9518)
	#if PM_32k_RC_CALIBRATION_ALGORITHM_EN
		ble_first_rx_tick_last = 0;
		ble_first_rx_tick_pre = 0;
		ble_actual_conn_interval_tick = 0;
	#endif
#endif
	bltParam.ble_state = BLE_STATE_BRX_E;

#if (0)
	//if callback take too long time, it will disconnect. so move it to after systimer_set_irq_capture in blt_send_adv in file ll_adv.c
	// 6	 6		4		  3       1			2		   2		2		2		 5	  1
	//scanA advA accesscode crcint winsize  winOffset interval latency  timeout  chn[5]  hop
	blt_p_event_callback (BLT_EV_FLAG_CONNECT, raw_pkt+DMA_RFRX_OFFSET_DATA, 34);
#endif
	/*
	 * If HCI_LE_Enhanced_Connection_Complete event is unmasked and the HCI_LE_Connection_Complete event is unmasked,
	 * only the HCI_LE_Enhanced_Connection_Complete event is sent when a new connection has been created.
	 */
#if (LL_FEATURE_ENABLE_LL_PRIVACY)
	if(hci_le_eventMask & HCI_LE_EVT_MASK_ENHANCED_CONNECTION_COMPLETE){
		u8 peerRpa[BLE_ADDR_LEN];
		u8 localRpa[BLE_ADDR_LEN];
		ll_resolvingList_getLocalResolvableAddr(blta.advPeerAddrType, blta.advPeerAddr, localRpa);
		ll_resolvingList_getPeerResolvableAddr(blta.advPeerAddrType, blta.advPeerAddr, peerRpa);
		hci_le_enhancedConnectionComplete_evt(BLE_SUCCESS, bltc.connHandle, LL_ROLE_SLAVE, bltc.conn_peer_addr_type, bltc.conn_peer_addr, localRpa, peerRpa, pInit->interval, pInit->latency, pInit->timeout, (pInit->hop & LL_CONNECT_REQ_SCA_MASK)>>5);
	}
	else
#endif
	if(hci_le_eventMask & HCI_LE_EVT_MASK_CONNECTION_COMPLETE){
		u8 addrTypep = pInit->txAddr? 1 : 0;

		u8 result[20];
		hci_le_connectionCompleteEvt_t *pEvt = (hci_le_connectionCompleteEvt_t *)result;

		pEvt->subEventCode = HCI_SUB_EVT_LE_CONNECTION_COMPLETE;
		pEvt->status = BLE_SUCCESS;
		pEvt->connHandle = bltc.connHandle;
		pEvt->role = LL_ROLE_SLAVE;
		pEvt->peerAddrType = addrTypep;
		smemcpy(pEvt->peerAddr, pInit->initA, BLE_ADDR_LEN);
		pEvt->connInterval = pInit->interval;
		pEvt->slaveLatency = pInit->latency;
		pEvt->supervisionTimeout = pInit->timeout;
		pEvt->masterClkAccuracy = (pInit->hop & LL_CONNECT_REQ_SCA_MASK)>>5;

		blc_hci_send_event ( HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, result, 19);
	}




	return 1;
}





/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void connUpdate_mapUpdate_proc(void)
{
	if (blttcon.conn_update == 1 && blttcon.conn_inst == blttcon.conn_inst_next)
	{
		//dgb_chn_up++;
		blttcon.conn_update = 0;
		smemcpy (blttcon.conn_chn_map, blttcon.conn_chn_map_next, 5);

		#if (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)
			if(blttcon.conn_chnsel)
				blc_calc_remapping_table(blttcon.conn_chn_map);
			else
				blt_ll_channelTable_calc (blttcon.conn_chn_map, blttcon.conn_chn_hop, blttcon.chn_tbl);
		#else
			blt_ll_channelTable_calc (blttcon.conn_chn_map, blttcon.conn_chn_hop, blttcon.chn_tbl);
		#endif
		bltc.tick_1st_rx = 0;
		blc_tlkEvent_pending |= EVENT_MASK_CHN_MAP_UPDATE;
	}
	else if (blttcon.conn_update == 2 && blttcon.conn_inst == blttcon.conn_inst_next)
	{	//new interval: only timer wakeup
		blttcon.conn_update = 0;
		bltc.conn_timeout = bltc.conn_timeout_next * 10000 * SYSTEM_TIMER_TICK_1US;
		bltc.conn_latency = bltc.conn_latency_next;
		blt_set_fsmTimeout_intLevel(bltc.conn_interval_next);
		u32 new_interval = bltc.conn_interval_next * SYSTEM_TIMER_TICK_1250US;
		
	#if (MCU_CORE_TYPE == MCU_CORE_9518)
		if(!blt_miscParam.pad32k_en)
		{
			conn_new_interval_flag = 1;
		}
	#endif

		bltc.connExpectTime += bltc.conn_offset_next * SYSTEM_TIMER_TICK_1250US + bltc.conn_interval - new_interval;

		bltc.conn_interval_n_1m25 = bltc.conn_interval_next;
		bltc.conn_interval = new_interval;
		bls_ll_setNewConnection_timing(bltc.conn_interval_next, bltc.conn_winsize_next, bltc.conn_offset_next);

		bltc.tick_1st_rx = 0;				//treat as no packet received

		blc_tlkEvent_pending |= EVENT_MASK_CONN_PARA_UPDATE;
	}
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	else if (blttcon.conn_update == 3 && blttcon.conn_inst == blttcon.conn_inst_next)
	{
		blttcon.conn_update = 0;

		if(ll_conn_phy_update_cb){
			ll_conn_phy_update_cb();
		}
	}
#endif
}

/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void blt_terminate_pending_proc(void)  //process rcvd master's terminate or slave send terminate
{
	//1. master terminate acked
	//when rcvd master's terminate, should delay several interval to make sure master rcvd ack
	if(bltc.conn_master_terminate){
		bltc.conn_master_terminate++;
		if (bltc.conn_master_terminate>=3 ){
			bltc.conn_terminate_pending = 2;  //slave can disconnect now
			return;
		}
	}

#if(BQB_5P0_TEST_ENABLE)
	//2. slave terminate acked/timeout
	if(bltc.conn_slave_terminate == TERMINTE_CMD_IN_FIFO){
		if( (((reg_dma_tx_wptr - reg_dma_tx_rptr) & 15)==0  &&  blt_txfifo.rptr == blt_txfifo.wptr)){

			bltc.conn_terminate_pending = 2;  //slave can disconnect now
			bltc.conn_terminate_reason = pkt_slave_terminate.reason;
			bltc.conn_slave_terminate = TERMINTE_CMD_IDLE;
		}

		else if( clock_time_exceed(bltc.conn_slaveTerminate_time, 500000) )//500 ms
		{
			bltc.conn_terminate_pending = 2;  //slave can disconnect now
			bltc.conn_terminate_reason = HCI_ERR_CONN_TERM_BY_LOCAL_HOST;
			bltc.conn_slave_terminate = TERMINTE_CMD_IDLE;
		}
#else
		//2. slave terminate acked/timeout
		if(bltc.conn_slave_terminate == TERMINTE_CMD_IN_FIFO){
			if( (((reg_dma_tx_wptr - reg_dma_tx_rptr) & FLD_DMA_RPTR_MASK)==0  &&  blt_txfifo.rptr == blt_txfifo.wptr) ||  \
				 clock_time_exceed(bltc.conn_slaveTerminate_time, 500000) ){ //500 ms
				bltc.conn_terminate_pending = 2;  //slave can disconnect now
				bltc.conn_terminate_reason = pkt_slave_terminate.reason;
				bltc.conn_slave_terminate = TERMINTE_CMD_IDLE;
			}
#endif
	}
}

/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void blt_terminate_proc(void)
{
	//blt conn terminate pending == 2 : master/slave send terminate OK
	//blt conn terminate pending != 2 : conn supervision timeout
#if	(BQB_5P0_TEST_ENABLE)
	if(bltc.conn_terminate_pending == 3)
	{
		bltc.conn_terminate_reason = HCI_ERR_CONN_FAILED_TO_ESTABLISH;
	}
	else
#endif
	if(bltc.conn_terminate_pending == 4)
	{
		bltc.conn_terminate_reason = HCI_ERR_LMP_LL_RESP_TIMEOUT;
	}
	else if(bltc.conn_terminate_pending != 2){
		bltc.conn_terminate_reason = HCI_ERR_CONN_TIMEOUT;
	}


	blc_tlkEvent_pending |= EVENT_MASK_TERMINATE;

	//Re-init DLE effective max TX/RX octets
	bltData.connEffectiveMaxTxOctets = MAX_OCTETS_DATA_LEN_27;
	bltData.connEffectiveMaxRxOctets = MAX_OCTETS_DATA_LEN_27;
	bltData.connRxDiff100 = 0;
	bltData.connTxDiff100 = 0;
	bltData.connMaxTxRxOctets_req = 0;

	bltc.conn_slave_terminate = TERMINTE_CMD_IDLE;
	bltc.conn_master_terminate = 0;
	bltc.conn_terminate_pending = 0;
	// make sure this flag is 0 when ADV, to save "bltParam.blt_state == BLS_LINK_STATE_CONN " judge condition for bltc.conn_tolerance_time window widen in ll_pm.c
	blttcon.conn_chnsel = 0;


	//clear
	bltc.conn_tolerance_time = 0;
	bltc.connHandle = BLE_INVALID_CONNECTION_HANDLE;

	// conn_update flag clear to 0, in case of last connect effect next connect
	blttcon.conn_update = 0;

	bltc.tick_1st_rx = 0;

#if (LL_FEATURE_ENABLE_LE_EXTENDED_ADVERTISING)
	if(pFunc_ll_SetAdv_Enable)
	{
		//todo
		pFunc_ll_SetAdv_Enable(bltParam.adv_en | BLC_FLAG_STK_ADV);
	}
#else
	{
		bls_ll_setAdvEnable(bltParam.adv_en | BLC_FLAG_STK_ADV);
	}
#endif

	bltc.long_suspend = 0;
	bltPm.conn_no_suspend = 0;
#if (LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN)
	bltParam.enable_LE_Authenticated_Payload = 0;
	bltParam.tick_LE_Authenticated_Payload = 0;
#endif
}



_attribute_ram_code_
u8 blt_brx_start ()
{
#if (MCU_CORE_TYPE == MCU_CORE_9518)
	ble_rf_set_rx_dma((u8*)bltParam.acl_rx_dma_buff, bltParam.acl_rx_dma_size);
#endif
	rf_set_tx_rx_off(); //must add
	STOP_RF_STATE_MACHINE;	// stop SM

	reset_sn_nesn();

	bltc.conn_brx_tick = clock_time ();
	bltc.tick_1st_rx = 0;
	bltc.conn_rcvd_ack_pkt = 0;

	blt_ll_start_common(blt_pconn);

	blt_push_fifo_hw ();


#if(LL_FEATURE_ENABLE_LE_2M_PHY || LL_FEATURE_ENABLE_LE_CODED_PHY)
	rf_tx_settle_adjust( tx_settle_slave[blt_conn_phy.conn_cur_phy] );
	reg_rf_rx_timeout = blt_conn_phy.conn_cur_phy == BLE_PHY_CODED ? 600 : 250;
#else
	rf_tx_settle_adjust(LL_SLAVE_TX_SETTLE);

#endif


	// core_fxx will lose in suspend & deepRetention, must re_set here
	if(bltc.conn_new_param){
		FSM_TIMEOUT_DISABLE;
	}
	else{
		FSM_TIMEOUT_ENABLE;

		#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
			if(bltParam.wirte_sonos_flash_req){

				/*  leave at least 3750uS for flash_write
				 *  RX window extend takes high priority, not change timeout.
				 */
				u32 duration_us = bltc.conn_duration/SYSTEM_TIMER_TICK_1US;
				u32 timeout_us = max2(duration_us, bltc.conn_fsm_timeout - 2500);
				reg_rf_fsm_timeout = timeout_us;
			}
			else
		#endif
			{
				reg_rf_fsm_timeout = bltc.conn_fsm_timeout;  //unit: uS
			}

	}

#if (MCU_CORE_TYPE == MCU_CORE_9518)
	ble_rf_set_tx_dma(blt_txfifo.depth , blt_txfifo.size>>4);

	rf_start_brx ((void *)blt_txfifo.p_default, clock_time () + 10);
#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
	rf_start_brx ((void *)blt_tx_empty_packet, clock_time () + 10);
#endif

	if(blc_rf_pa_cb){	blc_rf_pa_cb(PA_TYPE_RX_ON);  }

	bltPm.padWakeupCnt = 0;

    return 0;
}

_attribute_ram_code_
void blt_brx_post ()
{

	if(blc_rf_pa_cb){	blc_rf_pa_cb(PA_TYPE_OFF);  }

	STOP_RF_STATE_MACHINE;  //stop state machine
	//If there is a boundary RX packet received after brx_post, this RX packet can be dropped manually(clear FLD_RF_IRQ_RX)
	// but boundary RX packet after btx_post can not dropped manually!  So master must make sure boundary RX packet is correctly
	//processed by software. This difference is due to BTX/BRX  TX & RX sequence different
	CLEAR_ALL_RFIRQ_STATUS;

	bltParam.ble_state = BLE_STATE_BRX_E;
#if (MCU_CORE_TYPE == MCU_CORE_9518)
	bltParam.blc_state = BLE_STATE_BRX_E;
#endif

	blttcon.conn_inst++;
	blttcon.chn_idx++;
	if (blttcon.chn_idx >= 37)
	{
		blttcon.chn_idx = 0;
	}
	if(blttcon.conn_update){
		connUpdate_mapUpdate_proc();
	}

	if(bltc.conn_new_param){
		blt_brx_timing_update_init ();  //BLT_BRX_TIMING_STATUS_INIT
	}
	else{
		blt_brx_timing_update (bltc.tick_1st_rx,  bltc.tick_1st_rx ? BLT_BRX_TIMING_STATUS_OK : BLT_BRX_TIMING_STATUS_LOST);
	}

#if	(BQB_5P0_TEST_ENABLE)
	if(bltc.conn_establish_pending_flag && clock_time_exceed(bltc.conn_tick, bltc.conn_establish_pending_timeout))
	{
		bltc.conn_establish_pending_flag =0;
		bltc.conn_terminate_pending = 3;
		blt_terminate_proc();
	}
	else if(lmp_tick && (clock_time_exceed(lmp_tick, lmp_timeout)))
	{
		lmp_tick = 0;
		bltc.conn_terminate_pending = 4;
		blt_terminate_proc();
	}
#else
	if(lmp_tick && (clock_time_exceed(lmp_tick, lmp_timeout)))
	{
		lmp_tick = 0;
		lmp_timeout = 0;
		bls_peer_start_enc_flag = 0;
		bltc.conn_terminate_pending = 4;
		blt_terminate_proc();
	}
#endif

	if(bltc.conn_terminate_pending == 1){  //when master send terminate or slave send terminate
		blt_terminate_pending_proc();
	}

	if(bltc.conn_terminate_pending == 2 || (u32)(clock_time() - blttcon.conn_tick) > bltc.conn_timeout){
		blt_terminate_proc();
	}

#if (LL_SN_NESN_MANAGE_BY_SOFTWARE)
	if(!blt_bb.save_flg){   //for more insurance, if rx irq lost(empty packet), operation here would get correct sn/nesn/tx_rptr
		blt_save_snnesn ();
		#if (MCU_CORE_TYPE == MCU_CORE_9518)
			blt_save_dma_tx_ptr();
		#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
			blt_save_dma_tx_rptr();
		#endif
	}
#else
	#if (MCU_CORE_TYPE == MCU_CORE_9518)
		REG_ADDR8(0x0140a01) =  0x00;   // TODO: check with JunWen
	#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
		REG_ADDR8(0xf01) =  0x00;   //must
	#endif
#endif

#if (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
	#if (DEEP_RET_ENTRY_CONDITIONS_OPTIMIZATION_EN)
		blt_bb.dma_tx_rcvry_dat_depth = (reg_dma_tx_wptr - blt_bb.blt_dma_tx_rptr) & 15;


		// deep ret flg clear in brx_post, and set when deep ret back.
		if(pmParam.is_deepretn_back) {
			pmParam.is_deepretn_back = 0;
		}
	#endif
#endif

}









ble_sts_t 	bls_hci_le_getRemoteSupportedFeatures(u16 connHandle)
{
	if( IS_LL_CONNECTION_VALID(connHandle) ){
		if(!bltc.remoteFeatureReq){
			bltc.remoteFeatureReq = 1;
			u16 dat[6];
			dat[0] = 0x0903;			//type, len

			//TODO: only slave can send LL_SLAVE_FEATURE_REQ
			#if(LL_FEATURE_ENABLE_SLAVE_INITIATED_FEATURES_EXCHANGE)
				dat[1] = LL_SLAVE_FEATURE_REQ | LL_FEATURE_BYTE_0 << 8;				//feature
			#else
				dat[1] = LL_FEATURE_REQ | LL_FEATURE_BYTE_0 << 8;				//feature
			#endif

			dat[2] = LL_FEATURE_BYTE_1	| LL_FEATURE_BYTE_2 << 8;
			dat[3] = LL_FEATURE_BYTE_3	| LL_FEATURE_BYTE_4 << 8;
			dat[4] = LL_FEATURE_BYTE_5	| LL_FEATURE_BYTE_6 << 8;
			dat[5] = LL_FEATURE_BYTE_7;
			bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, (u8 *)dat);
		}
	}
	else{
		return HCI_ERR_UNKNOWN_CONN_ID;
	}


	return BLE_SUCCESS;
}


ble_sts_t 	bls_hci_le_readChannelMap(u16 connHandle, u8 *returnChannelMap)
{
	if( IS_LL_CONNECTION_VALID(connHandle) ){
		smemcpy (returnChannelMap, blttcon.conn_chn_map, 5);
		return BLE_SUCCESS;
	}
	else {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}
}

/*************************************************
 * 	used for set ltk base on hci ltk_req_reply_cmd when receive LL_ENC_REQ.
 */
ble_sts_t  blc_hci_ltkRequestReply (u16 connHandle,  u8*ltk)
{

	if( IS_LL_CONNECTION_VALID(connHandle) ){
		smemcpy((u8*)enc_ltk, ltk, 16);
		blc_cyrpt_para.st = SLAVE_LL_ENC_RSP_T;
		return BLE_SUCCESS;
	}
	else {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}

}

/*
 * @brief  API to notify none LTK associated with the connHandle
 *
 * */
ble_sts_t blc_hci_ltkRequestNegativeReply (u16 connHandle)
{

	if( IS_LL_CONNECTION_VALID(connHandle) ){
		blc_cyrpt_para.st = SLAVE_LL_REJECT_IND_T;
		return BLE_SUCCESS;
	}
	else {
		return HCI_ERR_UNKNOWN_CONN_ID;
	}

}




//------------------- Interrupt --------------------------------------------
_attribute_data_retention_	u8			blt_buff_conn[24] = {0};

/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void irq_blc_slave_tx(void)
{
//	static u32 no_tx_data;
//	no_tx_data++;


#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE)
	if (bltc.conn_stop_brx && bltParam.ble_state == BLE_STATE_BRX_S)
	{

		STOP_RF_STATE_MACHINE;			//stop more data
		if (bltc.conn_enc_dec_busy)			//security decoding ongoing
		{
			bltc.conn_pkt_dec_pending = blt_buff_conn;
		}
		else
		{
			blt_ll_channel_conn_update (blt_buff_conn);
		}

		bltc.conn_stop_brx = 0;
		bltParam.tx_irq_proc_en = 0;
	}
#endif
}


/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void bls_ll_procUpdateReq(u8 *raw_pkt)
{
	smemcpy (blt_buff_conn, raw_pkt + DMA_RFRX_OFFSET_HEADER, 24);
	if (blc_cyrpt_para.enable)
	{
		bltc.conn_pkt_rcvd_no = bltc.conn_pkt_rcvd;
		bltc.conn_stop_brx = 1;
		bltParam.tx_irq_proc_en = 1;
	}
	else											//decode immediately
	{
		blt_ll_channel_conn_update (blt_buff_conn);
	}
}




_attribute_ram_code_ int  irq_blc_slave_rx_data(u8 *raw_pkt, u32 tick_now)  //in ram 12us, bot in ram 120us
{

	int next_buffer = 0;

	u8 rf_len = raw_pkt[DMA_RFRX_OFFSET_RFLEN];
	u8 llid = raw_pkt[DMA_RFRX_OFFSET_HEADER] & 3;

#if(LL_SN_NESN_MANAGE_BY_SOFTWARE)
	if(bltParam.drop_rx_data){
		blt_bb.save_flg = 2; //mark, not save sn_nesn && dma_tx_rptr in brx_post stage, during brx stage: avoid 1st rx's crc24 software check err
		STOP_RF_STATE_MACHINE;  //stop state machine
		// 8278 Need ensure reg system tick irq BIT<0:2> is 0;
		systimer_set_irq_capture(clock_time () + 800); //trigger timeout
	}
	else{
		blt_bb.save_flg = 1;  //mark
		blt_save_snnesn ();
		#if (MCU_CORE_TYPE == MCU_CORE_9518)
			blt_save_dma_tx_ptr();
		#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
			blt_save_dma_tx_rptr();
		#endif
	}
#endif



	if (bltParam.ble_state == BLE_STATE_BRX_S && !bltParam.drop_rx_data)
	{

		u32 timeStamp = reg_rf_timestamp;

		if(!bltc.tick_1st_rx && !blttcon.conn_rx_num && (u32)(tick_now - timeStamp) < (rf_len*bltPHYs.oneByte_us+1000)* SYSTEM_TIMER_TICK_1US){
			#if (MCU_CORE_TYPE == MCU_CORE_9518)
			log_event_irq(BLE_IRQ_DBG_EN, SLEV_slave_1stRx);
				//time_stamp is captured after access_code
				#if(LL_FEATURE_ENABLE_LE_2M_PHY || LL_FEATURE_ENABLE_LE_CODED_PHY)
					bltc.tick_1st_rx = (u32)(timeStamp - bltPHYs.prmb_ac_us*SYSTEM_TIMER_TICK_1US) | 0x01;
				#else
					bltc.tick_1st_rx = (u32)(timeStamp - 40*SYSTEM_TIMER_TICK_1US) | 0x01;
				#endif
			#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
				///actually here should be same as 9518. temporary use previous processing method. after test OK,change that.
				bltc.tick_1st_rx = (u32)(timeStamp - 80*SYSTEM_TIMER_TICK_1US) | 0x01;  //time_stamp is captured after access_code
			#endif
			bltPm.timing_synced = 1;
		}


		blttcon.conn_tick = clock_time ();

		u8 r_sn = (raw_pkt[DMA_RFRX_OFFSET_HEADER] >> 3) & 1;
		if ( r_sn != bltc.conn_sn_master || (rf_len && !bltc.last_rf_len) )
		{
			bltc.conn_sn_master = r_sn;
			bltc.last_rf_len = rf_len;

			bltc.master_not_ack_slaveAckUpReq = 0;
			bltc.conn_rcvd_ack_pkt = 1;


			if (rf_len > 0 || blt_smp_empty_pkt_entry)	 //non empty packet
			{	//flag handle of master connection
				raw_pkt[2] = BLS_CONN_HANDLE;
				next_buffer = 1;

				#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE)
					///// proc master update_req in irq ///////
					//conn param update  rf_len = 12   encryption rf_len = 16
					//map update         rf_len = 8    encryption rf_len = 12
					//phy update         rf_len = 5    encryption rf_len = 9
					//Other control packages do not have three values with rf_len of 8,12,16
					#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
						if( llid == 3 && (blc_cyrpt_para.enable ? (rf_len==9 || rf_len==12 || rf_len == 16)
																: (rf_len==5 || rf_len==8  || rf_len == 12)))
					#else
						if( llid == 3 && (rf_len==8 || rf_len==12 || rf_len==16))
					#endif
					{
						#if 1  //save ramcode 48 byte
							bls_ll_procUpdateReq(raw_pkt);
						#else
							smemcpy (blt_buff_conn, raw_pkt + DMA_RFRX_OFFSET_HEADER, 24);
							if (blc_cyrpt_para.enable)
							{
								bltc.conn_pkt_rcvd_no = bltc.conn_pkt_rcvd;
								bltc.conn_stop_brx = 1;
							}
							else											//decode immediately
							{
								blt_ll_channel_conn_update (blt_buff_conn);
							}
						#endif
					}
				#endif

				if (rf_len)
				{
					bltc.conn_pkt_rcvd++;
				}
			}
		}

		if(bltc.conn_new_param){
			bltc.conn_new_param = 0;
			bltPm.conn_no_suspend = 0;
		}
	}

	return  next_buffer;
}



_attribute_ram_code_ void irq_slave_system_timer(void)
{

		if(bltParam.blt_state != BLS_LINK_STATE_CONN){
			return ;
		}

		if (bltParam.ble_state == BLE_STATE_BRX_S)		//start of BRX -> BRX end
		{
			//if we use DLE or Coded PHY, we need to consider extending the width of the received packet window, otherwise
			//the 1st data received will not normally succeed and will require subsequent retransmission..
			#if (1) //delay the entry time of brx_post
				if(rf_receiving_flag()){
					#if (LL_FEATURE_ENABLE_LE_CODED_PHY)
						if(blt_conn_phy.conn_cur_phy == BLE_PHY_CODED){ //timing for Coded PHY S8, do not care about S2
							//only support rf_len max to 27 bytes(DLE not support)
							//rf_len = 27 -> 2448 uS
							systimer_set_irq_capture( clock_time () + 2000*SYSTEM_TIMER_TICK_1US);
							return;
						}
						//maxRx is bigger than 100 bytes(RX window 1mS, consider some margin, 880us used for RX, corresponding to 100bytes 1M packet )
						else if(bltData.connRxDiff100)
					#else
						if(bltData.connRxDiff100)
					#endif
						{
							systimer_set_irq_capture( clock_time () + (bltData.connRxDiff100 * 8 + 200)*SYSTEM_TIMER_TICK_1US);  //1M packet 8us per byte, 200us for margin
							return;
						}
				}
			#endif

		#if (MCU_CORE_TYPE == MCU_CORE_9518)
			if(!blt_miscParam.pad32k_en)
			{
				if(ble_first_rx_tick_last != ble_first_rx_tick_pre){
					ble_actual_conn_interval_tick +=  bltc.conn_interval;
				}
			}
		#endif
			DBG_CHN1_LOW;
			blt_brx_post ();

			log_task_irq (BLE_IRQ_DBG_EN, SL01_brx, 0);
#if BLT_ADV_IN_CONN_SLAVE_EN
			if( bltParam.adv_extension_mask & BLS_FLAG_ADV_IN_SLAVE_MODE )
			{
			   // if(ll_module_advSlave_cb){  //save ramcode 4 byte
					#if (MCU_CORE_TYPE == MCU_CORE_9518)
						bltParam.blc_state = BLE_STATE_ADV_IN_SLAVE;
					#endif
					ll_module_advSlave_cb(); ///blc_ll_sendAdvInSlaveRole
			   // }
			}
#endif
#if BLT_SCAN_IN_CONN_SLAVE_EN
			if( blts.scan_extension_mask & BLS_FLAG_SCAN_IN_SLAVE_MODE )
			{
				#if (MCU_CORE_TYPE == MCU_CORE_9518)
					bltParam.blc_state = BLE_STATE_SCAN_IN_SLAVE;
				#endif
				blc_ll_switchScanChannel(0, 0);
			}
#endif

			bltParam.blt_busy = 0;


			if(bltParam.blt_state == BLS_LINK_STATE_CONN){
				systimer_set_irq_capture(bltc.connExpectTime - bltc.conn_tolerance_time);

				#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
					bltParam.acl_conn_start_tick = bltc.connExpectTime - bltc.conn_tolerance_time;
					bltParam.btxbrx_status = BTXBRX_NEARBY;
				#endif
			}
			else{  // adv/idle state

				systimer_clr_irq_status();
				systimer_irq_disable();

				#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
					bltParam.btxbrx_status = BTXBRX_NONE;
					bltParam.conn_role = LL_ROLE_NONE;
				#endif
			}

		}
		else if(bltParam.ble_state == BLE_STATE_BRX_E)		//BRX start
		{

			bltParam.blt_busy = 1;
			CLEAR_ALL_RFIRQ_STATUS;

			bltParam.ble_state = BLE_STATE_BRX_S;
		#if (MCU_CORE_TYPE == MCU_CORE_9518)
			bltParam.blc_state = BLE_STATE_BRX_S;
		#endif

			DBG_CHN1_HIGH;
			log_task_irq (BLE_IRQ_DBG_EN, SL01_brx, 1);
			blt_brx_start ();

			systimer_set_irq_capture(clock_time () + bltc.conn_duration);

			#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
				bltParam.btxbrx_status = BTXBRX_BUSY;
			#endif
		}
}



#if(DEBUG_TERMINATE_CNT_EN)
	void blt_debugTerminateCount(void)
	{
		u32 ter_cnt =0;
		flash_read_page(0x78000,4,(u8*)&ter_cnt);

		if(ter_cnt == 0xffff){
			ter_cnt = 0;
		}

		ter_cnt++;
		flash_erase_sector(0x78000);
		flash_write_page(0x78000, 4, (u8*)&ter_cnt);

	}
#endif

void bls_procPendingEvent(void)
{


	//terminate
	if(blc_tlkEvent_pending & EVENT_MASK_TERMINATE ){
		blc_tlkEvent_pending &= ~EVENT_MASK_TERMINATE;

		blc_ll_setEncryptionBusy (0);   //clear enc busy flag again
		blt_ll_version_ind_rsp_flag = 0;
		bltc.remoteVersionFlag = 0;

		if(ll_connTerminate_handler){
			ll_connTerminate_handler(BLS_CONN_HANDLE, &bltc.conn_terminate_reason);
		}

	#if(DEBUG_TERMINATE_CNT_EN)
		//just for debug
		blt_debugTerminateCount();
	#endif

		blt_p_event_callback (BLT_EV_FLAG_TERMINATE, &bltc.conn_terminate_reason, 1);

		if(hci_eventMask & HCI_EVT_MASK_DISCONNECTION_COMPLETE){
			u8 result[4];
			hci_disconnectionCompleteEvt_t *pEvt = (hci_disconnectionCompleteEvt_t *)result;

			pEvt->status = BLE_SUCCESS;
			//adv_init clears the handle value after terminate, so bltc.connhandle cannot be used when events are cache to mainlop
			//pEvt->connHandle = bltc.connHandle;
			pEvt->connHandle = BLS_CONN_HANDLE;
			pEvt->reason = bltc.conn_terminate_reason;

			blc_hci_send_event ( HCI_FLAG_EVENT_BT_STD | HCI_EVT_DISCONNECTION_COMPLETE, result, 4);
		}
	}


	if(blc_tlkEvent_pending & EVENT_MASK_CHN_MAP_REQ ){
		blt_p_event_callback (BLT_EV_FLAG_CHN_MAP_REQ, blttcon.conn_chn_map, 5);  //old map
		blc_tlkEvent_pending &= ~EVENT_MASK_CHN_MAP_REQ;
	}

	if(blc_tlkEvent_pending & EVENT_MASK_CONN_PARA_REQ ){
		blc_tlkEvent_pending &= ~EVENT_MASK_CONN_PARA_REQ;
		// 1	 	2		2		 2        2			2
		//winSize winOff  interval	latecny timeout  instant
		blt_p_event_callback (BLT_EV_FLAG_CONN_PARA_REQ, blc_tlkEvent_data + 3, 11);
	}


	//chn map update
	if(blc_tlkEvent_pending & EVENT_MASK_CHN_MAP_UPDATE ){
		blc_tlkEvent_pending &= ~EVENT_MASK_CHN_MAP_UPDATE;

		blt_p_event_callback (BLT_EV_FLAG_CHN_MAP_UPDATE, blttcon.conn_chn_map, 5);  //new map
	}

	//conn param update
	if(blc_tlkEvent_pending & EVENT_MASK_CONN_PARA_UPDATE ){
		blc_tlkEvent_pending &= ~EVENT_MASK_CONN_PARA_UPDATE;

		blt_p_event_callback (BLT_EV_FLAG_CONN_PARA_UPDATE, (u8 *)&bltc.conn_interval_next, 6);  //need change ?


		if(hci_le_eventMask & HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE){

			u8 result[10];
			hci_le_connectionUpdateCompleteEvt_t *pEvt = (hci_le_connectionUpdateCompleteEvt_t *)result;

			pEvt->subEventCode = HCI_SUB_EVT_LE_CONNECTION_UPDATE_COMPLETE;
			pEvt->status = BLE_SUCCESS;
			pEvt->connHandle = bltc.connHandle;
			pEvt->connInterval = bltc.conn_interval_next;
			pEvt->connLatency = bltc.conn_latency_next;
			pEvt->supervisionTimeout = bltc.conn_timeout_next;

			blc_hci_send_event (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META, result, 10);
		}
	}
}



int	blt_slave_main_loop_data (u8 *raw_pkt)
{
	if ( !blc_cyrpt_para.mic_fail)
	{
		blt_ll_packet_proc (raw_pkt);
	}

	return 0;
}


_attribute_data_retention_	u32  blt_dleSendTime_tick = 1100 * SYSTEM_TIMER_TICK_1MS;
void   blc_ll_setDataLengthReqSendingTime_after_connCreate(int time_ms)
{
	blt_dleSendTime_tick = time_ms * SYSTEM_TIMER_TICK_1MS;
}


int	blt_slave_main_loop_post (void)
{


#if (LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION)
	if(bltData.connMaxTxRxOctets_req == DATA_LENGTH_REQ_PENDING){
		if(bltParam.blt_state==BLS_LINK_STATE_CONN && !blc_ll_isEncryptionBusy() && ((u32)(clock_time() - bltc.conn_start_tick) > blt_dleSendTime_tick)){
			blt_ll_exchangeDataLength(LL_LENGTH_REQ, bltData.connMaxTxOctets);
		}
	}
#endif


	if(blc_tlkEvent_pending){
		bls_procPendingEvent();
	}





#if (LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN)
	if(    bltParam.blt_state == BLS_LINK_STATE_CONN && bltParam.tick_LE_Authenticated_Payload \
		&& clock_time_exceed(bltParam.tick_LE_Authenticated_Payload, bltParam.to_us_LE_Authenticated_Payload)){

		bltParam.tick_LE_Authenticated_Payload = clock_time() | 1;

		//send ll ping cmd to peerdevice
		u16 dat[8];
		dat[0] = 0x0103;		//type, len
		dat[1] = LL_PING_REQ;
		bls_ll_pushTxFifo (BLS_CONN_HANDLE | HANDLE_STK_FLAG, (u8 *)dat);
	}
#endif







	return 1;
}


//remove from ll_stack.h to ll_slave.h
void blc_ll_resetInfoRSSI(void)
{
	bltParam.ll_recentAvgRSSI = 0;
}


//move from conn_stack.h, for use API
#if (TRY_FIX_ERR_BY_ADD_BRX_WAIT)
	void blc_pm_modefy_brx_early_set(int us)
	{
		CONN_SHORT_TOLERANCE_TIME = us * SYSTEM_TIMER_TICK_1US;
	}

	int blc_pm_get_brx_early_time(void)
	{
		return CONN_SHORT_TOLERANCE_TIME;
	}
#endif






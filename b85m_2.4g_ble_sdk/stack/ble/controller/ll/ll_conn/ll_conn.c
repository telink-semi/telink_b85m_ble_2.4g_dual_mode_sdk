/********************************************************************************************************
 * @file	ll_conn.c
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

_attribute_data_retention_	_attribute_aligned_(4)	st_ll_conn_t		blttcon;
_attribute_data_retention_	_attribute_aligned_(4)	st_ll_conn_t	   *blt_pconn = &blttcon;
_attribute_data_retention_	_attribute_aligned_(4)	ll_push_fifo_handler_t ll_push_tx_fifo_handler = NULL;
_attribute_data_retention_	_attribute_aligned_(4)	u8 blt_ll_version_ind_rsp_flag = 0;

extern blt_event_callback_t		blt_p_event_callback ;


void blc_ll_initConnection_module(void)
{

#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	ll_get_conn_phy_ptr_cb = blt_ll_get_conn_phy_ptr;
#endif
}


#if (MCU_CORE_TYPE == MCU_CORE_9518)  //Kite/Vulture not use
ble_sts_t blc_ll_initAclConnTxFifo(u8 *pTxbuf, int size, int number)
{
	bltParam.ll_aclTxFifo_set = 1;

	if( number == 9 ){
		blt_txfifo.depth = 3;
		blt_txfifo.num = 8;
		blt_txfifo.mask = 7;
	}
	else if( number == 17 ){
		blt_txfifo.depth = 4;
		blt_txfifo.num = 16;
		blt_txfifo.mask = 15;
	}
	else if( number == 33 ){
		blt_txfifo.depth = 5;
		blt_txfifo.num = 32;
		blt_txfifo.mask = 31;
	}
	else{
		//4, 2 is too small
		return LL_ERR_INVALID_PARAMETER;
	}



	/* size must be 16*n */
	if( (size & 15) == 0){
		blt_txfifo.size = size;
	}
	else{
		return LL_ERR_INVALID_PARAMETER;
	}



	blt_txfifo.wptr = blt_txfifo.rptr = 0;
	blt_txfifo.p_default = pTxbuf;
	blt_txfifo.p_base =  (u8*)(pTxbuf + blt_txfifo.size);

	smemcpy( blt_txfifo.p_default,(u8 *)blt_tx_empty_packet,6);

	return BLE_SUCCESS;
}


ble_sts_t blc_ll_initAclConnRxFifo(u8 *pRxbuf, int size, int number)
{
	bltParam.ll_aclRxFifo_set = 1;

	/* number must be 2^n */
	if( IS_POWER_OF_2(number) && number > 3){
		blt_rxfifo.num = number;
		blt_rxfifo.mask = number - 1;
	}
	else{
		return LL_ERR_INVALID_PARAMETER;
	}



	/* size must be 16*n */
	if( (size & 15) == 0){
		blt_rxfifo.size = size;
	}
	else{
		return LL_ERR_INVALID_PARAMETER;
	}


	blt_rxfifo.wptr = blt_rxfifo.rptr = 0;
	blt_rxfifo.p_base = pRxbuf;




	bltParam.acl_rx_dma_buff = (u32)(blt_rxfifo.p_base + (blt_rxfifo.wptr & blt_rxfifo.mask) * blt_rxfifo.size);
	bltParam.acl_rx_dma_size = blt_rxfifo.size >> 4;





	return BLE_SUCCESS;
}


ble_sts_t	blc_ll_setAclConnMaxOctetsNumber(u8 maxRxOct, u8 maxTxOct)
{
	if(maxRxOct < MAX_OCTETS_DATA_LEN_27 || maxRxOct > MAX_OCTETS_DATA_LEN_EXTENSION){
		return HCI_ERR_INVALID_HCI_CMD_PARAMS;
	}

	if(maxTxOct < MAX_OCTETS_DATA_LEN_27 || maxTxOct > MAX_OCTETS_DATA_LEN_EXTENSION){
		return HCI_ERR_INVALID_HCI_CMD_PARAMS;
	}

	bltParam.maxRxOct = maxRxOct;
	bltParam.maxTxOct = maxTxOct;

	return BLE_SUCCESS;
}
#endif



_attribute_ram_code_ int blt_ll_connect_common(st_ll_conn_t *pc, rf_packet_connect_t * pInit)
{
	pc->conn_access_code_revert = pInit->accessCode[3] | (pInit->accessCode[2]<<8) | (pInit->accessCode[1]<<16) | (pInit->accessCode[0]<<24);
	smemcpy( (char*)&pc->conn_access_code, (char*)pInit->accessCode, 4);
	smemcpy( (char*)&pc->conn_crc, (char*)pInit->crcinit, 3);

	pc->conn_chn_hop = pInit->hop & 0x1f;
	smemcpy ( (char*)pc->conn_chn_map, (char*)pInit->chm, 5);

#if (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)
	if(bltParam.local_chSel){  // event must be generated, as long as local device support CSA2
		blc_tlkEvent_pending |= EVENT_MASK_CHN_SELECTION_ALGOTITHM;
	}

	if(bltParam.peer_chSel && bltParam.local_chSel)		//if(p->header.chan_sel && bltParam.local_chSel)
	{
		pc->conn_chnsel = 1;

		u32 conn_access_code;
		smemcpy ( (char*)&conn_access_code, (char*)pInit->accessCode, 4);
		pc->channel_id = (conn_access_code>>16) ^ (conn_access_code&0xffff);
		blc_calc_remapping_table(pc->conn_chn_map);//A remapping table is built that contains all the used channels.
	}
	else
	{
		pc->conn_chnsel = 0;

		pc->chn_idx = 0;
		blt_ll_channelTable_calc (pInit->chm, pc->conn_chn_hop, pc->chn_tbl);
	}
#else

	pc->chn_idx = 0;
	blt_ll_channelTable_calc (pInit->chm, pc->conn_chn_hop, pc->chn_tbl);
#endif

#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	blt_reset_conn_phy_param(&blt_conn_phy); //Reset conn_cur_phy and conn_cur_CI to the dft settings.
#endif

	pc->conn_inst = 0;
	blt_ll_version_ind_rsp_flag = 0;

	return 0;
}



_attribute_ram_code_ int blt_ll_start_common(st_ll_conn_t *pc)
{
	reg_rf_irq_mask = FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLG_RF_CONN_DONE;
	CLEAR_ALL_RFIRQ_STATUS; // clear all interrupt flag   //for slave: added by sihui 20190802()

#if (MCU_CORE_TYPE == MCU_CORE_9518)
	//reset_baseband();   //QiangKai: Eagle can not reset, all RF baseband setting will lost(But Kite/Vulture must add this)
#elif (MCU_CORE_TYPE == MCU_CORE_825x || MCU_CORE_TYPE == MCU_CORE_827x)
	reset_baseband();
#endif

	pc->conn_rx_num = 0;			//RX number (regardless of CRC correct or wrong)
	pc->conn_receive_packet = 0;		//RX with CRC correct
	pc->conn_receive_new_packet = 0;	//RX with CRC correct & new SN

#if (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)
	if(pc->conn_chnsel) //channel selection alg#2
	{
		pc->conn_chn = ll_chn_index_calc_cb(pc->conn_chn_map, pc->conn_inst, pc->channel_id);
	}
	else
	{
		pc->conn_chn = pc->chn_tbl[pc->chn_idx];
	}
#else
	//-----------      get next channel -------------------------
	blttcon.conn_chn = blttcon.chn_tbl[blttcon.chn_idx];
#endif

	// channel, CRC, Access Code
	rf_set_ble_channel(pc->conn_chn);

#if (MCU_CORE_TYPE == MCU_CORE_9518)
	reg_rf_rxtmaxlen = bltData.connEffectiveMaxRxOctets + 4;
#endif

#if(LL_FEATURE_ENABLE_LE_2M_PHY || LL_FEATURE_ENABLE_LE_CODED_PHY)
	if(ll_conn_phy_swicth_cb){
		ll_conn_phy_swicth_cb();
	}

	rf_set_ble_access_code_value(pc->conn_access_code_revert);
	rf_trigle_codedPhy_accesscode();
#else
	rf_set_ble_access_code_value(pc->conn_access_code_revert);
#endif

	rf_set_ble_crc_value(pc->conn_crc);   //save ram_code 12 byte compare to "rf_set_ble_crc"

	return 0;
}


/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518)  //for Eagle IRQ priority
_attribute_ram_code_
#endif
void blt_ll_channelTable_calc(u8* chm, u8 hop, u8 *ptbl) {
	u8 tableTemp[37], num = 0;
	foreach(k, 37) {
		if (chm[k >> 3] & BIT(k & 0x07)) {
			tableTemp[num++] = k;
		}
	}
	u8 k = 0, l = 0;
	foreach(i, 37) {
		k += hop;
		if (k >= 37) {
			k -= 37;
		}
		if (chm[k >> 3] & BIT(k & 0x07)) {
			ptbl[l] = k;
		} else {
			u8 m = k;
			while (m >= num) {
				m -= num;
			}
			ptbl[l] = tableTemp[m];
		}
		++l;
	}
}




void blc_procPendingEvent(void)
{
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	if(blc_tlkEvent_pending & EVENT_MASK_PHY_UPDATE ){
		blc_tlkEvent_pending &= ~EVENT_MASK_PHY_UPDATE;

		if(hci_le_eventMask & HCI_LE_EVT_MASK_PHY_UPDATE_COMPLETE)
		{
			hci_le_phyUpdateComplete_evt(blttcon.connHandle, BLE_SUCCESS, blt_conn_phy.conn_cur_phy);
		}

		u8 new_phy[2] = {blt_conn_phy.conn_cur_phy, blt_conn_phy.conn_cur_phy};
		blt_p_event_callback (BLT_EV_FLAG_PHY_UPDATE, new_phy, 2);

	}
#endif

#if(LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)
	if(blc_tlkEvent_pending & EVENT_MASK_CHN_SELECTION_ALGOTITHM)
	{
		blc_tlkEvent_pending &= ~EVENT_MASK_CHN_SELECTION_ALGOTITHM;
		if(hci_le_eventMask & HCI_LE_EVT_MASK_CHANNEL_SELECTION_ALGORITHM)
		{
			hci_le_chennel_selection_algorithm_evt(blttcon.connHandle,blttcon.conn_chnsel);
		}
	}
#endif

#if LL_FEATURE_ENABLE_LE_DATA_LENGTH_EXTENSION
	if(blc_tlkEvent_pending & EVENT_MASK_DATA_LEN_UPDATE)
	{
		if (hci_le_eventMask & HCI_LE_EVT_MASK_DATA_LENGTH_CHANGE) {
			hci_le_data_len_update_evt(blttcon.connHandle, bltData.connEffectiveMaxTxOctets, bltData.connEffectiveMaxRxOctets,
				LL_PACKET_OCTET_TIME(bltData.connEffectiveMaxTxOctets),LL_PACKET_OCTET_TIME(bltData.connEffectiveMaxRxOctets));
			//todo: change txtime/rxtime according to DLE result
		}
		blc_tlkEvent_pending &= ~EVENT_MASK_DATA_LEN_UPDATE;
	}
#endif

	//rx data abondom
	if(blc_tlkEvent_pending & EVENT_MASK_RX_DATA_ABANDOM ){
		blc_tlkEvent_pending &= ~EVENT_MASK_RX_DATA_ABANDOM;

		blt_p_event_callback (BLT_EV_FLAG_RX_DATA_ABANDOM, NULL, 0);
	}

}










int blt_ll_conn_main_loop_post(void)
{

#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)
	if(blt_conn_phy.phy_req_pending){
		blt_ll_sendPhyReq();
	}
#endif

#if (SONOS_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
	if(bltParam.wirte_sonos_flash_req){
		if(clock_time_exceed(bltParam.wirte_sonos_flash_req, 500 * 1000)){  //decrease bandwith for 500mS
			bltParam.wirte_sonos_flash_req = 0;
		}
	}
#endif

	return 0;
}





ble_sts_t	blt_ll_unknown_rsp(u16 connHandle, u8 op_code )
{

	u8 temp[4];
	rf_packet_ll_unknown_rsp_t *pCtrl = (rf_packet_ll_unknown_rsp_t* )temp;
	pCtrl->type = LLID_CONTROL;
	pCtrl->rf_len = 2;
	pCtrl->opcode = LL_UNKNOWN_RSP;
	pCtrl->unknownType = op_code;

	ll_push_tx_fifo_handler(connHandle, (u8*)pCtrl);


	return BLE_SUCCESS;
}








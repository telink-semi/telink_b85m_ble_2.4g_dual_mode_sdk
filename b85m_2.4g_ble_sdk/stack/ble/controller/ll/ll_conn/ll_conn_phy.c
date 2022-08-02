/********************************************************************************************************
 * @file	ll_conn_phy.c
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




/*******************************  2M PHY, Coded PHY  *********************************/
#if (LL_FEATURE_ENABLE_LE_2M_PHY | LL_FEATURE_ENABLE_LE_CODED_PHY)




_attribute_data_retention_	_attribute_aligned_(4) ll_conn_phy_t blt_conn_phy;

_attribute_data_retention_ 	ll_conn_phy_update_callback_t	ll_conn_phy_update_cb = NULL;
_attribute_data_retention_ 	ll_conn_phy_switch_callback_t	ll_conn_phy_swicth_cb = NULL;


ll_conn_phy_t*	blt_ll_get_conn_phy_ptr(u16 connHandle)
{
	return &blt_conn_phy;
}





void blt_ll_sendPhyReq(void)
{
	if(blt_conn_phy.phy_req_pending && !blc_ll_isEncryptionBusy() ){ //
		u8 phy_req_dat[6];
		rf_pkt_ll_phy_req_rsp_t *pReq = (rf_pkt_ll_phy_req_rsp_t* )phy_req_dat;
		pReq->llid = LLID_CONTROL;
		pReq->rf_len = 3;
		pReq->opcode = LL_PHY_REQ;
		pReq->tx_phys = pReq->rx_phys = blt_conn_phy.conn_prefer_phys;

		if (ll_push_tx_fifo_handler(HANDLE_STK_FLAG, phy_req_dat)) {  //connHandle no use, due to single connection
			blt_conn_phy.phy_req_pending = 0;
		}
	}

}



_attribute_ram_code_ int blt_ll_updateConnPhy(void)
{
	// LE Set PHY Command send by Host/Application, PHY Update Event must be generated
	// PHY changed, PHY Update Event must be generated
#if CERT_SCHEME
	blc_tlkEvent_pending |= EVENT_MASK_PHY_UPDATE;
	if(blt_conn_phy.conn_cur_phy != blt_conn_phy.conn_next_phy)
	{
		bltData.connEffectiveMaxRxOctets = bltData.connEffectiveMaxTxOctets = bltData.connInitialMaxTxOctets;
		blc_tlkEvent_pending |= EVENT_MASK_DATA_LEN_UPDATE;
	}
#else
	if(blt_conn_phy.phy_req_trigger || blt_conn_phy.conn_cur_phy != blt_conn_phy.conn_next_phy){
		blt_conn_phy.phy_req_trigger = 0;
		blc_tlkEvent_pending |= EVENT_MASK_PHY_UPDATE;
	}
#endif
	blt_conn_phy.conn_cur_phy = blt_conn_phy.conn_next_phy;  //new PHY used
	if(blt_conn_phy.conn_next_CI){
		blt_conn_phy.conn_cur_CI = blt_conn_phy.conn_next_CI;
		blt_conn_phy.conn_next_CI = 0;
	}

	return 0;
}


//_attribute_data_retention_ int BBBBBBBB = 0;
_attribute_ram_code_ int blt_ll_switchConnPhy(void)
{
	if(blt_conn_phy.conn_cur_phy != bltPHYs.cur_llPhy || (blt_conn_phy.conn_cur_CI != bltPHYs.cur_CI && blt_conn_phy.conn_cur_phy == BLE_PHY_CODED)){
		ll_phy_switch_cb(blt_conn_phy.conn_cur_phy, blt_conn_phy.conn_cur_CI);
	}

	return 0;
}
















#endif  //end of  LL_FEATURE_ENABLE_ CHANNEL_SELECTION_ALGORITHM2

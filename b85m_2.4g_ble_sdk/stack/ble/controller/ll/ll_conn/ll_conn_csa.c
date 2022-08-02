/********************************************************************************************************
 * @file	ll_conn_csa.c
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



#if (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2)



_attribute_data_retention_	u8 g_remapping_table[37];
_attribute_data_retention_	u8 g_num_used_chans = 0;

_attribute_data_retention_	ll_chn_index_calc_callback_t	ll_chn_index_calc_cb = NULL;




void blc_ll_initChannelSelectionAlgorithm_2_feature(void)
{
	LL_FEATURE_MASK_0 |= (LL_FEATURE_ENABLE_CHANNEL_SELECTION_ALGORITHM2	<<14);
	bltParam.local_chSel = CHANNAL_SELECTION_ALGORITHM_2;

	ll_chn_index_calc_cb = blc_ll_channel_index_calc_csa2;
}






/////////////////////////////////////// LE CSA2 //////////////////////////////////////////////
/**********************************************************
 *	@brief	CSA2:Byte flipping
 *	@param	8bit input
 *	@return	8bit output
 */
_attribute_ram_code_ static u8  reverse_u8(register u8 x)
{
    x = (((x & 0xaa) >> 1) | ((x & 0x55) << 1));
    x = (((x & 0xcc) >> 2) | ((x & 0x33) << 2));
    return((x >> 4) | (x << 4));
}

/**********************************************************
 *	@brief	CSA2:Permutation operation
 *	@param	16bit input
 *	@return	16bit output
 *	@reference core 5.0 | Vol 6, Part B page 2646
 */
_attribute_ram_code_ static u16 blt_ll_conn_csa2_perm(u16 in)
{
    u16 out = 0;
    out = reverse_u8(in>>8)<<8 | reverse_u8(in&0xff);

    return out;
}

/**********************************************************
 *	@brief	CSA2: Event pseudo-random number generation
 *	@param 	counter: The 16-bit input counter changes for each event.
 *	                 For periodic advertising it is the event counter paEventCounter
 *	        ch_id  : The 16-bit input channelIdentifier is fixed for any given connection or
 *					 periodic advertising; it is calculated from the Access Address by:
 *					 channelIdentifier = (Access Address31-16) XOR (Access Address15-0)
 *	@return	prn_e
 *	@reference core 5.0 | Vol 6, Part B page 2646
 */
_attribute_ram_code_ static u16	blt_ll_conn_csa2_prng(u16 counter, u16 ch_id)
{
    u16 prn_e;

    prn_e = counter ^ ch_id;

    prn_e = blt_ll_conn_csa2_perm(prn_e);
    prn_e = (prn_e * 17) + ch_id;

    prn_e = blt_ll_conn_csa2_perm(prn_e);
    prn_e = (prn_e * 17) + ch_id;

    prn_e = blt_ll_conn_csa2_perm(prn_e);
    prn_e = (prn_e * 17) + ch_id;

    prn_e = prn_e ^ ch_id;

    return prn_e;
}

/*	This code in RF irq and system irq put in RAM by force
 * Because of the flash resource contention problem, when the
 * flash access is interrupted by a higher priority interrupt,
 * the interrupt processing function cannot operate the flash
*/
#if (MCU_CORE_TYPE == MCU_CORE_9518) //for Eagle IRQ priority
_attribute_ram_code_
#endif
u8 blc_calc_remapping_table(u8 chm[5]){
	/*
	 * A remapping table is built that contains all the used channels in ascending
	 * order, indexed from zero.
	 */
	g_num_used_chans = 0;
	foreach(k, 37)
	{
		if(chm[k>>3] & BIT(k & 0x07)){
			g_remapping_table[g_num_used_chans++] = k;
		}
	}

	return 0;
}

/**********************************************************
 *	@brief	CSA2: Obtain the channel index for the event.
 *	@param 	counter: The 16-bit input counter changes for each event.
 *	                 For periodic advertising it is the event counter paEventCounter
 *	        ch_id  : The 16-bit input channelIdentifier is fixed for any given connection or
 *					 periodic advertising; it is calculated from the Access Address by:
 *					 channelIdentifier = (Access Address31-16) XOR (Access Address15-0)
 *	@return	The channel index for the event.
 *	@reference core 5.0 | Vol 6, Part B page 2646
 */
_attribute_ram_code_ u8 blc_ll_channel_index_calc_csa2(u8 chm[5], u16 event_cntr, u16 channel_id)
{
    u16 channel_unmapped;
    u8  remap_index;
    u16 prn_e;

    prn_e = blt_ll_conn_csa2_prng(event_cntr, channel_id);

    channel_unmapped = prn_e % 37;

    /*
     * If unmappedChannel is the channel index of a used channel according to the
     * channel map, it is used as the channel index for the event.
     */
    if(chm[channel_unmapped>>3] & BIT(channel_unmapped & 0x07)){
        return channel_unmapped;
    }

	/*
	 * If unmappedChannel is the index of an unused channel according to the channel
	 * map, then the channel index for the event is calculated from prn_e and N (the
	 * number of used channels) by first calculating the value remappingIndex as:
	 */
//    remap_index = (g_num_used_chans * prn_e) / 0x10000;
    remap_index = (g_num_used_chans * prn_e) >> 16;

    /*
     * Then using remappingIndex as an index into the remapping table to obtain
     * the channel index for the event.
     */
    return g_remapping_table[remap_index];
}










#endif  //end of  LL_FEATURE_ENABLE_ CHANNEL_SELECTION_ALGORITHM2

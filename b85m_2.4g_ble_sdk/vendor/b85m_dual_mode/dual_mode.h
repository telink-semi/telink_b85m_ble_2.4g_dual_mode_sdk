/********************************************************************************************************
 * @file	dual_mode.h
 *
 * @brief	This is the header file for BLE SDK
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
#ifndef DUAL_MODE_H_
#define DUAL_MODE_H_

// ---------- GFSK RX MODE ENUM -----------
#define GFSK_RX_MODE_ENUM_NORMAL		1
#define GFSK_RX_MODE_ENUM_SRX			2

// ---------- DUAL MODE STATE ENUM --------
typedef enum{
	DUAL_MODE_ST_BLE					= 0,
	DUAL_MODE_ST_GFSK					= 1,
	DUAL_MODE_ST_GFSK_ONLY				= 2,
}dual_mode_st_e;

extern dual_mode_st_e 		s_dual_mode_st;
extern my_fifo_t			scan_rx_fifo;
extern u8 					scan_rx_fifo_b[];

static inline int is_gfsk_only_mode()
{
	return (DUAL_MODE_ST_GFSK_ONLY == s_dual_mode_st);
}

/**
*	@brief     This function serves to reboot RF power
*	@param[in] none.
*	@return	   none.
*/
static inline void reset_rf_power(void)
{
	analog_write(0x34,0x81);  		// power off
 	analog_write(0x34,0x80);  		// power on
}

void dual_mode_init();
void dual_mode_main_loop();
int dual_mode_gfsk_rf_irq_handler (void);
void dual_mode_blt_brx_start_init ();

void switch_to_gfsk_only_mode();


#endif /* DUAL_MODE_H_ */

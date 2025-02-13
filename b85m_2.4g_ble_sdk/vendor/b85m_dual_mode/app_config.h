/********************************************************************************************************
 * @file	app_config.h
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
#pragma once

///////////////////////// Dual Mode Configuration//////////////////////////////////////////////
// -- dual mode with ble sample and 2.4g srx
#define DUAL_MODE_ENABLE								1
#if DUAL_MODE_ENABLE
#define GFSK_RX_MODE_SEL								GFSK_RX_MODE_ENUM_SRX
#define GFSK_RX_CHN										4
#endif

///////////////////////// Feature Configuration////////////////////////////////////////////////
#if DUAL_MODE_ENABLE
#define BLE_APP_PM_ENABLE								0
#define PM_DEEPSLEEP_RETENTION_ENABLE					0
#define TEST_CONN_CURRENT_ENABLE            			0 //test connection current, disable UI to have a pure power
#define BLE_REMOTE_SECURITY_ENABLE      				0
#define BLE_REMOTE_OTA_ENABLE							1
#else
#define BLE_APP_PM_ENABLE								1
#define PM_DEEPSLEEP_RETENTION_ENABLE					1
#define TEST_CONN_CURRENT_ENABLE            			0 //test connection current, disable UI to have a pure power
#define BLE_REMOTE_SECURITY_ENABLE      				1
#endif

/////////////////////// Sample Test Board Select Configuration ///////////////////////////////
#define BOARD_825X_EVK_C1T139A30						1     //TLSR8258DK48
#define BOARD_825X_DONGLE_C1T139A3						2     //
#define BOARD_827X_EVK_C1T197A30						3	  //TLSR8278DK48
#define BOARD_827X_DONGLE_C1T201A3						4	  //



#if (__PROJECT_8258_DUAL_MODE__)
	/* can only choose BOARD_825X_EVK_C1T139A30 or BOARD_825X_DONGLE_C1T139A3,
	 * default use EVK, user can select Dongle */
	#if DUAL_MODE_ENABLE
	#define BOARD_SELECT							BOARD_825X_DONGLE_C1T139A3
	#else
	#define BOARD_SELECT							BOARD_825X_EVK_C1T139A30
	#endif
#elif (__PROJECT_8278_DUAL_MODE__)
	/* can only choose BOARD_827X_EVK_C1T197A30 or BOARD_827X_DONGLE_C1T201A3,
	 * default use EVK, user can select Dongle */
	#if DUAL_MODE_ENABLE
	#define BOARD_SELECT							BOARD_827X_DONGLE_C1T201A3
	#else
	#define BOARD_SELECT							BOARD_827X_EVK_C1T197A30
	#endif
#endif


///////////////////////// DEBUG  Configuration ////////////////////////////////////////////////
#define DEBUG_GPIO_ENABLE								0

#define UART_PRINT_DEBUG_ENABLE             			0
#if UART_PRINT_DEBUG_ENABLE
#define DEBUG_INFO_TX_PIN								GPIO_PB2
#define	PB2_FUNC										AS_GPIO
#define PB2_INPUT_ENABLE								0
#define PB2_OUTPUT_ENABLE								1
#define	PULL_WAKEUP_SRC_PB2								PM_PIN_PULLUP_1M
#define PB2_DATA_OUT									1
#endif


///////////////////////// UI Configuration ////////////////////////////////////////////////////
#if (TEST_CONN_CURRENT_ENABLE)
	#define	UI_KEYBOARD_ENABLE							0
	#define	UI_BUTTON_ENABLE							0
	#define	UI_LED_ENABLE								0
#else
	#if(BOARD_SELECT == BOARD_825X_EVK_C1T139A30 || BOARD_SELECT == BOARD_827X_EVK_C1T197A30)
		/* EVK use keyboard matrix */
		#if DUAL_MODE_ENABLE
		#define	UI_KEYBOARD_ENABLE						0
		#else
		#define	UI_KEYBOARD_ENABLE						1
		#endif
	#elif(BOARD_SELECT == BOARD_825X_DONGLE_C1T139A3 || BOARD_SELECT == BOARD_827X_DONGLE_C1T201A3)
		/* Dongle use button */
		#if DUAL_MODE_ENABLE
		#define	UI_BUTTON_ENABLE						0
		#else
		#define	UI_BUTTON_ENABLE						1
		#endif
	#endif

	//#define UI_LED_ENABLE								1  //enable LED if you want
#endif


#ifndef	UI_KEYBOARD_ENABLE
#define UI_KEYBOARD_ENABLE								0
#endif

#ifndef	UI_BUTTON_ENABLE
#define UI_BUTTON_ENABLE								0
#endif

#ifndef	UI_LED_ENABLE
#define UI_LED_ENABLE									0
#endif



/**
 *  @brief  Keyboard Configuration
 */
#if (UI_KEYBOARD_ENABLE)   // if test pure power, kyeScan GPIO setting all disabled
	#define	MATRIX_ROW_PULL					PM_PIN_PULLDOWN_100K
	#define	MATRIX_COL_PULL					PM_PIN_PULLUP_10K
	#define	KB_LINE_HIGH_VALID				0   //dirve pin output 0 when keyscan, scanpin read 0 is valid

	#define			CR_VOL_UP				0xf0  ////
	#define			CR_VOL_DN				0xf1

	#define		KB_MAP_NORMAL	{	{CR_VOL_DN,		VK_1},	 \
									{CR_VOL_UP,		VK_2}, }


	//////////////////// KEY CONFIG (EVK board) ///////////////////////////
	#if (BOARD_SELECT == BOARD_825X_EVK_C1T139A30 || BOARD_SELECT == BOARD_827X_EVK_C1T197A30)
		/* 825X EVK and 827X EVK use same GPIO for KeyMatrix: PB2/PB3/PB4/PB5 */
		#define  KB_DRIVE_PINS  {GPIO_PB4, GPIO_PB5}
		#define  KB_SCAN_PINS   {GPIO_PB2, GPIO_PB3}

		//drive pin as gpio
		#define	PB4_FUNC				AS_GPIO
		#define	PB5_FUNC				AS_GPIO

		//drive pin need 100K pulldown
		#define	PULL_WAKEUP_SRC_PB4		MATRIX_ROW_PULL
		#define	PULL_WAKEUP_SRC_PB5		MATRIX_ROW_PULL

		//drive pin open input to read gpio wakeup level
		#define PB4_INPUT_ENABLE		1
		#define PB5_INPUT_ENABLE		1

		//scan pin as gpio
		#define	PB2_FUNC				AS_GPIO
		#define	PB3_FUNC				AS_GPIO

		//scan  pin need 10K pullup
		#define	PULL_WAKEUP_SRC_PB2		MATRIX_COL_PULL
		#define	PULL_WAKEUP_SRC_PB3		MATRIX_COL_PULL

		//scan pin open input to read gpio level
		#define PB2_INPUT_ENABLE		1
		#define PB3_INPUT_ENABLE		1
	#else
		#error "Current board do not support keyboard !"
	#endif


	#define		KB_MAP_NUM		KB_MAP_NORMAL
	#define		KB_MAP_FN		KB_MAP_NORMAL
#endif



/**
 *  @brief  Button Configuration
 */
#if (UI_BUTTON_ENABLE)
	#if (BOARD_SELECT == BOARD_825X_DONGLE_C1T139A3 || BOARD_SELECT == BOARD_827X_DONGLE_C1T201A3)
		/* 825X Dongle and 827X Dongle use same GPIO for Button: PD5/PD6 */
		#define	SW1_GPIO				GPIO_PD5
		#define	SW2_GPIO				GPIO_PD6
		#define PD5_FUNC				AS_GPIO
		#define PD6_FUNC				AS_GPIO
		#define PD5_INPUT_ENABLE		1
		#define PD6_INPUT_ENABLE		1
		#define PULL_WAKEUP_SRC_PD5     PM_PIN_PULLUP_10K
		#define PULL_WAKEUP_SRC_PD6     PM_PIN_PULLUP_10K
	#else
		#error "Current board do not support button !"
	#endif
#endif


/**
 *  @brief  LED Configuration
 */
#if DUAL_MODE_ENABLE
	#if (BOARD_SELECT == BOARD_825X_DONGLE_C1T139A3)
//red LED on:2.4G only mode. green led shining:ble adv receiving. blue led shining:2.4G receiving.
#define GREEN_LED_PIN           		GPIO_PA2	// just for test
#define RED_LED_PIN             		GPIO_PA3	// just for test
#define BLUE_LED_PIN             		GPIO_PB0	// just for test

#define PA2_FUNC						AS_GPIO
#define PA3_FUNC						AS_GPIO
#define PB0_FUNC						AS_GPIO

#define	PA2_OUTPUT_ENABLE				1
#define	PA3_OUTPUT_ENABLE				1
#define	PB0_OUTPUT_ENABLE				1

#define	PA2_DATA_OUT					0
#define	PA3_DATA_OUT					0
#define	PB0_DATA_OUT					0
	#else
// TODO
	#endif
#elif (UI_LED_ENABLE)
	#if (BOARD_SELECT == BOARD_825X_EVK_C1T139A30 || BOARD_SELECT == BOARD_827X_EVK_C1T197A30)
		/* 825X EVK and 827X EVK use same GPIO for LED: PD2/PD3/PD4/PD5 */
		#define	GPIO_LED_BLUE			GPIO_PD2
		#define	GPIO_LED_GREEN			GPIO_PD3
		#define	GPIO_LED_WHITE			GPIO_PD4
		#define	GPIO_LED_RED			GPIO_PD5

		#define PD2_FUNC				AS_GPIO
		#define PD3_FUNC				AS_GPIO
		#define PD4_FUNC				AS_GPIO
		#define PD5_FUNC				AS_GPIO

		#define	PD2_OUTPUT_ENABLE		1
		#define	PD3_OUTPUT_ENABLE		1
		#define PD4_OUTPUT_ENABLE		1
		#define	PD5_OUTPUT_ENABLE		1

		#define LED_ON_LEVAL 			1 		//gpio output high voltage to turn on led
	#elif (BOARD_SELECT == BOARD_825X_DONGLE_C1T139A3 || BOARD_SELECT == BOARD_827X_DONGLE_C1T201A3)
		/* 825X Dongle and 827X Dongle use same GPIO for LED: PA3/PB1/PA2/PB0/PA4 */
		#define	GPIO_LED_RED			GPIO_PA3
		#define	GPIO_LED_WHITE			GPIO_PB1
		#define	GPIO_LED_GREEN			GPIO_PA2
		#define	GPIO_LED_BLUE			GPIO_PB0
		#define	GPIO_LED_YELLOW			GPIO_PA4

		#define PA3_FUNC				AS_GPIO
		#define PB1_FUNC				AS_GPIO
		#define PA2_FUNC				AS_GPIO
		#define PB0_FUNC				AS_GPIO
		#define PA4_FUNC				AS_GPIO

		#define	PA3_OUTPUT_ENABLE		1
		#define	PB1_OUTPUT_ENABLE		1
		#define PA2_OUTPUT_ENABLE		1
		#define	PB0_OUTPUT_ENABLE		1
		#define	PA4_OUTPUT_ENABLE		1

		#define LED_ON_LEVAL 			1 		//gpio output high voltage to turn on led
	#endif
#endif





/////////////////// DEEP SAVE FLG //////////////////////////////////
#define USED_DEEP_ANA_REG                   DEEP_ANA_REG0 //u8,can save 8 bit info when deep
#define CONN_DEEP_FLG	                    BIT(0) //if 1: conn deep, 0: adv deep






///////////////////////// System Clock  Configuration /////////////////////////////////////////
#define CLOCK_SYS_CLOCK_HZ  								16000000

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	#define SYS_CLK_TYPE  									SYS_CLK_16M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	#define SYS_CLK_TYPE  									SYS_CLK_24M_Crystal
#elif(CLOCK_SYS_CLOCK_HZ == 32000000)
	#define SYS_CLK_TYPE  									SYS_CLK_32M_Crystal
#elif(CLOCK_SYS_CLOCK_HZ == 48000000)
	#define SYS_CLK_TYPE  									SYS_CLK_48M_Crystal
#else
	#error "unsupported system clock !"
#endif

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};



/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms




#if(DEBUG_GPIO_ENABLE)
	#if (BOARD_SELECT == BOARD_825X_EVK_C1T139A30)
		#define GPIO_CHN0							GPIO_PD0
		#define GPIO_CHN1							GPIO_PD1
		#define GPIO_CHN2							GPIO_PD6
		#define GPIO_CHN3							GPIO_PD7
		#define GPIO_CHN4							GPIO_PA2
		#define GPIO_CHN5							GPIO_PA3
		#define GPIO_CHN6							GPIO_PA4
		#define GPIO_CHN7							0

		#define PD0_OUTPUT_ENABLE					1
		#define PD1_OUTPUT_ENABLE					1
		#define PD6_OUTPUT_ENABLE					1
		#define PD7_OUTPUT_ENABLE					1
		#define PA2_OUTPUT_ENABLE					1
		#define PA3_OUTPUT_ENABLE					1
		#define PA4_OUTPUT_ENABLE					1
	#elif (BOARD_SELECT == BOARD_827X_EVK_C1T197A30)
		#define GPIO_CHN0							GPIO_PD0
		#define GPIO_CHN1							GPIO_PD1
		#define GPIO_CHN2							GPIO_PD6
		#define GPIO_CHN3							GPIO_PD7
		#define GPIO_CHN4							GPIO_PA2
		#define GPIO_CHN5							GPIO_PA3
		#define GPIO_CHN6							GPIO_PA4
		#define GPIO_CHN7							GPIO_PB0

		#define PD0_OUTPUT_ENABLE					1
		#define PD1_OUTPUT_ENABLE					1
		#define PD6_OUTPUT_ENABLE					1
		#define PD7_OUTPUT_ENABLE					1
		#define PA2_OUTPUT_ENABLE					1
		#define PA3_OUTPUT_ENABLE					1
		#define PA4_OUTPUT_ENABLE					1
		#define PB0_OUTPUT_ENABLE					1
	#else
		#error "add debug GPIO definition by yourself !"
	#endif
#endif  //end of DEBUG_GPIO_ENABLE









#include "../common/default_config.h"

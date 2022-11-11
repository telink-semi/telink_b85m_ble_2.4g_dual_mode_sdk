/********************************************************************************************************
 * @file	esb_ll.h
 *
 * @brief	This is the header file for 8355
 *
 * @author	2.4G Group
 * @date	2019
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
#ifndef _ESB_LL_H_
#define _ESB_LL_H_
#define NORDIC_RX_SETTLE_TIME_US                120
#define NORDIC_TX_SETTLE_TIME_US                110
#define PTX_RETRY_DELAY_TIME_US                 10

//Nordic
#define RF_NORDIC_PACKET_LENGTH_OK(p)           (p[0] == (p[12]&0x3f)+15)
#define RF_NORDIC_PACKET_CRC_OK(p)              ((p[p[0]+3] & 0x51) == 0x40)

#define TLSR_SUCCESS                            (0)
#define TLSR_ERROR_INVALID_PARAM                (-1)
#define TLSR_ERROR_BUSY                         (-2)


/**@brief Enhanced ShockBurst address width. */
typedef enum {
    ADDRESS_WIDTH_3BYTES = 3,      /**< Set address width to 3 bytes */
    ADDRESS_WIDTH_4BYTES,          /**< Set address width to 4 bytes */
    ADDRESS_WIDTH_5BYTES           /**< Set address width to 5 bytes */
} ESB_AddressWidthTypeDef;

/**@brief Enhanced ShockBurst pipe IDs. */
typedef enum {
    ESB_PIPE0 = 0,          /**< Select pipe0 */
    ESB_PIPE1,              /**< Select pipe1 */
    ESB_PIPE2,              /**< Select pipe2 */
    ESB_PIPE3,              /**< Select pipe3 */
    ESB_PIPE4,              /**< Select pipe4 */
    ESB_PIPE5,              /**< Select pipe5 */
    ESB_TX,                 /**< Refer to TX address*/
    ESB_PIPE_ALL = 0xFF     /**< Close or open all pipes*/
} ESB_PipeIDTypeDef;

/**@brief Enhanced ShockBurst state machine status. */
typedef enum {
    ESB_STATE_MACHINE_STATUS_IDLE = 0,          /**< Idle */
    ESB_STATE_MACHINE_STATUS_TX_SETTLE,         /**< TX Settle*/
    ESB_STATE_MACHINE_STATUS_TX,                /**< TX */
    ESB_STATE_MACHINE_STATUS_RX_WAIT,           /**< RX Wait */
    ESB_STATE_MACHINE_STATUS_RX,                /**< RX */
    ESB_STATE_MACHINE_STATUS_TX_WAIT,           /**< RX Wait */
} ESB_StatemachineStatusTypeDef;

/**@brief Enhanced ShockBurst mode. */
typedef enum {
    ESB_MODE_PTX = 0,      /**< PTX Mode */
    ESB_MODE_PRX,          /**< PRX Mode */
} ESB_ModeTypeDef;

/**@brief Enhanced ShockBurst bitrate mode. */
typedef enum {
    ESB_BITRATE_1MBPS = 0,      /**< 1Mbit radio mode. */
    ESB_BITRATE_2MBPS,          /**< 2Mbit radio mode. */
    ESB_BITRATE_500kBPS,        /**< 500kbit radio mode. */
    ESB_BITRATE_250KBPS,        /**< 250Kbit radio mode. */
} ESB_BitrateTypeDef;

/**@brief Enhanced ShockBurst modulation index. */
typedef enum {
	ESB_RF_MI_0000 = 0,
	ESB_RF_MI_0076 = 76,        /**< MI = 0.076 */
	ESB_RF_MI_0320 = 320,		/**< MI = 0.32 */
	ESB_RF_MI_0500 = 500,		/**< MI = 0.5 */
	ESB_RF_MI_0600 = 600,		/**< MI = 0.6 */
	ESB_RF_MI_0700 = 700,		/**< MI = 0.7 */
	ESB_RF_MI_0800 = 800,		/**< MI = 0.8 */
	ESB_RF_MI_0900 = 900,		/**< MI = 0.9 */
	ESB_RF_MI_1200 = 1200,		/**< MI = 1.2 */
	ESB_RF_MI_1300 = 1300,		/**< MI = 1.3 */
	ESB_RF_MI_1400 = 1400,		/**< MI = 1.3 */
}ESB_MIVauleTypeDef;

/**@brief Enhanced ShockBurst radio transmission power modes. */
typedef enum {
    ESB_RF_POWER_10DBM = 51,      /**< 10 dBm radio transmit power. */
    ESB_RF_POWER_9DBM  = 43,      /**< 9 dBm radio transmit power. */
    ESB_RF_POWER_8DBM  = 37,      /**< 8 dBm radio transmit power. */
    ESB_RF_POWER_7DBM  = 33,      /**< 7 dBm radio transmit power. */
    ESB_RF_POWER_6DBM  = 29,      /**< 6 dBm radio transmit power. */
    ESB_RF_POWER_5DBM  = 25,      /**< 5 dBm radio transmit power. */
    ESB_RF_POWER_4DBM  = 25,      /**< 4 dBm radio transmit power. */
    ESB_RF_POWER_3DBM  = 185,     /**< 3 dBm radio transmit power. */
    ESB_RF_POWER_2DBM  = 176,     /**< 2 dBm radio transmit power. */
    ESB_RF_POWER_1DBM  = 169,     /**< 1 dBm radio transmit power. */
    ESB_RF_POWER_0DBM  = 164,     /**< 0 dBm radio transmit power. */
    ESB_RF_POWER_M_1DBM  = 160,   /**< -1 dBm radio transmit power. */
    ESB_RF_POWER_M_2DBM  = 156,   /**< -2 dBm radio transmit power. */
    ESB_RF_POWER_M_3DBM  = 154,   /**< -3 dBm radio transmit power. */
    ESB_RF_POWER_M_4DBM  = 150,   /**< -4 dBm radio transmit power. */
    ESB_RF_POWER_M_5DBM  = 148,   /**< -5 dBm radio transmit power. */
    ESB_RF_POWER_M_6DBM  = 146,   /**< -6 dBm radio transmit power. */
    ESB_RF_POWER_M_7DBM  = 144,   /**< -7 dBm radio transmit power. */
    ESB_RF_POWER_M_8DBM  = 142,   /**< -8 dBm radio transmit power. */
    ESB_RF_POWER_M_9DBM  = 140,   /**< -9 dBm radio transmit power. */
    ESB_RF_POWER_M_11DBM  = 138,  /**< -11 dBm radio transmit power. */
    ESB_RF_POWER_M_13DBM  = 136,  /**< -13 dBm radio transmit power. */
    ESB_RF_POWER_M_15DBM  = 134,  /**< -15 dBm radio transmit power. */
    ESB_RF_POWER_M_18DBM  = 132,  /**< -18 dBm radio transmit power. */
    ESB_RF_POWER_M_24DBM  = 130,  /**< -24 dBm radio transmit power. */
    ESB_RF_POWER_M_30DBM  = 0xff, /**< -30 dBm radio transmit power. */
    ESB_RF_POWER_M_50dBm  = 128,  /**< -50 dBm radio transmit power. */
} ESB_OutputPowerTypeDef;



/**
 * @brief       Initiate the the Enhanced ShockBurst module. 
 * @note        This function must be called at the beginning of the ESB configuration.
 * @param       bitrate  Radio bitrate.
 * @return      none.
 */
extern void ESB_Init(ESB_BitrateTypeDef bitrate);

/**
 * @brief       Set the radio bitrate.
 * @param       bitrate  Radio bitrate.
 * @return      none.
 */
extern void ESB_SetBitrate(ESB_BitrateTypeDef bitrate);

/**
 * @brief       Set the channel to use for the radio. 
 * @param       channel Channel to use for the radio.
 * @return      none.
 */
extern void ESB_SetRFChannel(unsigned char channel);

/**
 * @brief       Set the new channel to use for the radio. 
 * @param       channel New channel to use for the radio.
 * @return      none.
 */
extern void ESB_SetNewRFChannel(unsigned char channel);

/**
 * @brief       Set the radio output power.
 * @param       power   Output power.
 * @return      none.
  */
extern void ESB_SetOutputPower(ESB_OutputPowerTypeDef power);

/**
 * @brief       Set one pipe as a TX pipe.
 * @param       pipe_id Pipe to be set as a TX pipe.
 * @return      none.
 */
extern void ESB_SetTXPipe(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Get the current TX pipe.
 * @return      The pipe set as a TX pipe.
*/
extern unsigned char ESB_GetTXPipe(void);

/**
 * @brief       Update the read pointer of the TX FIFO.
 * @param       pipe_id Pipe id.
 * @return      none.
 */
extern void ESB_UpdateTXFifoRptr(ESB_PipeIDTypeDef pipe_id);


/** Enables the ACK payload feature
 * @param enable Whether to enable or disable ACK payload
 */
extern void ESB_EnableAckPayload(unsigned char enable);

/**
 * @brief       Enable the W_TX_PAYLOAD_NOACK command.
 * @param       enable  Whether to enable or disable NoAck option in 9-bit Packet control field.
 * @return      none.
 */
extern void ESB_EnableNoAck(unsigned char enable);

/**
 * @brief       Write the payload that will be transmitted with ACK in the specified pipe.
 * @param       pipe_id     Pipe that transmits the payload.
 * @param       payload     Pointer to the payload data.
 * @param       length      Size of the data to transmit.
 * @return
 */
extern void ESB_WriteAckPayload(ESB_PipeIDTypeDef pipe_id, const unsigned char *payload, unsigned char length);

/**
 * @brief       Open one or all pipes.
 * @param       pipe_id Radio pipes to open.
 * @return      none.
 */
extern void ESB_OpenPipe(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Close one or all pipes.
 * @param       pipe_id Radio pipes to close.
 * @return      none.
 */
extern void ESB_ClosePipe(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Set the address for pipes.
 * @param       pipe_id Radio pipe to set the address for.
 * @param       addr    Pointer to the address data.
 * @return      none.
 */
extern void ESB_SetAddress(ESB_PipeIDTypeDef pipe_id, const unsigned char *addr);

/**
 * @brief       Get the address for pipes.
 * @param       pipe_id Radio pipe to get the address for
 * @param       addr    Pointer to a buffer that address data are written to.
 * @return      Numbers of bytes copied to addr.
 */
extern unsigned char ESB_GetAddress(ESB_PipeIDTypeDef pipe_id, unsigned char *addr);

/**
 * @brief       Set the the number of retransmission attempts and the packet retransmit delay.
 * @param       retry_times Number of retransmissions. Setting the parmater to 0 disables retransmission.
 * @param       retry_delay Delay between retransmissions.
 * @return      none.
 */
extern void ESB_SetAutoRetry(unsigned char retry_times, unsigned short retry_delay);

/**
 * @brief       Set the width of the address.
 * @param       address_width   Width of the ESB address (in bytes).
 * @return      none.
 */
extern void ESB_SetAddressWidth(ESB_AddressWidthTypeDef address_width);

/**
 * @brief       Get the width of the address.
 * @return      Width of the ESB address (in bytes).
 */
extern unsigned char ESB_GetAddressWidth(void);

/**
 * @brief       Check status for a selected pipe.
 * @param       pipe_id Pipe id to check status for.
 * @return      Pipe status.
 */
extern unsigned char ESB_GetPipeStatus(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Get the dropped packet count.
 * @return      Dropped packet count.
 */
extern unsigned char ESB_GetPacketLostCtr(void);

/* Status functions prototypes */

/**
 * @brief       Check if the TX FIFO is empty.
 * @param       pipe_id pipe id for which to check.
 * @return      1: the TX FIFO is empty; 0: the packet is not empty.
 */
extern unsigned char ESB_TxFifoEmpty(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Check if TX FIFO is full.
 * @param       pipe_id pipe id for which to check.
 * @return      TRUE TX FIFO empty or not.
 */
extern unsigned char ESB_TxFifoFull(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Get the number of retransmission attempts.
 * @return      Number of retransmissions.
 */
extern unsigned char ESB_GetTransmitAttempts(void);

/**
 * @brief       Get the carrier detect status.
 * @return      Carrier detect status.
 */
extern unsigned char ESB_GetCarrierDetect(void);

/**
 * @brief       Get the pipe that has received a packet.
 * @return      Pipe id.
 */
extern unsigned char ESB_GetRxDataSource(void);

/**
 * @brief       Read an RX payload.
 * @param       rx_pload   Pointer to the buffer where the payload will be stored.
 * @return      pipe number (MSB) and packet length (LSB).
 */
extern unsigned short ESB_ReadRxPayload(unsigned char *rx_pload);

/**
 * @brief       Get the RX timestamp. 
 * @note        It is required to call ESB_ReadRxPayload() before this function is called.
 * @return      RX timestamp.
 */
extern unsigned int ESB_GetTimestamp(void);

/**
 * @brief       Get the RX RSSI value. 
 * @note        It is required to call ESB_ReadRxPayload() before this function is called.
 * @return      RSSI value.
 */
extern signed int ESB_GetRxRssiValue(void);

/**
 * @brief       Write a payload for transmission.
 * @param       pipe_id     Pipe id used for this payload.
 * @param       tx_pload    Pointer to the buffer that contains the payload data.
 * @param       length      Length of the payload.
 * @retval      0           Error
 * @retval      >0          The length of written bytes
 */
extern unsigned char ESB_WriteTxPayload(ESB_PipeIDTypeDef pipe_id, const unsigned char *tx_pload, unsigned char length);

/**
 * @brief       Reuse the last transmitted payload for the next packet.
 * @param       pipe_id pipe id.
 * @return      none.
 */
extern void ESB_ReuseTx(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Remove remaining items from the RX buffer.
 * @return      none.
 */
extern void ESB_FlushRx(void);

/**
 * @brief       Remove remaining items from the TX buffer.
 * @param       pipe_id Pipe id.
 * @return      none.
 */
extern void ESB_FlushTx(ESB_PipeIDTypeDef pipe_id);

/**
 * @brief       Trigger transmission in the specified pipe.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_BUSY             If the function failed because the radio was busy.
 */
extern int ESB_PTXTrig(void);

/**
 * @brief       Trigger reception in the specified pipe.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_BUSY             If the function failed because the radio was busy.
 */
extern int ESB_PRXTrig(void);

/**
 * @brief       Set the RX wait time.
 * @param       wait_us     Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int ESB_RxWaitSet(unsigned short wait_us);

/**
 * @brief       Set the TX wait time.
 * @param       wait_us     Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int ESB_TxWaitSet(unsigned short wait_us);

/**
 * @brief       Set the RX timerout time.
 * @param       period_us   Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int ESB_RxTimeoutSet(unsigned short period_us);

/**
 * @brief       Set the TX settle time.
 * @param       period_us   Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int ESB_TxSettleSet(unsigned short period_us);

/**
 * @brief       Set the RX settle time.
 * @param       period_us   Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int ESB_RxSettleSet(unsigned short period_us);

/**
 * @brief       Set the mode of ESB radio.
 * @param       mode    ESB_MODE_PTX or ESB_MODE_PRX.
 * @return      none.
 */
extern void ESB_ModeSet(ESB_ModeTypeDef mode);

/**
 * @brief       Stop the ESB state machine.
 * @return      none.
 */
extern void ESB_ModeStop(void);

/**
 * @brief       Check whether the received packet is valid.
 * @return      1: the packet is valid; 0: the packet is invalid.
 */
extern unsigned char ESB_IsRxPacketValid(void);

/**
 * @brief       Set the frequency deviation of the transmitter, which follows the equation below.
 *              frequency deviation = bitrate/(modulation index)^2
 * @param       mi_value    Modulation index.
 * @return      none.
 */
extern void ESB_SetTxMI(ESB_MIVauleTypeDef mi_value);

/**
 * @brief       Set the frequency deviation of the receiver, which follows the equation below.
 *              frequency deviation = bitrate/(modulation index)^2
 * @param       mi_value    Modulation index.
 * @return      none.
 */
extern void  ESB_SetRxMI(ESB_MIVauleTypeDef mi_value);

/**
 * @brief      Set the length of the preamble field of the on-air data packet.
 * @note       The valid range of this parameter is 1-16.
 * @param      preamble_len Preamble length.
 * @return     none.
 */
extern void ESB_Preamble_Set(unsigned char preamble_len);

/**
 * @brief      Read the length of the preamble field of the on-air data packet.
 * @return     Preamble length.
 */
extern unsigned char ESB_Preamble_Read(void);

/**
 * @brief      disable the receiver preamble detection banking duiring the first byte of pdu.
 * @param      none.
 * @return     none.
 */
extern void ESB_Preamble_Detect_Disable(void);
#endif /*_ESB_LL_H_*/

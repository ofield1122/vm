/******************************************************************************
 * @file     uart_transfer.h
 * @brief    General UART ISP slave header file
 * @version  1.0.0
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE        	64

/*-------------------------------------------------------------*/

extern uint8_t uart_rcvbuf[];
extern uint8_t volatile bUartDataReady;
extern uint8_t volatile bufhead;

/*-------------------------------------------------------------*/
void UART_Init(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void PutString(void);
uint32_t UART_IS_CONNECT(void);

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

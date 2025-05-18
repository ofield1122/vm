/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC131.h"
#include "main.h"

#define PLL_CLOCK   50000000

#define RXBUFSIZE   1024
//test

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

#define UART3_HEADER 0xAA
#define UART3_MAX_DATA_LEN 64

typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t len;
    uint8_t data[UART3_MAX_DATA_LEN];
    uint8_t crc;
} UART3_Packet;

uint8_t calc_crc(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= buf[i]; // simple XOR checksum
    }
    return crc;
}

typedef enum {
    UART3_WAIT_HEADER,
    UART3_WAIT_ID,
    UART3_WAIT_LEN,
    UART3_WAIT_DATA,
    UART3_WAIT_CRC
} UART3_RX_State;

volatile UART3_RX_State uart3_rx_state = UART3_WAIT_HEADER;
volatile UART3_Packet uart3_rx_packet;
volatile uint8_t uart3_rx_data_index = 0;

void uart3_send_fast(uint8_t id, uint8_t *data, uint8_t len)
{
    uint8_t buf[UART3_MAX_DATA_LEN + 4]; // header + id + len + data + crc
    uint32_t total_len = 0;

    buf[total_len++] = UART3_HEADER;
    buf[total_len++] = id;
    buf[total_len++] = len;

    memcpy(&buf[total_len], data, len);
    total_len += len;

    buf[total_len++] = calc_crc(&buf[1], len + 2); // id+len+data

    for (uint32_t i = 0; i < total_len; i++)
    {
        while (UART_IS_TX_FULL(UART3));  // Wait until there is room
        UART_WRITE(UART3, buf[i]);
    }
}


void uart3_process_packet(UART3_Packet *packet)
{
    printf("Received Packet: ID=0x%02X, LEN=%d\n", packet->id, packet->len);

    // Example: switch by ID
    switch (packet->id)
    {
        case 0x01:
            // handle command 1
            break;
        case 0x02:
            // handle command 2
            break;
        default:
            printf("Unknown Packet ID: 0x%02X\n", packet->id);
            break;
    }
}


void UART3_IRQHandler(void)
{
    if (UART_GET_INT_FLAG(UART3, UART_ISR_RDA_IF_Msk))
    {
        uint8_t byte = UART_READ(UART3);

        switch (uart3_rx_state)
        {
            case UART3_WAIT_HEADER:
                if (byte == UART3_HEADER)
                {
                    uart3_rx_packet.header = byte;
                    uart3_rx_state = UART3_WAIT_ID;
                }
                break;

            case UART3_WAIT_ID:
                uart3_rx_packet.id = byte;
                uart3_rx_state = UART3_WAIT_LEN;
                break;

            case UART3_WAIT_LEN:
                uart3_rx_packet.len = byte;
                uart3_rx_data_index = 0;
                if (uart3_rx_packet.len > UART3_MAX_DATA_LEN)
                {
                    uart3_rx_state = UART3_WAIT_HEADER; // error recovery
                }
                else
                {
                    uart3_rx_state = UART3_WAIT_DATA;
                }
                break;

            case UART3_WAIT_DATA:
                uart3_rx_packet.data[uart3_rx_data_index++] = byte;
                if (uart3_rx_data_index >= uart3_rx_packet.len)
                {
                    uart3_rx_state = UART3_WAIT_CRC;
                }
                break;

            case UART3_WAIT_CRC:
                uart3_rx_packet.crc = byte;
                uint8_t crc = calc_crc((uint8_t *)&uart3_rx_packet.id, 1 + 1 + uart3_rx_packet.len); // id + len + data
                if (crc == uart3_rx_packet.crc)
                {
                    // ✅ valid packet received
                    uart3_process_packet((UART3_Packet *)&uart3_rx_packet);
                }
                else
                {
                    printf("UART3 CRC error!\n");
                }
                uart3_rx_state = UART3_WAIT_HEADER; // ready for next packet
                break;
        }
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1)); 
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Enable UART3 module clock ,connect to CPU*/
    CLK_EnableModuleClock(UART3_MODULE);

    /* Select UART3 module clock source */
    CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    /* Set GPA multi-function pins for UART3 RXD(PA.5) and TXD(PA.6) */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA5_Msk | SYS_GPA_MFP_PA6_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA5_UART3_RXD | SYS_GPA_MFP_PA6_UART3_TXD;


}
 #if 0
/**
 * @brief       GPIO PA/PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA/PB default IRQ, declared in startup_NUC1311.s.
 */
void GPAB_IRQHandler(void)
{
    /* To check if PB.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PA and PB interrupts */
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
        printf("Un-expected interrupts.\n");
    }
}
/**
 * @brief       GPIO PC/PD/PF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PC/PD/PF default IRQ, declared in startup_NUC1311.s.
 */

 
void GPCDEF_IRQHandler(void)
{
    /* To check if PE.5 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PF, BIT5))
    {
        GPIO_CLR_INT_FLAG(PF, BIT5);
        printf("PF.5 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC, PD and PF interrupts */
        PC->ISRC = PC->ISRC;
        PD->ISRC = PD->ISRC;
        PF->ISRC = PF->ISRC;
        printf("Un-expected interrupts.\n");
    }
}

#else

	
typedef void (*InterruptHandler)(void); // define a function pointer type

typedef struct {
    GPIO_T *port;
    uint32_t pin;
    const char *name;
    InterruptHandler handler; // add a function pointer for custom action
} InterruptPin;

void COIN0_proces(void)
{
    printf("Button 1 pressed (PF3)\n");
}
void BLOCK0_proces(void)
{
    printf("Button 2 pressed (PF5)\n");
}
void PRIZE0_proces(void)
{
    printf("Button 2 pressed (PF5)\n");
}

void COIN1_proces(void)
{
    printf("Button 1 pressed (PF3)\n");
}
void BLOCK1_proces(void)
{
    printf("Button 2 pressed (PF5)\n");
}
void PRIZE1_proces(void)
{
    printf("Button 2 pressed (PF5)\n");
}

void COIN2_proces(void)
{
    printf("Button 1 pressed (PF3)\n");
}
void BLOCK2_proces(void)
{
    printf("Button 2 pressed (PF5)\n");
}
void PRIZE2_proces(void)
{
    printf("Button 2 pressed (PF5)\n");
}

InterruptPin interruptPins[] = {
	{ PA, BIT2, "Button1_PA3", COIN0_proces },
	{ PA, BIT3, "Button1_PA3", BLOCK0_proces },
	{ PB, BIT3, "Button1_PA3", PRIZE0_proces },	
    { PF, BIT3, "Button1_PF3", COIN1_proces },
    { PF, BIT5, "Button2_PF5", BLOCK1_proces },
    { PD, BIT4, "Sensor_PD4", PRIZE1_proces },
    { PC, BIT5, "Input_PC5", COIN2_proces },
    { PC, BIT2, "Input_PC1", BLOCK2_proces },
	{ PC, BIT1, "Input_PC1", PRIZE2_proces },
};

#define NUM_INTERRUPT_PINS (sizeof(interruptPins) / sizeof(interruptPins[0]))

void InitInterruptPins(void)
{
    for (int i = 0; i < NUM_INTERRUPT_PINS; i++)
    {
        GPIO_SetMode(interruptPins[i].port, interruptPins[i].pin, GPIO_PMD_INPUT);
        GPIO_EnableInt(interruptPins[i].port, interruptPins[i].pin, GPIO_INT_RISING);
    }
	
	NVIC_EnableIRQ(GPAB_IRQn);
    NVIC_EnableIRQ(GPCDEF_IRQn);
}

void GPAB_IRQHandler(void)
{
   int handled = 0;

    for (int i = 0; i < NUM_INTERRUPT_PINS; i++)
    {
        if (GPIO_GET_INT_FLAG(interruptPins[i].port, interruptPins[i].pin))
        {
            GPIO_CLR_INT_FLAG(interruptPins[i].port, interruptPins[i].pin);
            printf("[%s] interrupt occurred!\n", interruptPins[i].name);
			handled = 1;
            
            if (interruptPins[i].handler)
            {
                interruptPins[i].handler(); // call the specific handler
            }
        }
    }
	if (!handled)
    {
		/* Un-expected interrupt. Just clear all PA and PB interrupts */
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
        printf("Un-expected interrupts.\n");
	}
}

void GPCDEF_IRQHandler(void)
{
    int handled = 0;

    for (int i = 0; i < NUM_INTERRUPT_PINS; i++)
    {
        if (GPIO_GET_INT_FLAG(interruptPins[i].port, interruptPins[i].pin))
        {
            GPIO_CLR_INT_FLAG(interruptPins[i].port, interruptPins[i].pin);
            printf("[%s] interrupt occurred!\n", interruptPins[i].name);
            handled = 1;
            if (interruptPins[i].handler)
            {
                interruptPins[i].handler(); // call the specific handler
            }
        }
    }
	if (!handled)
    {
		// Clear unexpected interrupts
		PC->ISRC = PC->ISRC;
		PD->ISRC = PD->ISRC;
		PF->ISRC = PF->ISRC;
	}
}


#endif



void UART3_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART3_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART3, 115200);
}
void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}
/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/




int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf and testing */
    UART0_Init();
	UART3_Init();
    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %luHz\n", SystemCoreClock);

    printf("\n\nch UART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

/*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("PA.9 and PF.5 are used to test interrupt ......\n");

    /* Configure CIN0(PA.9) as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT9, GPIO_PMD_INPUT);
    GPIO_EnableInt(PA, 9, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPAB_IRQn);

    /* Configure BLK1(PF.5) as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PF, BIT5, GPIO_PMD_INPUT);
    GPIO_EnableInt(PF, 5, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPCDEF_IRQn);



    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART0->ISR;

    /* Receive Data Available Interrupt Handle */
    if(u32IntSts & UART_ISR_RDA_INT_Msk)
    {
        printf("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Receive Line Status Error Handle */
            if(u32IntSts & UART_ISR_RLS_INT_Msk)
            {                
                /* Clear Receive Line Status Interrupt */
                UART_ClearIntFlag(UART0, UART_ISR_RLS_INT_Msk);   
            }

            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }
        printf("\nTransmission Test:");
    }

    /* Transmit Holding Register Empty Interrupt Handle */
    if(u32IntSts & UART_ISR_THRE_INT_Msk)
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            while(UART_IS_TX_FULL(UART0));  /* Wait Tx is not full to transmit data */
            UART_WRITE(UART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }

    /* Buffer Error Interrupt Handle */    
    if(u32IntSts & UART_ISR_BUF_ERR_INT_Msk)   
    {
        /* Clear Buffer Error Interrupt */
        UART_ClearIntFlag(UART0, UART_ISR_BUF_ERR_INT_Msk);             
    }       

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC.
        UART0 is set to debug port. UART0 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART0 will print the received char on screen.
    */

    /* Enable UART RDA, THRE, RLS and Buffer Error interrupt */
    UART_EnableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_BUF_ERR_IEN_Msk));
    NVIC_EnableIRQ(UART02_IRQn);
    while(g_bWait);

    /* Disable UART RDA, THRE, RLS and Buffer Error interrupt */
    UART_DisableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RLS_IEN_Msk | UART_IER_BUF_ERR_IEN_Msk));
    NVIC_DisableIRQ(UART02_IRQn);
    g_bWait = TRUE;
    printf("\nUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"


#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       1

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


void PowerDownFunction(void);
void GPAB_IRQHandler(void);
void LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled and Power-down entry condition */
    CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();
}

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

void LvrSetting(void)
{
    if(SET_LVR == 0)
    {
        /* Disable LVR */
        SYS_DISABLE_LVR();
        CLK_SysTickDelay(200);
    }
    else
    {
        /* Enable LVR */
        SYS_ENABLE_LVR();
        CLK_SysTickDelay(200);
    }
}

void PorSetting(void)
{
    if(SET_POR == 0)
    {
        /* Disable POR */
        SYS_DISABLE_POR();
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK->PWRCON &= ~CLK_PWRCON_OSC10K_EN_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->CLKSTATUS & CLK_CLKSTATUS_IRC10K_STB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK->PWRCON |= CLK_PWRCON_OSC10K_EN_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (CLK->CLKSTATUS & CLK_CLKSTATUS_IRC10K_STB_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

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

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.3 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Enter to Power-Down                                           |\n");
    printf("|  6. Wait for PB.3 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /*
        To measure Power-down current:
        On NuMaker-NUC131U V1.0 board, remove components, e.g. Nu-Link2-Me and R7.
        Remove R16 and then user can measure target chip power consumption by AMMETER connector.
    */

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFP = 0;
    SYS->GPB_MFP = SYS_GPB_MFP_PB1_UART0_TXD;
    SYS->GPC_MFP = 0;
    SYS->GPD_MFP = 0;
    SYS->GPE_MFP = 0;
    SYS->GPF_MFP = 0;
    SYS->ALT_MFP = 0;
    SYS->ALT_MFP2 = 0;
    SYS->ALT_MFP3 = 0;
    SYS->ALT_MFP4 = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode. They are default output high. */
    PA->PMD = 0xFFFFFFFF;
    PB->PMD = 0xFFFFFFFF;
    PC->PMD = 0xFFFFFFFF;
    PD->PMD = 0xFFFFFFFF;
    PE->PMD = 0xFFFFFFFF;
    PF->PMD = 0xFFFFFFFF;

    /* Unlock protected registers for Power-down setting */
    SYS_UnlockReg();

    /* LVR setting */
    LvrSetting();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if( LircSetting() < 0 ) goto lexit;

    /* Wake-up source configuration */
    /* Configure PB.3 as Quasi mode and enable interrupt by falling edge trigger */
    PB->PMD = (PB->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_QUASI << GPIO_PMD_PMD3_Pos);
    PB->IMD |= (GPIO_IMD_EDGE << 3);
    PB->IEN |= (BIT3 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPAB_IRQn);

    /* Enter to Power-down mode */
    printf("Enter to Power-Down ......\n");
    PowerDownFunction();

    /* Waiting for PB.3 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while(1);
}

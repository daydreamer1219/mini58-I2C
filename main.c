/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 15/05/28 10:37a $
 * @brief    Read/write EEPROM via I2C interface using FIFO mode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Mini58Series.h"


uint8_t __IO g_u8EndFlag = 0;
uint8_t data;
typedef void (*I2C_FUNC)(uint32_t u32Status);

I2C_FUNC __IO s_I2CHandlerFn = NULL;

void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    //printf("RX:%d\n" , u32Status);
    if (u32Status == 0x60)                      /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_SI|I2C_AA);
    }
    else if (u32Status == 0x80)                 /* SLA+W has been transmitted and ACK has been received */
    {
				data = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI|I2C_AA);
    }
    else if (u32Status == 0xA0)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_AA | I2C_SI);
    }
    else if (u32Status == 0xA8)                 /* DATA has been transmitted and ACK has been received */
    {
					I2C_SET_DATA(I2C0,0x21);
            I2C_SET_CONTROL_REG(I2C0, I2C_AA | I2C_SI);
        
    }
    else if (u32Status == 0xC0)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
			 g_u8EndFlag = 1;
        P07 = 1;
    }
   
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_XTAL,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_I2C0_SDA | SYS_MFP_P35_I2C0_SCL;

    /* Set P1 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);

    /* I2C pin enable schmitt trigger */
    SYS->P3_MFP |= SYS_MFP_TYPE_Msk(4) | SYS_MFP_TYPE_Msk(5);

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}



uint8_t data1,status,flag=1;
void EEPROM_Read(void)
{
                     
   I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
	I2C_WAIT_READY(I2C); 
	while(flag){
	status=I2C_GET_STATUS(I2C);
		if(status==0x60){
			data1=I2C_GET_DATA(I2C);
			I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
			I2C_WAIT_READY(I2C); 
				status=I2C_GET_STATUS(I2C);
			if(status==0x80){
			data1=I2C_GET_DATA(I2C);
				status=I2C_GET_STATUS(I2C);
			}
		I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
				status=I2C_GET_STATUS(I2C);
			I2C_WAIT_READY(I2C); 
		if(status==0xA0){
		data1=I2C_GET_DATA(I2C);
			I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
			status=I2C_GET_STATUS(I2C);
			I2C_WAIT_READY(I2C); 
				data1=I2C_GET_DATA(I2C);
		
		}
		if(status==0xA8){
		I2C_SET_CONTROL_REG(I2C, I2C_SI|I2C_AA) ;
			status=I2C_GET_STATUS(I2C);
			I2C_WAIT_READY(I2C); 
			data1=I2C_GET_DATA(I2C);

		}
		
		if(status==0xC0){
		I2C_SET_CONTROL_REG(I2C, I2C_SI|I2C_AA);
			status=I2C_GET_STATUS(I2C);
			I2C_WAIT_READY(I2C); 
			data1=I2C_GET_DATA(I2C);
			flag=0;
		}
		}
		}
	}
      
	

int main(void)
{
		uint32_t tout;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 baud rate */
	SYS_ResetModule(UART0_RST);
    UART_Open(UART0, 115200);

    /* Setup write buffer */
    /* Open I2C and set to 100k */
    I2C_Open(I2C0, 50000);
		I2C_SetSlaveAddr(I2C, 0, 0x15, 0);
	 I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
     s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;
		CLK_SysTickDelay(5000);
		I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
	 tout = SystemCoreClock;
        while ((g_u8EndFlag == 0) && (tout-- > 0));
        if (g_u8EndFlag == 0)
        {
            printf("Wait RX interrupt flag time-out!\n");
            while (1);
        }
        tout = SystemCoreClock;
        while ((I2C0->CTL & I2C_CTL_STO_Msk) && (tout-- > 0));
        if (I2C0->CTL & I2C_CTL_STO_Msk)
        {
            printf("RX wait I2C_CTL_STO_Msk cleared time-out!\n");
            while (1);
        }
}

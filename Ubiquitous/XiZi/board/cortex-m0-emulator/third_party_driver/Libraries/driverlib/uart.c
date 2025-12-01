//*****************************************************************************
//
// uart.c - Driver for the UART.
//
// Copyright (c) 2005-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 10636 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup uart_api
//! @{
//
//*****************************************************************************
#include <stdarg.h>

#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/opt.h"

//*****************************************************************************
//
// This is a mapping between interrupt number (for the peripheral interrupts
// only) and the register that contains the interrupt enable for that
// interrupt.
//
//*****************************************************************************
static const unsigned long g_pulEnRegs[] =
{
    NVIC_EN0, NVIC_EN1, NVIC_EN2, NVIC_EN3, NVIC_EN4
};

void UARTlInitialize(void) {
    GPIO_DIRSET = BIT(USB_TX);
    GPIO_DIRCLR = BIT(USB_RX);
    SET_FIELD(GPIO_PINCNF[USB_TX], GPIO_PINCNF_PULL, GPIO_Pullup);
    SET_FIELD(GPIO_PINCNF[USB_RX], GPIO_PINCNF_PULL, GPIO_Pullup);

    UART_BAUDRATE = UART_BAUD_9600;     // 9600 baud
    UART_CONFIG = 0;                    // format 8N1
    UART_PSELTXD = USB_TX;              // choose pins
    UART_PSELRXD = USB_RX;
    /* Configure the uart device interrupt */
    UART_ENABLE = UART_Enabled;
    UART_INTENSET = (1 << 17) | (1<<2);
    UART_STARTTX = 1;
    UART_STARTRX = 1;
    UART_RXDRDY = 0;
    UART_TXDRDY = 1;
    /* Enable the uart interrupt in NVIC */
    HWREG(g_pulEnRegs[(18 - 16) / 32]) = 1 << ((18 - 16) & 31);
}

void UARTConfigSetExpClk(unsigned long ulBase, unsigned long ulUARTClk,
                    unsigned long ulBaud, unsigned long ulConfig)
{
    NO_IMPL();
}

tBoolean UARTCharsAvail(unsigned long ulBase)
{
    return UART_RXDRDY;
}

long UARTCharGetNonBlocking(unsigned long ulBase)
{
    char ch;

    if (UART_RXDRDY) {
        UART_RXDRDY = 0;
        ch = UART_RXD;
        return ch;
    }
    return -1;
}

long UARTCharGet(unsigned long ulBase)
{
    char ch;

    while (!UART_RXDRDY);
    UART_RXDRDY = 0;
    ch = UART_RXD;
    return ch;
}

tBoolean UARTCharPutNonBlocking(unsigned long ulBase, unsigned char ucData)
{
    if (UART_TXDRDY) {
        UART_TXDRDY = 0;
        UART_TXD = ucData;
        return true;
    }
    return false;
}

void UARTCharPut(unsigned long ulBase, unsigned char ucData)
{
    while (!UART_TXDRDY);
    UART_TXDRDY = 0;
    UART_TXD = ucData;
}

void UARTIntEnable(unsigned long ulBase, unsigned long ulIntFlags)
{
    NO_IMPL();
}

void UARTIntDisable(unsigned long ulBase, unsigned long ulIntFlags)
{
    NO_IMPL();
}

unsigned long UARTIntStatus(unsigned long ulBase, tBoolean bMasked)
{
    return UART_INTENCLR;
}

void UARTIntClear(unsigned long ulBase, unsigned long ulIntFlags)
{
    UART_INTENCLR = ulIntFlags;
    UART_INTENSET = ulIntFlags;
}

#ifndef __UART_H__
#define __UART_H__

#include <xs_base.h>

#define NO_IMPL() KPrintf("%s is not implemented!\n", __func__)

/* Device pins */
#define USB_TX 24
#define USB_RX 25

/* UART */
#define UART_STARTRX   ADDR(0x40002000)
#define UART_STARTTX   ADDR(0x40002008)
#define UART_RXDRDY    ADDR(0x40002108)
#define UART_TXDRDY    ADDR(0x4000211c)
#define UART_ENABLE    ADDR(0x40002500)
#define UART_PSELTXD   ADDR(0x4000250c)
#define UART_PSELRXD   ADDR(0x40002514)
#define UART_RXD       ADDR(0x40002518)
#define UART_TXD       ADDR(0x4000251c)
#define UART_BAUDRATE  ADDR(0x40002524)
#define UART_CONFIG    ADDR(0x4000256c)
#define UART_INTEN     ADDR(0x40002300)
#define UART_INTENSET  ADDR(0x40002304)
#define UART_INTENCLR  ADDR(0x40002308)

#define UART_Enabled 0x4
#define UART_INT_TXDRDY 7
#define UART_INT_RXDRDY 2

#define UART_BAUD_9600 0x00275000

#ifdef __cplusplus
extern "C"
{
#endif

extern void UARTlInitialize(void);
extern void UARTConfigSetExpClk(unsigned long ulBase, unsigned long ulUARTClk,
                                unsigned long ulBaud, unsigned long ulConfig);
extern tBoolean UARTCharsAvail(unsigned long ulBase);
extern long UARTCharGetNonBlocking(unsigned long ulBase);
long UARTCharGet(unsigned long ulBase);
extern tBoolean UARTCharPutNonBlocking(unsigned long ulBase,
                                       unsigned char ucData);
extern void UARTCharPut(unsigned long ulBase, unsigned char ucData);
extern void UARTIntEnable(unsigned long ulBase, unsigned long ulIntFlags);
extern void UARTIntDisable(unsigned long ulBase, unsigned long ulIntFlags);
extern unsigned long UARTIntStatus(unsigned long ulBase, tBoolean bMasked);
extern void UARTIntClear(unsigned long ulBase, unsigned long ulIntFlags);

#ifndef DEPRECATED

#define UARTCharNonBlockingGet(a) \
        UARTCharGetNonBlocking(a)
#define UARTCharNonBlockingPut(a, b) \
        UARTCharPutNonBlocking(a, b)
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __UART_H__
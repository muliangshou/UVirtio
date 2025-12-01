/*
 * Copyright (c) 2020 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 * 2018-12-11     greedyhao    Porting for stm32f7xx
 * 2019-01-03     zylx         modify DMA initialization and spixfer function
 * 2020-01-15     whj4674672   Porting for stm32h7xx
 * 2020-06-18     thread-liu   Porting for stm32mp1xx
 * 2020-10-14     Dozingfiretruck   Porting for stm32wbxx
 */

/**
* @file connect_spi.c
* @brief support ok1052-c board spi function and register to bus framework
* @version 1.0
* @author AIIT XUOS Lab
* @date 2022-03-01
*/

/*************************************************
File name: connect_spi.c
Description: support ok1052-c board spi configure and spi bus register function
Others: take RT-Thread v4.0.2/bsp/stm32/libraries/HAL_Drivers/drv_spi.c for references
                https://github.com/RT-Thread/rt-thread/tree/v4.0.2
History:
1. Date: 2022-03-01
Author: AIIT XUOS Lab
Modification:
1. support ok1052-c board spi configure, write and read
2. support ok1052-c board spi bus device and driver register
3. add ok1052-c board spi test letter command
*************************************************/

#include "board.h"
#include "connect_spi.h"
#include "fsl_lpspi.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SPI_PCS_FOR_INIT (kLPSPI_Pcs0)

/* Select USB1 PLL PFD0 (720 MHz) as lpspi clock source */
#define SPI_CLOCK_SOURCE_SELECT (1U)

/* Clock divider for master lpspi clock source */
#define SPI_CLOCK_SOURCE_DIVIDER (7U)

#define SPI_CLOCK_FREQ (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (SPI_CLOCK_SOURCE_DIVIDER + 1U))

#define TRANSFER_SIZE (512U)        /*! Transfer dataSize .*/
#define TRANSFER_BAUDRATE (500000U) /*! Transfer baudrate - 500k */

#define spi_print KPrintf

typedef struct __spi_transfer_t
{
    uint32_t size;
    volatile uint32_t cnt;
    uint8_t water;
    uint8_t fifo;
    volatile bool completed;
    uint8_t buf[TRANSFER_SIZE];
}spi_transfer_t;

#if defined(BSP_USING_SPI1)
struct Stm32Spi spi1;
spi_transfer_t spi1_rx_data;
spi_transfer_t spi1_tx_data;
#endif

#if defined(BSP_USING_SPI2)
struct Stm32Spi spi2;
spi_transfer_t spi2_rx_data;
spi_transfer_t spi2_tx_data;
#endif

#if defined(BSP_USING_SPI3)
struct Stm32Spi spi3;
spi_transfer_t spi3_rx_data;
spi_transfer_t spi3_tx_data;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void LPSPI1_IRQHandler(int vector, void *param)
{
    LPSPI_Type *spi_base = LPSPI1;
    int rx_size = spi1_rx_data.size < TRANSFER_SIZE ? spi1_rx_data.size : TRANSFER_SIZE;
    int tx_size = spi1_tx_data.size < TRANSFER_SIZE ? spi1_tx_data.size : TRANSFER_SIZE;

    if (spi1_rx_data.cnt < rx_size)
    {
        /* First, disable the interrupts to avoid potentially triggering another interrupt
         * while reading out the RX FIFO as more data may be coming into the RX FIFO. We'll
         * re-enable the interrupts on the LPSPI state after reading out the FIFO.
         */
        LPSPI_DisableInterrupts(spi_base, kLPSPI_RxInterruptEnable);

        while (LPSPI_GetRxFifoCount(spi_base))
        {
            /*Read out the data*/
            spi1_rx_data.buf[spi1_rx_data.cnt] = LPSPI_ReadData(spi_base);
            spi1_rx_data.cnt++;

            if (spi1_rx_data.cnt == rx_size)
            {
                break;
            }
        }

        /* Re-enable the interrupts only if rxCount indicates there is more data to receive,
         * else we may get a spurious interrupt.
         * */
        if (spi1_rx_data.cnt < rx_size)
        {
            /* Set the TDF and RDF interrupt enables simultaneously to avoid race conditions */
            LPSPI_EnableInterrupts(spi_base, kLPSPI_RxInterruptEnable);
        }
    }

    /*Update rxWatermark. There isn't RX interrupt for the last datas if the RX count is not greater than rxWatermark.*/
    if ((rx_size - spi1_rx_data.cnt) <= spi1_rx_data.water)
    {
        spi_base->FCR =
            (spi_base->FCR & (~LPSPI_FCR_RXWATER_MASK)) |
            LPSPI_FCR_RXWATER(((rx_size - spi1_rx_data.cnt) > 1) ? ((rx_size - spi1_rx_data.cnt) - 1U) : (0U));
    }

    if (spi1_tx_data.cnt < tx_size)
    {
        while ((LPSPI_GetTxFifoCount(spi_base) < spi1_tx_data.fifo) &&
               (spi1_tx_data.cnt - spi1_rx_data.cnt < spi1_tx_data.fifo))
        {
            /*Write the word to TX register*/
            LPSPI_WriteData(spi_base, spi1_tx_data.buf[spi1_tx_data.cnt]);
            spi1_tx_data.cnt++;

            if (spi1_tx_data.cnt == tx_size)
            {
                spi1_tx_data.completed = true;
                /* Complete the transfer and disable the interrupts */
                LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);
                break;
            }
        }
    }

    /* Check if we're done with this transfer.*/
    if ((spi1_tx_data.cnt == tx_size) && (spi1_rx_data.cnt == tx_size))
    {
        spi1_tx_data.completed = true;
        /* Complete the transfer and disable the interrupts */
        LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

DECLARE_HW_IRQ(LPSPI1_IRQn, LPSPI1_IRQHandler, NONE);

void LPSPI2_IRQHandler(int vector, void *param)
{
    LPSPI_Type *spi_base = LPSPI2;
    int rx_size = spi2_rx_data.size < TRANSFER_SIZE ? spi2_rx_data.size : TRANSFER_SIZE;
    int tx_size = spi2_tx_data.size < TRANSFER_SIZE ? spi2_tx_data.size : TRANSFER_SIZE;

    if (spi2_rx_data.cnt < rx_size)
    {
        /* First, disable the interrupts to avoid potentially triggering another interrupt
         * while reading out the RX FIFO as more data may be coming into the RX FIFO. We'll
         * re-enable the interrupts on the LPSPI state after reading out the FIFO.
         */
        LPSPI_DisableInterrupts(spi_base, kLPSPI_RxInterruptEnable);

        while (LPSPI_GetRxFifoCount(spi_base))
        {
            /*Read out the data*/
            spi2_rx_data.buf[spi2_rx_data.cnt] = LPSPI_ReadData(spi_base);
            spi2_rx_data.cnt++;

            if (spi2_rx_data.cnt == rx_size)
            {
                break;
            }
        }

        /* Re-enable the interrupts only if rxCount indicates there is more data to receive,
         * else we may get a spurious interrupt.
         * */
        if (spi2_rx_data.cnt < rx_size)
        {
            /* Set the TDF and RDF interrupt enables simultaneously to avoid race conditions */
            LPSPI_EnableInterrupts(spi_base, kLPSPI_RxInterruptEnable);
        }
    }

    /*Update rxWatermark. There isn't RX interrupt for the last datas if the RX count is not greater than rxWatermark.*/
    if ((rx_size - spi2_rx_data.cnt) <= spi2_rx_data.water)
    {
        spi_base->FCR =
            (spi_base->FCR & (~LPSPI_FCR_RXWATER_MASK)) |
            LPSPI_FCR_RXWATER(((rx_size - spi2_rx_data.cnt) > 1) ? ((rx_size - spi2_rx_data.cnt) - 1U) : (0U));
    }

    if (spi2_tx_data.cnt < tx_size)
    {
        while ((LPSPI_GetTxFifoCount(spi_base) < spi2_tx_data.fifo) &&
               (spi2_tx_data.cnt - spi2_rx_data.cnt < spi2_tx_data.fifo))
        {
            /*Write the word to TX register*/
            LPSPI_WriteData(spi_base, spi2_tx_data.buf[spi2_tx_data.cnt]);
            spi2_tx_data.cnt++;

            if (spi2_tx_data.cnt == tx_size)
            {
                spi2_tx_data.completed = true;
                /* Complete the transfer and disable the interrupts */
                LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);
                break;
            }
        }
    }

    /* Check if we're done with this transfer.*/
    if ((spi2_tx_data.cnt == tx_size) && (spi2_rx_data.cnt == tx_size))
    {
        spi2_tx_data.completed = true;
        /* Complete the transfer and disable the interrupts */
        LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

DECLARE_HW_IRQ(LPSPI2_IRQn, LPSPI2_IRQHandler, NONE);

void LPSPI3_IRQHandler(int vector, void *param)
{
    LPSPI_Type *spi_base = LPSPI3;
    int rx_size = spi3_rx_data.size < TRANSFER_SIZE ? spi3_rx_data.size : TRANSFER_SIZE;
    int tx_size = spi3_tx_data.size < TRANSFER_SIZE ? spi3_tx_data.size : TRANSFER_SIZE;

    if (spi3_rx_data.cnt < rx_size)
    {
        /* First, disable the interrupts to avoid potentially triggering another interrupt
         * while reading out the RX FIFO as more data may be coming into the RX FIFO. We'll
         * re-enable the interrupts on the LPSPI state after reading out the FIFO.
         */
        LPSPI_DisableInterrupts(spi_base, kLPSPI_RxInterruptEnable);

        while (LPSPI_GetRxFifoCount(spi_base))
        {
            /*Read out the data*/
            spi3_rx_data.buf[spi3_rx_data.cnt] = LPSPI_ReadData(spi_base);
            spi3_rx_data.cnt++;

            if (spi3_rx_data.cnt == rx_size)
            {
                break;
            }
        }

        /* Re-enable the interrupts only if rxCount indicates there is more data to receive,
         * else we may get a spurious interrupt.
         * */
        if (spi3_rx_data.cnt < rx_size)
        {
            /* Set the TDF and RDF interrupt enables simultaneously to avoid race conditions */
            LPSPI_EnableInterrupts(spi_base, kLPSPI_RxInterruptEnable);
        }
    }

    /*Update rxWatermark. There isn't RX interrupt for the last datas if the RX count is not greater than rxWatermark.*/
    if ((rx_size - spi3_rx_data.cnt) <= spi3_rx_data.water)
    {
        spi_base->FCR =
            (spi_base->FCR & (~LPSPI_FCR_RXWATER_MASK)) |
            LPSPI_FCR_RXWATER(((rx_size - spi3_rx_data.cnt) > 1) ? ((rx_size - spi3_rx_data.cnt) - 1U) : (0U));
    }

    if (spi3_tx_data.cnt < tx_size)
    {
        while ((LPSPI_GetTxFifoCount(spi_base) < spi3_tx_data.fifo) &&
               (spi3_tx_data.cnt - spi3_rx_data.cnt < spi3_tx_data.fifo))
        {
            /*Write the word to TX register*/
            LPSPI_WriteData(spi_base, spi3_tx_data.buf[spi3_tx_data.cnt]);
            spi3_tx_data.cnt++;

            if (spi3_tx_data.cnt == tx_size)
            {
                spi3_tx_data.completed = true;
                /* Complete the transfer and disable the interrupts */
                LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);
                break;
            }
        }
    }

    /* Check if we're done with this transfer.*/
    if ((spi3_tx_data.cnt == tx_size) && (spi3_rx_data.cnt == tx_size))
    {
        spi3_tx_data.completed = true;
        /* Complete the transfer and disable the interrupts */
        LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

DECLARE_HW_IRQ(LPSPI3_IRQn, LPSPI3_IRQHandler, NONE);

void SpiMasterInit(LPSPI_Type *base)
{
    lpspi_master_config_t cfg;

    /*Master config*/
    cfg.baudRate     = TRANSFER_BAUDRATE;
    cfg.bitsPerFrame = 8;
    cfg.cpol         = kLPSPI_ClockPolarityActiveHigh;
    cfg.cpha         = kLPSPI_ClockPhaseFirstEdge;
    cfg.direction    = kLPSPI_MsbFirst;

    cfg.pcsToSckDelayInNanoSec        = 1000000000 / cfg.baudRate;
    cfg.lastSckToPcsDelayInNanoSec    = 1000000000 / cfg.baudRate;
    cfg.betweenTransferDelayInNanoSec = 1000000000 / cfg.baudRate;

    cfg.whichPcs           = SPI_PCS_FOR_INIT;
    cfg.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    cfg.pinCfg        = kLPSPI_SdiInSdoOut;
    cfg.dataOutConfig = kLpspiDataOutRetained;

    LPSPI_MasterInit(base, &cfg, SPI_CLOCK_FREQ);
}

void SpiSlaveInit(LPSPI_Type *base)
{
    lpspi_slave_config_t cfg;

    /*Slave config*/
    cfg.bitsPerFrame = 8;
    cfg.cpol         = kLPSPI_ClockPolarityActiveHigh;
    cfg.cpha         = kLPSPI_ClockPhaseFirstEdge;
    cfg.direction    = kLPSPI_MsbFirst;

    cfg.whichPcs           = SPI_PCS_FOR_INIT;
    cfg.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    cfg.pinCfg        = kLPSPI_SdiInSdoOut;
    cfg.dataOutConfig = kLpspiDataOutRetained;

    LPSPI_SlaveInit(base, &cfg);
}

int SpiHwParamInit(struct Stm32HwSpi *spi_param)
{
    LPSPI_Type *spi_base = spi_param->base;
    uint32_t whichPcs = kLPSPI_Pcs0;
    uint8_t fifo_size, tx_water, rx_water;

    /*The TX and RX FIFO sizes are always the same*/
    fifo_size = LPSPI_GetRxFifoSize(spi_base);

    /*Set the RX and TX watermarks to reduce the ISR times.*/
    if (fifo_size > 1)
    {
        tx_water = 1;
        rx_water = fifo_size - 2;
    }
    else
    {
        tx_water = 0;
        rx_water = 0;
    }

    LPSPI_SetFifoWatermarks(spi_base, tx_water, rx_water);

    LPSPI_Enable(spi_base, false);
    spi_base->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable(spi_base, true);

    /*Flush FIFO , clear status , disable all the inerrupts.*/
    LPSPI_FlushFifo(spi_base, true, true);
    LPSPI_ClearStatusFlags(spi_base, kLPSPI_AllStatusFlag);
    LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);

    spi_base->TCR =
        (spi_base->TCR &
         ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
        LPSPI_TCR_CONT(0) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(whichPcs);

    /* Enable the NVIC for LPSPI peripheral. Note that below code is useless if the LPSPI interrupt is in INTMUX ,
     * and you should also enable the INTMUX interupt in your application.
     */
    EnableIRQ(spi_param->irq);
    LPSPI_EnableInterrupts(spi_base, kLPSPI_RxInterruptEnable);
}

int SpiReadData(struct Stm32HwSpi *spi_param, uint8_t *buf, int len)
{
    int fifo_size;
    int timeout = 50; // wait for read SPI data
    LPSPI_Type *spi_base = spi_param->base;

    BOARD_InitSPIPins();

    /*Set clock source for LPSPI*/
    CLOCK_SetMux(kCLOCK_LpspiMux, SPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, SPI_CLOCK_SOURCE_DIVIDER);

    uint32_t errorCount;
    uint32_t i;
    uint32_t whichPcs;
    uint8_t txWatermark;

    BOARD_InitSPIPins();

    /*Set clock source for LPSPI*/
    CLOCK_SetMux(kCLOCK_LpspiMux, SPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, SPI_CLOCK_SOURCE_DIVIDER);

    /*Set up the transfer data*/
    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        spi1_rx_data.buf[i] = 0;
    }

    SpiSlaveInit(spi_base);

    /******************Set up slave first ******************/
    spi1_rx_data.completed = false;
    spi1_rx_data.cnt = 0;
    spi1_rx_data.size = len;
    whichPcs = SPI_PCS_FOR_INIT;

    /*The TX and RX FIFO sizes are always the same*/
    spi1_rx_data.fifo = LPSPI_GetRxFifoSize(spi_base);

    /*Set the RX and TX watermarks to reduce the ISR times.*/
    if (spi1_rx_data.fifo > 1)
    {
        txWatermark        = 1;
        spi1_rx_data.water = spi1_rx_data.fifo - 2;
    }
    else
    {
        txWatermark        = 0;
        spi1_rx_data.water = 0;
    }

    LPSPI_SetFifoWatermarks(spi_base, txWatermark, spi1_rx_data.water);

    LPSPI_Enable(spi_base, false);
    spi_base->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable(spi_base, true);

    /*Flush FIFO , clear status , disable all the interrupts.*/
    LPSPI_FlushFifo(spi_base, true, true);
    LPSPI_ClearStatusFlags(spi_base, kLPSPI_AllStatusFlag);
    LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);

    spi_base->TCR =
        (spi_base->TCR &
         ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
        LPSPI_TCR_CONT(0) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(whichPcs);

    /* Enable the NVIC for LPSPI peripheral. Note that below code is useless if the LPSPI interrupt is in INTMUX ,
     * and you should also enable the INTMUX interrupt in your application.
     */
    EnableIRQ(spi_param->irq);
    LPSPI_EnableInterrupts(spi_base, kLPSPI_RxInterruptEnable);

    /******************Wait for slave transfer completed.******************/
    while((spi1_rx_data.cnt < (len - 1)) && (timeout--))
    {
        MdelayKTask(100);
    }

    if(buf && (len <= TRANSFER_SIZE))
    {
        memcpy(buf, spi1_rx_data.buf, len);
    }

    LPSPI_Deinit(spi_base);
}

void SpiWriteData(struct Stm32HwSpi *spi_param, const uint8_t *buf, uint16_t len)
{
    uint32_t errorCount;
    uint32_t i;
    uint32_t whichPcs;
    uint8_t txWatermark;
    LPSPI_Type *spi_base = spi_param->base;
    int timeout = 1000; // wait for write SPI data

    BOARD_InitSPIPins();

    /*Set clock source for LPSPI*/
    CLOCK_SetMux(kCLOCK_LpspiMux, SPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, SPI_CLOCK_SOURCE_DIVIDER);
    /*Set up the transfer data*/
    memcpy(spi1_tx_data.buf, buf, len);

    SpiMasterInit(spi_base);

    spi1_rx_data.size = len;
    spi1_tx_data.size = len;
    spi1_tx_data.completed = false;
    spi1_tx_data.cnt = 0;
    spi1_rx_data.cnt = 0;
    whichPcs = SPI_PCS_FOR_INIT;

    /*The TX and RX FIFO sizes are always the same*/
    spi1_tx_data.fifo = LPSPI_GetRxFifoSize(spi_base);

    /*Set the RX and TX watermarks to reduce the ISR times.*/
    if (spi1_tx_data.fifo > 1)
    {
        txWatermark         = 1;
        spi1_rx_data.water = spi1_tx_data.fifo - 2;
    }
    else
    {
        txWatermark         = 0;
        spi1_rx_data.water = 0;
    }

    LPSPI_SetFifoWatermarks(spi_base, txWatermark, spi1_rx_data.water);

    LPSPI_Enable(spi_base, false);
    spi_base->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable(spi_base, true);

    /*Flush FIFO , clear status , disable all the inerrupts.*/
    LPSPI_FlushFifo(spi_base, true, true);
    LPSPI_ClearStatusFlags(spi_base, kLPSPI_AllStatusFlag);
    LPSPI_DisableInterrupts(spi_base, kLPSPI_AllInterruptEnable);

    spi_base->TCR =
        (spi_base->TCR &
         ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
        LPSPI_TCR_CONT(0) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(whichPcs);

    /* Enable the NVIC for LPSPI peripheral. Note that below code is useless if the LPSPI interrupt is in INTMUX ,
     * and you should also enable the INTMUX interupt in your application.
     */
    EnableIRQ(spi_param->irq);
    LPSPI_EnableInterrupts(spi_base, kLPSPI_AllInterruptEnable);

    while((spi1_tx_data.cnt < (spi1_tx_data.size - 1)) && (timeout--))
    {
        MdelayKTask(10);
    }

    spi_print("SPI: master rx %d tx %d size %d completed s %d m %d!\r\n", spi1_rx_data.cnt,
        spi1_tx_data.cnt, spi1_tx_data.size, spi1_rx_data.completed, spi1_tx_data.completed);

    LPSPI_Deinit(spi_base);
}

static uint32 Stm32SpiWriteData(struct SpiHardwareDevice *spi_dev, struct SpiDataStandard *spi_datacfg)
{
    int state;
    x_size_t message_length, already_send_length;
    uint16 send_length;
    const uint8 *write_buf;

    NULL_PARAM_CHECK(spi_dev);
    NULL_PARAM_CHECK(spi_datacfg);

    struct Stm32Spi *StmSpi = CONTAINER_OF(spi_dev->haldev.owner_bus, struct Stm32Spi, spi_bus);
    struct Stm32HwSpi *StmCfg = (struct Stm32HwSpi *) spi_dev->haldev.private_data;

    while(NONE != spi_datacfg) {

      message_length = spi_datacfg->length;
      write_buf = spi_datacfg->tx_buff;
      while (message_length) {
          if (message_length > 65535){
              send_length = 65535;
              message_length = message_length - 65535;
          } else {
              send_length = message_length;
              message_length = 0;
          }

          /* calculate the start address */
          already_send_length = spi_datacfg->length - send_length - message_length;
          write_buf = (uint8 *)spi_datacfg->tx_buff + already_send_length;

          if (spi_datacfg->tx_buff) {
              SpiWriteData(StmCfg, spi_datacfg->tx_buff, send_length);
          }

          if (state != 0) {
              KPrintf("spi write error : %d\n", state);
              spi_datacfg->length = 0;
          }
      }

      spi_datacfg = spi_datacfg->next;
    }

    return EOK;
}

static uint32 Stm32SpiReadData(struct SpiHardwareDevice *spi_dev, struct SpiDataStandard *spi_datacfg)
{
    int state;
    x_size_t message_length, already_send_length;
    uint16 read_length, spi_read_length = 0;
    uint8 *read_buf;

    NULL_PARAM_CHECK(spi_dev);
    NULL_PARAM_CHECK(spi_datacfg);

    struct Stm32Spi *StmSpi = CONTAINER_OF(spi_dev->haldev.owner_bus, struct Stm32Spi, spi_bus);
    struct Stm32HwSpi *StmCfg = (struct Stm32HwSpi *) spi_dev->haldev.private_data;

    while (NONE != spi_datacfg) {
        message_length = spi_datacfg->length;
        read_buf = spi_datacfg->rx_buff;
        while (message_length) {
            if (message_length > 65535) {
                read_length = 65535;
                message_length = message_length - 65535;
            } else {
                read_length = message_length;
                message_length = 0;
            }

            /* calculate the start address */
            already_send_length = spi_datacfg->length - read_length - message_length;
            read_buf = (uint8 *)spi_datacfg->rx_buff + already_send_length;

            if (spi_datacfg->rx_buff) {
                memset(read_buf, 0xff, read_length);
                SpiReadData(StmCfg, read_buf, read_length);
            }

            if (state != 0) {
                KPrintf("spi read error : %d\n", state);
                spi_datacfg->length = 0;
            }
        }

        spi_read_length += spi_datacfg->length;
        spi_datacfg = spi_datacfg->next;
    }

    return spi_read_length;
}

static const struct SpiDevDone spi_dev_done =
{
  .dev_open = NONE,
  .dev_close = NONE,
  .dev_write = Stm32SpiWriteData,
  .dev_read = Stm32SpiReadData,
};

static x_err_t Stm32SpiInit(struct Stm32Spi *spi_drv, struct SpiMasterParam *cfg)
{
    NULL_PARAM_CHECK(spi_drv);
    NULL_PARAM_CHECK(cfg);
}

static uint32 SpiDrvInit(struct SpiDriver *spi_drv)
{
    NULL_PARAM_CHECK(spi_drv);

    SpiDeviceParam *dev_param = (SpiDeviceParam *)(spi_drv->driver.private_data);

    struct Stm32Spi *StmSpi = CONTAINER_OF(spi_drv->driver.owner_bus, struct Stm32Spi, spi_bus);

    return Stm32SpiInit(StmSpi, dev_param->spi_master_param);
}

static uint32 SpiDrvConfigure(struct SpiDriver *spi_drv, struct SpiMasterParam *spi_param)
{
    NULL_PARAM_CHECK(spi_drv);
    NULL_PARAM_CHECK(spi_param);

    SpiDeviceParam *dev_param = (SpiDeviceParam *)(spi_drv->driver.private_data);

    dev_param->spi_master_param = spi_param;
    dev_param->spi_master_param->spi_work_mode = dev_param->spi_master_param->spi_work_mode  & SPI_MODE_MASK;

    return EOK;
}

/*Configure the spi device param, make sure struct (configure_info->private_data) = (SpiMasterParam)*/
static uint32 Stm32SpiDrvConfigure(void *drv, struct BusConfigureInfo *configure_info)
{
    NULL_PARAM_CHECK(drv);
    NULL_PARAM_CHECK(configure_info);

    x_err_t ret = EOK;
    struct SpiDriver *spi_drv = (struct SpiDriver *)drv;
    struct SpiMasterParam *spi_param;

    switch (configure_info->configure_cmd)
    {
      case OPE_INT:
          ret = SpiDrvInit(spi_drv);
          break;
      case OPE_CFG:
          spi_param = (struct SpiMasterParam *)configure_info->private_data;
          ret = SpiDrvConfigure(spi_drv, spi_param);
          break;
      default:
          break;
    }

    return ret;
}

static int BoardSpiBusInit(struct Stm32Spi *stm32spi_bus, struct SpiDriver *spi_driver, char* drv_name)
{
    x_err_t ret = EOK;

    /*Init the spi bus */
    ret = SpiBusInit(&stm32spi_bus->spi_bus, stm32spi_bus->bus_name);
    if (EOK != ret) {
        KPrintf("Board_Spi_init SpiBusInit error %d\n", ret);
        return ERROR;
    }

    /*Init the spi driver*/
    ret = SpiDriverInit(spi_driver, drv_name);
     if (EOK != ret) {
        KPrintf("Board_Spi_init SpiDriverInit error %d\n", ret);
        return ERROR;
    }

    /*Attach the spi driver to the spi bus*/
    ret = SpiDriverAttachToBus(drv_name, stm32spi_bus->bus_name);
    if (EOK != ret) {
        KPrintf("Board_Spi_init SpiDriverAttachToBus error %d\n", ret);
        return ERROR;
    }

    return ret;
}

static int Stm32HwSpiBusInit(void)
{
    x_err_t ret = EOK;
    struct Stm32Spi *StmSpiBus;

#ifdef BSP_USING_SPI1
    StmSpiBus = &spi1;
    StmSpiBus->instance = LPSPI1;
    StmSpiBus->bus_name = SPI_1_BUS_NAME;
    StmSpiBus->spi_bus.private_data = &spi1;

    static struct SpiDriver spi_driver_1;
    memset(&spi_driver_1, 0, sizeof(struct SpiDriver));

    spi_driver_1.configure = &(Stm32SpiDrvConfigure);

    ret = BoardSpiBusInit(StmSpiBus, &spi_driver_1, SPI_1_DRV_NAME);
    if (EOK != ret) {
        KPrintf("Board_Spi_Init spi_bus_init %s error ret %u\n", StmSpiBus->bus_name, ret);
        return ERROR;
    }

    static struct SpiHardwareDevice spi1_dev;
    static struct Stm32HwSpi spi1_cfg;

    spi1_cfg.base = StmSpiBus->instance;
    spi1_cfg.irq = LPSPI1_IRQn;
    spi1_dev.spi_dev_done = &spi_dev_done;

    ret = SpiDeviceRegister(&spi1_dev, (void *)&spi1_cfg, SPI_1_DEV_NAME_0);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }

    ret = SpiDeviceAttachToBus(SPI_1_DEV_NAME_0, SPI_1_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_SPI2
    StmSpiBus = &spi2;
    StmSpiBus->instance = LPSPI2;
    StmSpiBus->bus_name = SPI_2_BUS_NAME;
    StmSpiBus->spi_bus.private_data = &spi2;

    static struct SpiDriver spi_driver_2;
    memset(&spi_driver_2, 0, sizeof(struct SpiDriver));

    spi_driver_2.configure = &(Stm32SpiDrvConfigure);

    ret = BoardSpiBusInit(StmSpiBus, &spi_driver_2, SPI_2_DRV_NAME);
    if (EOK != ret) {
        KPrintf("Board_Spi_Init spi_bus_init %s error ret %u\n", StmSpiBus->bus_name, ret);
        return ERROR;
    }

    static struct SpiHardwareDevice spi2_dev;
    static struct Stm32HwSpi spi2_cfg;

    spi2_cfg.base = StmSpiBus->instance;
    spi2_cfg.irq = LPSPI2_IRQn;
    spi2_dev.spi_dev_done = &spi_dev_done;

    ret = SpiDeviceRegister(&spi2_dev, (void *)&spi2_cfg, SPI_2_DEV_NAME_0);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }

    ret = SpiDeviceAttachToBus(SPI_2_DEV_NAME_0, SPI_2_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }
#endif

#ifdef BSP_USING_SPI3
    StmSpiBus = &spi3;
    StmSpiBus->instance = LPSPI3;
    StmSpiBus->bus_name = SPI_3_BUS_NAME;
    StmSpiBus->spi_bus.private_data = &spi3;

    static struct SpiDriver spi_driver_3;
    memset(&spi_driver_3, 0, sizeof(struct SpiDriver));

    spi_driver_3.configure = &(Stm32SpiDrvConfigure);

    ret = BoardSpiBusInit(StmSpiBus, &spi_driver_3, SPI_3_DRV_NAME);
    if (EOK != ret) {
        KPrintf("Board_Spi_Init spi_bus_init %s error ret %u\n", StmSpiBus->bus_name, ret);
        return ERROR;
    }

    static struct SpiHardwareDevice spi3_dev;
    static struct Stm32HwSpi spi3_cfg;

    spi3_cfg.base = StmSpiBus->instance;
    spi3_cfg.irq = LPSPI3_IRQn;
    spi3_dev.spi_dev_done = &spi_dev_done;

    ret = SpiDeviceRegister(&spi3_dev, (void *)&spi3_cfg, SPI_3_DEV_NAME_0);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }

    ret = SpiDeviceAttachToBus(SPI_3_DEV_NAME_0, SPI_3_BUS_NAME);
    if (ret != EOK) {
        KPrintf("ADC1 device register error %d\n", ret);
        return ERROR;
    }
#endif

    return EOK;
}

x_err_t HwSpiDeviceAttach(const char *bus_name, const char *device_name)
{
    NULL_PARAM_CHECK(bus_name);
    NULL_PARAM_CHECK(device_name);

    x_err_t result;
    struct SpiHardwareDevice *spi_device;
    static SpiDeviceParam spi_dev_param;
    memset(&spi_dev_param, 0, sizeof(SpiDeviceParam));

    /* attach the device to spi bus*/
    spi_device = (struct SpiHardwareDevice *)x_malloc(sizeof(struct SpiHardwareDevice));
    CHECK(spi_device);
    memset(spi_device, 0, sizeof(struct SpiHardwareDevice));

    spi_device->spi_dev_done = &spi_dev_done;

    result = SpiDeviceRegister(spi_device, (void *)&spi_dev_param, device_name);
    if (result != EOK) {
        SYS_ERR("%s device %p register faild, %d\n", device_name, spi_device, result);
    }

    result = SpiDeviceAttachToBus(device_name, bus_name);
    if (result != EOK) {
        SYS_ERR("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    CHECK(result == EOK);

    return result;
}

int Imxrt1052HwSpiInit(void)
{
    return Stm32HwSpiBusInit();
}

void SpiReadTest(void *arg)
{
    uint32_t i;
    uint8_t test_buf[32] = {0};
    struct Stm32HwSpi spi_param;
    spi_param.base = LPSPI1;
    spi_param.irq = LPSPI1_IRQn;

    SpiReadData(&spi_param, test_buf, 32);

    for(i = 0; i < sizeof(test_buf) ;i ++)
    {
        spi_print("%d - %x\n", i, test_buf[i]);
    }
}

SHELL_EXPORT_CMD (SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
                   spiread, SpiReadTest, SPI Read);

void SpiWriteTest(void *arg)
{
    uint32_t i;
    uint8_t test_buf[100] = {0};
    struct Stm32HwSpi spi_param;
    spi_param.base = LPSPI1;
    spi_param.irq = LPSPI1_IRQn;

    for(i = 0; i < sizeof(test_buf) ;i ++)
    {
        test_buf[i] = i;
    }

    SpiWriteData(&spi_param, test_buf, 100);
}

SHELL_EXPORT_CMD (SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_PARAM_NUM(0),
                   spiwrite, SpiWriteTest, SPI Write );


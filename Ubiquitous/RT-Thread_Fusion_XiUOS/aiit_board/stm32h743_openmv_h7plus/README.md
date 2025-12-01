# OPENMV4 H7 PLUS说明

## 使用说明

- 使用ST-LINK无法连接OPENMV时，可以尝试连接RST引脚和GND引脚进入RESET后连接。
- 当前串口通信需要使用USB转TTL连接USART1(P1 TX)(P0 RX)。
- 用户RAM空间为512Kb AXI-SRAM，起始地址 0x24000000。

## 以下为引脚硬件的连接表

### GPIO

| 引脚 | 作用  |
| ---- | ----- |
| PC0  | LED_R |
| PC1  | LED_G |
| PC2  | LED_B |

### USART1 串口

| 引脚 | 作用      |
| ---- | --------- |
| PB14 | USART1 TX |
| PB15 | USART1 RX |

### SDRAM (FMC接口) IS42S32800  BANK1

| 引脚 | 作用       |
| ---- | ---------- |
| PF0  | FMC_A0     |
| PF1  | FMC_A1     |
| PF2  | FMC_A2     |
| PF3  | FMC_A3     |
| PF4  | FMC_A4     |
| PF5  | FMC_A5     |
| PF12 | FMC_A6     |
| PF13 | FMC_A7     |
| PF14 | FMC_A8     |
| PF15 | FMC_A9     |
| PG0  | FMC_A10    |
| PG1  | FMC_A11    |
| PG4  | FMC_BA0    |
| PG5  | FMC_BA1    |
| PD14 | FMC_D0     |
| PD15 | FMC_D1     |
| PD0  | FMC_D2     |
| PD1  | FMC_D3     |
| PE7  | FMC_D4     |
| PE8  | FMC_D5     |
| PE9  | FMC_D6     |
| PE10 | FMC_D7     |
| PE11 | FMC_D8     |
| PE12 | FMC_D9     |
| PE13 | FMC_D10    |
| PE14 | FMC_D11    |
| PE15 | FMC_D12    |
| PD8  | FMC_D13    |
| PD9  | FMC_D14    |
| PD10 | FMC_D15    |
| PH8  | FMC_D16    |
| PH9  | FMC_D17    |
| PH10 | FMC_D18    |
| PH11 | FMC_D19    |
| PH12 | FMC_D20    |
| PH13 | FMC_D21    |
| PH14 | FMC_D22    |
| PH15 | FMC_D23    |
| PI0  | FMC_D24    |
| PI1  | FMC_D25    |
| PI2  | FMC_D26    |
| PI3  | FMC_D27    |
| PI6  | FMC_D28    |
| PI7  | FMC_D29    |
| PI9  | FMC_D30    |
| PI10 | FMC_D31    |
| PE0  | FMC_NBL0   |
| PE1  | FMC_NBL1   |
| PI4  | FMC_NBL2   |
| PI5  | FMC_NBL3   |
| PA7  | FMC_SDNWE  |
| PC4  | FMC_SDNE0  |
| PC5  | FMC_SDCKE0 |
| PG8  | FMC_SDCLK  |
| PG15 | FMC_SDNCAS |
| PF11 | FMC_SDNRAS |

### QSPI FLASH W25Q256JV

| 引脚 | 作用            |
| ---- | --------------- |
| PF6  | QUADSPI_BK1_IO3 |
| PF7  | QUADSPI_BK1_IO2 |
| PF8  | QUADSPI_BK1_IO0 |
| PF9  | QUADSPI_BK1_IO1 |
| PF10 | QUADSPI_CLK     |
| PG6  | QUADSPI_BK1_NCS |

### SDIO USD-1040310811

| 引脚 | 作用       |
| ---- | ---------- |
| PC8  | SDMMC1_D0  |
| PC9  | SDMMC1_D1  |
| PC10 | SDMMC1_D2  |
| PC11 | SDMMC1_D3  |
| PC12 | SDMMC1_CK  |
| PD2  | SDMMC1_CMD |

### USBCDC

| 引脚 | 作用          |
| ---- | ------------- |
| PA11 | USB_OTG_FS_DM |
| PA12 | USB_OTG_FS_DP |
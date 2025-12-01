# STM32F407最小系统板说明

采用ST公司的32位ARM-Cortex M4内核的STM32F407

## 以下为引脚硬件的连接表

## **W25q16(SPI1 )**

| 引脚 | 作用          | 引脚序号  | W25q16  |
| ---- | ---------     | --------  | --------|
| PA5  | SPI1_SCK      | 41        | CLK     |
| PA6  | SPI1_MISO     | 42        | DO      |
| PA7  | SPI1_MOSI     | 43        | DI      |
| PB0  | SpiFlash_nCS  | 46        | nCS     |

## **CRF1278-L3(SPI2 )**

| 引脚 | 作用          | 引脚序号  | CRF1278 |
| ---- | ---------     | --------  | --------|
| PB13 | SPI2_SCK      | 74        | SCK     |
| PC2  | SPI2_MISO     | 28        | MISO    |
| PC3  | SPI2_MOSI     | 29        | MOSI    |
| PC6  | LORA_nCS      | 96        | NSS     |

## **XPT2046(SPI3)**

| 引脚 | 作用          | 引脚序号   | XPT2046 |
| ---- | ---------     | --------   | --------|
| PB3  | SPI3_SCK      | 133        | DCLK    |
| PB4  | SPI3_MISO     | 134        | DOUT    |
| PB5  | SPI3_MOSI     | 135        | DIN     |
| PG13 | T_nCS         | 128        | CS      |

## **sensor(I2C1)**

| PB6  | I2C_SCL     | 
| ---- | ----------  |
| PB7  | I2C_SDA     |


## **TTL_debug(uart1)**   
| 引脚 | 作用      |  引脚序号   | TTL_debug  |
| ---- | ----------|------------ |---------   |
| PA9  | USART1_TX |    101      |USART1_RX   |
| PA10 | USART1_RX |    102      |USART1_TX   |

## **NB/4G(uart2)**  

| 引脚 | 作用      |  引脚序号   |  NB/4G     |
| ---- | ----------|------------ |---------   |
| PA2  | USART2_TX |    36       |USART2_RX   |
| PA3  | USART2_RX |    37       |USART2_TX   |

## **Ethernet/WIFI(uart3)** 

| 引脚  | 作用      |  引脚序号   | Ethernet/WIFI |
| ----  | ----------|------------ |-------------  |
| PB10  | USART3_TX |    69       |UART0_RXD      |
| PB11  | USART3_RX |    70       |UART0_TXD      |


## **BT-HC08(uart4)** 

| 引脚       | 作用     |  引脚序号   | BT-HC08  |
| -----------| ---------|------------ |--------- |
| PA0_WAKEUP | UART4_TX |    34       | RXD      |
| PA1        | UART4_RX |    35       | TXD      |

## **SD**

| 引脚 | 作用         | 引脚序号   | TF-01A   |
| ---- | ---------    | --------   | ---------|
| PC10 | SDIO_D2      | 114        | DATA2    |
| PC11 | SDIO_D3      | 115        | DATA3    |
| PD2  | SDIO_CMD     | 116        | CMD      |
| PC12 | SDIO_CK      | 113        | CLK      |
| PC8  | SDIO_D0      | 98         | DATA0    |
| PC9  | SDIO_D1      | 99         | DATA1    |

## **USB-HOST**

| 引脚       |  作用      |  引脚序号    |  USB-S     |
| -----------| ---------  |------------  |------------|
| PA12       | USB_OTG_DP |    104       | USB_OTG_DP |
| PA11       | USB_OTG_DM |    103       | USB_OTG_DM |

## **CAN**
| 引脚       |  作用     |  引脚序号    |  USB-S  |
| -----------| --------- |------------  |---------|
| PB8        | CAN1_RX   |    139       | CAN1_TX |
| PB9        | CAN1_TX   |    140       | CAN1_RX |

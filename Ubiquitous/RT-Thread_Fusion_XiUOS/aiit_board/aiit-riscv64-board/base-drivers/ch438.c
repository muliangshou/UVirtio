#include <rtthread.h>
#include <rtdevice.h>
#include <drv_io_config.h>
#include <gpiohs.h>
#include "board.h"
#include "ch438.h"
#include "sleep.h"
#include <math.h>

static rt_uint8_t	offsetadd[] = {0x00,0x10,0x20,0x30,0x08,0x18,0x28,0x38,};		/* Offset address of serial port number */
struct rt_serial_device *extuart_serial_parm[8];

void CH438_INIT(void)
{	
	CH438_set_output();
 	gpiohs_set_drive_mode(FPIOA_CH438_NWR, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_NRD, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_ALE, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_INT, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_485_DIR, GPIO_DM_OUTPUT);
	
	gpiohs_set_pin(FPIOA_CH438_NWR, GPIO_PV_HIGH);
	gpiohs_set_pin(FPIOA_CH438_NRD, GPIO_PV_HIGH);
	gpiohs_set_pin(FPIOA_CH438_ALE, GPIO_PV_HIGH);
}	

void CH438_PORT_INIT( rt_uint8_t ext_uart_no,rt_uint32_t BaudRate )
{
	rt_uint32_t	div;
	rt_uint8_t	DLL,DLM,dlab;
	rt_uint8_t	REG_LCR_ADDR;
	rt_uint8_t	REG_DLL_ADDR;
	rt_uint8_t	REG_DLM_ADDR;
	rt_uint8_t	REG_IER_ADDR;
	rt_uint8_t	REG_MCR_ADDR;
	rt_uint8_t	REG_FCR_ADDR;
	rt_uint8_t	REG_RBR_ADDR;
	rt_uint8_t	REG_THR_ADDR;
	rt_uint8_t	REG_IIR_ADDR;
	
	REG_LCR_ADDR = offsetadd[ext_uart_no] | REG_LCR0_ADDR;
	REG_DLL_ADDR = offsetadd[ext_uart_no] | REG_DLL0_ADDR;
	REG_DLM_ADDR = offsetadd[ext_uart_no] | REG_DLM0_ADDR;
	REG_IER_ADDR = offsetadd[ext_uart_no] | REG_IER0_ADDR;
	REG_MCR_ADDR = offsetadd[ext_uart_no] | REG_MCR0_ADDR;
	REG_FCR_ADDR = offsetadd[ext_uart_no] | REG_FCR0_ADDR;
	REG_RBR_ADDR = offsetadd[ext_uart_no] | REG_RBR0_ADDR;
	REG_THR_ADDR = offsetadd[ext_uart_no] | REG_THR0_ADDR;
	REG_IIR_ADDR = offsetadd[ext_uart_no] | REG_IIR0_ADDR;
			
    WriteCH438Data( REG_IER_ADDR, BIT_IER_RESET );             /* Reset the serial port */
	rt_thread_delay(50);
	
	dlab = ReadCH438Data(REG_IER_ADDR);
	dlab &= 0xDF;
	WriteCH438Data(REG_IER_ADDR, dlab);
	
	dlab = ReadCH438Data(REG_LCR_ADDR);
	dlab |= 0x80;
	WriteCH438Data(REG_LCR_ADDR, dlab);

    div = ( Fpclk >> 4 ) / BaudRate;
    DLM = div >> 8;
    DLL = div & 0xff;
	WriteCH438Data( REG_DLL_ADDR, DLL );             /* Set baud rate */
    WriteCH438Data( REG_DLM_ADDR, DLM );
	WriteCH438Data( REG_FCR_ADDR, BIT_FCR_RECVTG1 | BIT_FCR_RECVTG0 | BIT_FCR_FIFOEN );    /* Set FIFO mode */

    WriteCH438Data( REG_LCR_ADDR, BIT_LCR_WORDSZ1 | BIT_LCR_WORDSZ0 );

    WriteCH438Data( REG_IER_ADDR, /*BIT_IER_IEMODEM | BIT_IER_IETHRE |  BIT_IER_IELINES | */BIT_IER_IERECV );

    WriteCH438Data( REG_MCR_ADDR, BIT_MCR_OUT2  );

	WriteCH438Data(REG_FCR_ADDR,ReadCH438Data(REG_FCR_ADDR)| BIT_FCR_RFIFORST | BIT_FCR_TFIFORST);

}

static void CH438_set_output(void)
{
 	gpiohs_set_drive_mode(FPIOA_CH438_D0, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D1, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D2, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D3, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D4, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D5, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D6, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(FPIOA_CH438_D7, GPIO_DM_OUTPUT);
	
}
static void CH438_set_input(void)
{
	gpiohs_set_drive_mode(FPIOA_CH438_D0, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D1, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D2, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D3, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D4, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D5, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D6, GPIO_DM_INPUT_PULL_UP);
	gpiohs_set_drive_mode(FPIOA_CH438_D7, GPIO_DM_INPUT_PULL_UP);
	
}


void set_485_input(rt_uint8_t	ch_no)
{
	if(ch_no == 1)
		gpiohs_set_pin(FPIOA_485_DIR, GPIO_PV_LOW);
}

void set_485_output(rt_uint8_t	ch_no)
{
	if(ch_no == 1)
		gpiohs_set_pin(FPIOA_485_DIR, GPIO_PV_HIGH);
}

rt_uint8_t ReadCH438Data( rt_uint8_t addr )
{
	rt_uint8_t dat = 0;

	gpiohs_set_pin(FPIOA_CH438_NWR,GPIO_PV_HIGH);	
	gpiohs_set_pin(FPIOA_CH438_NRD,GPIO_PV_HIGH);	
	gpiohs_set_pin(FPIOA_CH438_ALE,GPIO_PV_HIGH);	

	CH438_set_output();
	usleep(1);
	
	if(addr &0x80)	gpiohs_set_pin(FPIOA_CH438_D7,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D7,GPIO_PV_LOW);	
	if(addr &0x40)	gpiohs_set_pin(FPIOA_CH438_D6,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D6,GPIO_PV_LOW);	
	if(addr &0x20)	gpiohs_set_pin(FPIOA_CH438_D5,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D5,GPIO_PV_LOW);	
	if(addr &0x10)	gpiohs_set_pin(FPIOA_CH438_D4,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D4,GPIO_PV_LOW);	
	if(addr &0x08)	gpiohs_set_pin(FPIOA_CH438_D3,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D3,GPIO_PV_LOW);	
	if(addr &0x04)	gpiohs_set_pin(FPIOA_CH438_D2,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D2,GPIO_PV_LOW);	
	if(addr &0x02)	gpiohs_set_pin(FPIOA_CH438_D1,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D1,GPIO_PV_LOW);	
	if(addr &0x01)	gpiohs_set_pin(FPIOA_CH438_D0,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D0,GPIO_PV_LOW);	
		
	usleep(1);

	gpiohs_set_pin(FPIOA_CH438_ALE,GPIO_PV_LOW);	

	usleep(1);		

	CH438_set_input();
	usleep(1);
	
	gpiohs_set_pin(FPIOA_CH438_NRD,GPIO_PV_LOW);	
	
	usleep(1);	
	
	dat = 0;
	if (gpiohs_get_pin(FPIOA_CH438_D7))	dat |= 0x80;
	if (gpiohs_get_pin(FPIOA_CH438_D6))	dat |= 0x40;
	if (gpiohs_get_pin(FPIOA_CH438_D5))	dat |= 0x20;
	if (gpiohs_get_pin(FPIOA_CH438_D4))	dat |= 0x10;
	if (gpiohs_get_pin(FPIOA_CH438_D3))	dat |= 0x08;
	if (gpiohs_get_pin(FPIOA_CH438_D2))	dat |= 0x04;
	if (gpiohs_get_pin(FPIOA_CH438_D1))	dat |= 0x02;
	if (gpiohs_get_pin(FPIOA_CH438_D0))	dat |= 0x01;
	
	gpiohs_set_pin(FPIOA_CH438_NRD,GPIO_PV_HIGH);	
	gpiohs_set_pin(FPIOA_CH438_ALE,GPIO_PV_HIGH);	

	usleep(1);

	return dat;
}
	

static void WriteCH438Data( rt_uint8_t addr, rt_uint8_t dat)
{
	gpiohs_set_pin(FPIOA_CH438_ALE,GPIO_PV_HIGH);	
	gpiohs_set_pin(FPIOA_CH438_NRD,GPIO_PV_HIGH);	
	gpiohs_set_pin(FPIOA_CH438_NWR,GPIO_PV_HIGH);	

	CH438_set_output();
	usleep(1);	
	
	if(addr &0x80)	gpiohs_set_pin(FPIOA_CH438_D7,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D7,GPIO_PV_LOW);	
	if(addr &0x40)	gpiohs_set_pin(FPIOA_CH438_D6,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D6,GPIO_PV_LOW);	
	if(addr &0x20)	gpiohs_set_pin(FPIOA_CH438_D5,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D5,GPIO_PV_LOW);	
	if(addr &0x10)	gpiohs_set_pin(FPIOA_CH438_D4,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D4,GPIO_PV_LOW);	
	if(addr &0x08)	gpiohs_set_pin(FPIOA_CH438_D3,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D3,GPIO_PV_LOW);	
	if(addr &0x04)	gpiohs_set_pin(FPIOA_CH438_D2,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D2,GPIO_PV_LOW);	
	if(addr &0x02)	gpiohs_set_pin(FPIOA_CH438_D1,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D1,GPIO_PV_LOW);	
	if(addr &0x01)	gpiohs_set_pin(FPIOA_CH438_D0,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D0,GPIO_PV_LOW);	
	
	usleep(1);	
	
	gpiohs_set_pin(FPIOA_CH438_ALE,GPIO_PV_LOW);	
	usleep(1);
	
	if(dat &0x80)	gpiohs_set_pin(FPIOA_CH438_D7,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D7,GPIO_PV_LOW);	
	if(dat &0x40)	gpiohs_set_pin(FPIOA_CH438_D6,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D6,GPIO_PV_LOW);	
	if(dat &0x20)	gpiohs_set_pin(FPIOA_CH438_D5,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D5,GPIO_PV_LOW);	
	if(dat &0x10)	gpiohs_set_pin(FPIOA_CH438_D4,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D4,GPIO_PV_LOW);	
	if(dat &0x08)	gpiohs_set_pin(FPIOA_CH438_D3,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D3,GPIO_PV_LOW);	
	if(dat &0x04)	gpiohs_set_pin(FPIOA_CH438_D2,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D2,GPIO_PV_LOW);	
	if(dat &0x02)	gpiohs_set_pin(FPIOA_CH438_D1,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D1,GPIO_PV_LOW);	
	if(dat &0x01)	gpiohs_set_pin(FPIOA_CH438_D0,GPIO_PV_HIGH);	else	gpiohs_set_pin(FPIOA_CH438_D0,GPIO_PV_LOW);	
	
	usleep(1);	

	gpiohs_set_pin(FPIOA_CH438_NWR,GPIO_PV_LOW);	

	usleep(1);	
	
	gpiohs_set_pin(FPIOA_CH438_NWR,GPIO_PV_HIGH);	
	gpiohs_set_pin(FPIOA_CH438_ALE,GPIO_PV_HIGH);	
	
	usleep(1);	

	CH438_set_input();

	return;
}

static void WriteCH438Block( rt_uint8_t mAddr, rt_uint8_t mLen, rt_uint8_t *mBuf )   
{
    while ( mLen -- ) 	
	  WriteCH438Data( mAddr, *mBuf++ );
}

static int Ch438Irq(void *parameter)
{
	rt_uint8_t gInterruptStatus;
	rt_uint8_t port = 0;
	struct rt_serial_device *serial = (struct rt_serial_device *)parameter;
	/* multi irq may happen*/
	gInterruptStatus = ReadCH438Data(REG_SSR_ADDR);
	port = log(gInterruptStatus & 0xFF)/log(2);

	rt_hw_serial_isr(extuart_serial_parm[port], RT_SERIAL_EVENT_RX_IND);
}

static rt_err_t rt_extuart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
	rt_uint32_t baud_rate = cfg->baud_rate;
	rt_uint16_t port = cfg->reserved;

	CH438_PORT_INIT(port, baud_rate);

	return RT_EOK;
}

static rt_err_t extuart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
	rt_uint16_t ext_uart_no = serial->config.reserved;
	static rt_uint16_t register_flag = 0;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
		if(1 == register_flag)
		{
			gpiohs_irq_unregister(FPIOA_CH438_INT);
			register_flag = 0;
		}
        break;
    case RT_DEVICE_CTRL_SET_INT:
		if(0 == register_flag)
		{
			gpiohs_set_drive_mode(FPIOA_CH438_INT, GPIO_DM_INPUT_PULL_UP);
    		gpiohs_set_pin_edge(FPIOA_CH438_INT,GPIO_PE_FALLING);
			gpiohs_irq_register(FPIOA_CH438_INT, 1, Ch438Irq, (void*)serial);
			register_flag = 1;
		}
		
		break;
    }
    return (RT_EOK);
}

static int drv_extuart_putc(struct rt_serial_device *serial, char c)
{
	uint16_t ext_uart_no = serial->config.reserved;
	rt_uint8_t	REG_LSR_ADDR,REG_THR_ADDR;
	
	REG_LSR_ADDR = offsetadd[ext_uart_no] | REG_LSR0_ADDR;
	REG_THR_ADDR = offsetadd[ext_uart_no] | REG_THR0_ADDR;


	if((ReadCH438Data( REG_LSR_ADDR ) & BIT_LSR_TEMT) != 0)
	{
		WriteCH438Block( REG_THR_ADDR, 1, &c );
		return 1;
	} else {
		return 0;
	}
	
}

static int drv_extuart_getc(struct rt_serial_device *serial)
{
	rt_uint8_t	dat = 0;
	rt_uint8_t	REG_LSR_ADDR,REG_RBR_ADDR;
	uint16_t ext_uart_no = serial->config.reserved;///< get extern uart port
	
	REG_LSR_ADDR = offsetadd[ext_uart_no] | REG_LSR0_ADDR;
	REG_RBR_ADDR = offsetadd[ext_uart_no] | REG_RBR0_ADDR;

	if((ReadCH438Data(REG_LSR_ADDR) & BIT_LSR_DATARDY) == 0x01)
	{
		dat = ReadCH438Data( REG_RBR_ADDR );
		if(dat >= 0)
			return dat;
	} else {
		return -1;
	}
}

const struct rt_uart_ops extuart_ops =
{
    rt_extuart_configure,
    extuart_control,
    drv_extuart_putc,
    drv_extuart_getc,
    RT_NULL
};

int rt_hw_ch438_init(void)
{
    struct rt_serial_device *extserial;
    struct device_uart      *extuart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	rt_err_t ret;

    {
        static struct rt_serial_device  extserial0;

        extserial  = &extserial0;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 115200;
		extserial->config.reserved  = 0; ///< extern uart port

        extuart_serial_parm[0] = &extserial0;

        ret = rt_hw_serial_register(extserial,
                              "extuart_dev0",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev0 register failed.\n");
		}
    }
	{
        static struct rt_serial_device  extserial1;

        extserial  = &extserial1;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 9600;
		extserial->config.reserved = 1; ///< extern uart port

        extuart_serial_parm[1] = &extserial1;

        ret = rt_hw_serial_register(extserial,
                              "extuart_dev1",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev1 register failed.\n");
		}
    }
	{
        static struct rt_serial_device  extserial2;

        extserial  = &extserial2;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 9600;
		extserial->config.reserved = 2; ///< extern uart port

        extuart_serial_parm[2] = &extserial2;

       ret = rt_hw_serial_register(extserial,
                              "extuart_dev2",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev2 register failed.\n");
		}

    }
	{
        static struct rt_serial_device  extserial3;

        extserial  = &extserial3;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 9600;
		extserial->config.reserved = 3; ///< extern uart port

       ret = rt_hw_serial_register(extserial,
                              "extuart_dev3",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev3 register failed.\n");
		}

		extuart_serial_parm[3] = &extserial3;
    }
	{
        static struct rt_serial_device  extserial4;

        extserial  = &extserial4;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 9600;
		extserial->config.reserved = 4; ///< extern uart port

       ret = rt_hw_serial_register(extserial,
                              "extuart_dev4",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev4 register failed.\n");
		}

		extuart_serial_parm[4] = &extserial4;
    }
	{
        static struct rt_serial_device  extserial5;

        extserial  = &extserial5;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 115200;
		extserial->config.reserved = 5; ///< extern uart port

       ret = rt_hw_serial_register(extserial,
                              "extuart_dev5",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev5 register failed.\n");
		}

		extuart_serial_parm[5] = &extserial5;
    }
	{
        static struct rt_serial_device  extserial6;

        extserial  = &extserial6;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 57600;
		extserial->config.reserved = 6; ///< extern uart port

       ret = rt_hw_serial_register(extserial,
                              "extuart_dev6",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev6 register failed.\n");
		}

		extuart_serial_parm[6] = &extserial6;
    }
	{
        static struct rt_serial_device  extserial7;

        extserial  = &extserial7;
        extserial->ops              = &extuart_ops;
        extserial->config           = config;
        extserial->config.baud_rate = 9600;
		extserial->config.reserved = 7; ///< extern uart port

       ret = rt_hw_serial_register(extserial,
                              "extuart_dev7",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                              extuart);
		if(ret < 0){
			rt_kprintf("extuart_dev7 register failed.\n");
		}
		extuart_serial_parm[7] = &extserial7;

    }

	CH438_INIT();
    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_ch438_init);

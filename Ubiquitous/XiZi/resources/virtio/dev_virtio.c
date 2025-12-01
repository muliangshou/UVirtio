/**
* @file dev_virtio.c
* @brief register virtio dev function using bus driver framework
* @version 1.0 
* @author Elliot
* @date 2021-04-24
*/

/*************************************************
File name: dev_virtio.c

History: 
1. Date: 2022-04-20
Author: Elliot
Modification: 
1. support virtio dev register, configure, write and read
2. add bus driver framework support, include INT and DMA mode
*************************************************/

#include <bus_virtio.h>
#include <dev_virtio.h>
#include <virtio.h>

static DoubleLinklistType virtiodev_linklist;

/**
 *	VirtioFeaturesSelect
 *	Description: 
 *		set virtio register bits according to virtio cap (TODO: not implemented)
 * 	@regs:	virtio device registers
 *  @caps:	capabilities a virtio device support, fixed
 *  @n: 	number of capability entries
 *  @whom:	name of the virtio device
 */
void VirtioFeaturesSelect(VirtioRegs *regs, struct VirtioCap *caps, uint32 n, char *whom)
{
    uint32 i;
	uint32 bank = 0;
	uint32 driver = 0;
	uint32 device;

    // DeviceFeaturesSel = 0 => use feature bit 0 to 31
    // DeviceFeaturesSel = 1 => use feature bit 32 to 63
	WRITE_ONCE(regs->DeviceFeaturesSel, bank);
	mb();
    // NOTE: read features the device offers, this should be originally determined by device itself
	device = READ_ONCE(regs->DeviceFeatures);

	for (i = 0; i < n; i++) {
		if (caps[i].bit / 32 != bank) {
			/* Time to write our selected bits for this bank */
			WRITE_ONCE(regs->DriverFeaturesSel, bank);
			mb();
			WRITE_ONCE(regs->DriverFeatures, driver);
			if (device) {
				/*printf("%s: device supports unknown bits"
				       " 0x%x in bank %u\n", whom, device,
				   bank);*/
			}
			/* Now we set these variables for next time. */
			bank = caps[i].bit / 32;
			WRITE_ONCE(regs->DeviceFeaturesSel, bank);
			mb();
			device = READ_ONCE(regs->DeviceFeatures);
		}
        // NOTE: variable device contains device supported features
        // NOTE: variable driver contains features driver choosed
		if (device & (1 << caps[i].bit)) {
			if (caps[i].support) {
				driver |= (1 << caps[i].bit);
			} else {
				/*printf("virtio supports unsupported option %s
				   "
				       "(%s)\n",
				       caps[i].name, caps[i].help);*/
			}
			/* clear this from device now */
			device &= ~(1 << caps[i].bit);
		}
	}
	/* Time to write our selected bits for this bank */
    // NOTE: same as DeviceFeaturesSel bit
	WRITE_ONCE(regs->DriverFeaturesSel, bank);
	mb();
	WRITE_ONCE(regs->DriverFeatures, driver);
	if (device) {
		/*printf("%s: device supports unknown bits"
		       " 0x%x in bank %u\n", whom, device, bank);*/
	}
}

/* Find vq of virtio device */
VirtqueueType VirtioDevFindVq(struct VirtioHardwareDevice *dev, const char *vq_name)
{
	VirtqueueType vq = NONE;

    DoubleLinklistType *node = NONE;
    DoubleLinklistType *head = &(dev->vqs);

    for (node = head->node_next; node != head; node = node->node_next)
    {
        vq = SYS_DOUBLE_LINKLIST_ENTRY(node, struct virtqueue, vq_list);
        if(!strcmp(vq->name, vq_name)) {
            return vq;
        }
    }

    KPrintf("VirtioDevFindVq cannot find the %s vq.return NULL\n", vq_name);
    return NONE;
}

// NOTE: this is used to open a device
static uint32 VirtioDevOpen(void *dev)
{
    NULL_PARAM_CHECK(dev);

    int virtio_operation_cmd;
    struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)dev;
    struct Driver *drv = vdev->haldev.owner_bus->owner_driver;
    struct VirtioDriver *virtio_drv = (struct VirtioDriver *)drv;

    // FIXME: init hardware uart1
    virtio_operation_cmd = OPER_SET_INT;
    virtio_drv->drv_done->configure(virtio_drv, virtio_operation_cmd);

    // FIXME: maybe these initialization should not be here
    VirtioDeviceStatusInit(drv);
 
    // Step 7: initialization according to device type
    switch (vdev->regs->DeviceID)
    {
    case VIRTIO_ID_BLOCK:
        VirtioBlkInit(vdev);
        break;
    case VIRTIO_ID_SERIAL:
        VirtioSerialInit(vdev);
        break;
    case VIRTIO_ID_NETWORK:
        VirtioNetInit(vdev);
        break;
    default:
        KPrintf("VirtioDevOpen: Device ID unsupported.\n");
		return ERROR;
    }
    
    vdev->haldev.dev_sem = KSemaphoreCreate(0);
	if (vdev->haldev.dev_sem < 0) {
		KPrintf("VirtioDevOpen create sem failed .\n");
		return ERROR;
	}

    return EOK;
}

static uint32 VirtioDevClose(void *dev)
{
    NULL_PARAM_CHECK(dev);

    int virtio_operation_cmd = OPER_CLR_INT;
    struct VirtioHardwareDevice *virtio_dev = (struct VirtioHardwareDevice *)dev;
    struct Driver *drv = virtio_dev->haldev.owner_bus->owner_driver;
    struct VirtioDriver *virtio_drv = (struct VirtioDriver *)drv;

    // TODO: device close

    KSemaphoreDelete(virtio_dev->haldev.dev_sem);
    return EOK;
}

static uint32 VirtioDevWrite(void *dev, struct BusBlockWriteParam *write_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(write_param);

    x_err_t ret = EOK;

    struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)dev;
    struct VirtioDevParam *virtio_dev_param = (struct VirtioDevParam *)vdev->haldev.private_data;
    
    switch (vdev->regs->DeviceID)
    {
    case VIRTIO_ID_BLOCK:
        ret = VirtioBlkWrite(vdev, write_param);
        break;
    case VIRTIO_ID_SERIAL:
        ret = VirtioSerialWrite(vdev, write_param);
        break;
    case VIRTIO_ID_NETWORK:
        ret = VirtioNetWrite(vdev, write_param);
        break;
    default:
        KPrintf("VirtioDevWrite: Device ID %d unsupported.\n", vdev->regs->DeviceID);
		return ERROR;
    }
    if (EOK != ret) {
        KPrintf("VirtioDevWrite error %d\n", ret);
        return ERROR;
    }

    return EOK;
}

static uint32 VirtioDevRead(void *dev, struct BusBlockReadParam *read_param)
{
    NULL_PARAM_CHECK(dev);
    NULL_PARAM_CHECK(read_param);

    x_err_t ret = EOK;

    struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)dev;
    struct VirtioDevParam *virtio_dev_param = (struct VirtioDevParam *)vdev->haldev.private_data;

    switch (vdev->regs->DeviceID)
    {
    case VIRTIO_ID_BLOCK:
        ret = VirtioBlkRead(vdev, read_param);
        break;
    case VIRTIO_ID_SERIAL:
        ret = VirtioSerialRead(vdev, read_param);
        break;
    case VIRTIO_ID_NETWORK:
        ret = VirtioNetRead(vdev, read_param);
        // VirtioNetDeviceRx(vdev, 1);
        break;
    default:
        KPrintf("VirtioDevRead: Device ID unsupported.\n");
		return ERROR;
    }
    if (EOK != ret) {
        KPrintf("VirtioDevRead error %d\n", ret);
        return ERROR;
    }

    return EOK;
}

void VirtioSetIsr(void *dev, int event)
{
    // TODO: currently only use write, no need to handle interrupt
    NULL_PARAM_CHECK(dev);

    struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)dev;
    struct VirtioDevParam *virtio_dev_param = (struct VirtioDevParam *)vdev->haldev.private_data;

    switch (vdev->regs->DeviceID)
    {
    case VIRTIO_ID_BLOCK:
        KPrintf("VirtioSetIsr: virtio-blk do not support isr yet!\n");
        break;
    case VIRTIO_ID_SERIAL:
        VirtioSerialDeviceGetChar(vdev, event);
        break;
    case VIRTIO_ID_NETWORK:
        VirtioNetDeviceRx(vdev, event);
        break;
    default:
        KPrintf("VirtioSetIsr: Device ID unsupported.\n");
		return;
    }
    return;
}

// FIXME: set device regs according to virtio spec, this should be done automatedly by hardware
static void VirtioRegsInit(VirtioRegs *regs)
{
    // Step 1: set MagicValue
    regs->MagicValue = VIRTIO_MAGIC;
    
    // Step 2: set VirtioVersion
    regs->Version = VIRTIO_VERSION;

    // Step 3: set DeviceID
    // FIXME: since we now only try virtio-serial this is dummy
    regs->DeviceID = VIRTIO_ID_BLOCK;

    // TODO: device features related initialization
    return;
}

const struct VirtioDevDone virtio_dev_done =
{
    .open = VirtioDevOpen,
    .close = VirtioDevClose,
    .write = VirtioDevWrite,
    .read = VirtioDevRead,
};

/*Create the virtio device linklist*/
static void VirtioDeviceLinkInit()
{
    InitDoubleLinkList(&virtiodev_linklist);
}

HardwareDevType VirtioDeviceFind(const char *dev_name, enum DevType dev_type)
{
    NULL_PARAM_CHECK(dev_name);
    
    struct HardwareDev *device = NONE;

    DoubleLinklistType *node = NONE;
    DoubleLinklistType *head = &virtiodev_linklist;

    for (node = head->node_next; node != head; node = node->node_next) {
        device = SYS_DOUBLE_LINKLIST_ENTRY(node, struct HardwareDev, dev_link);
        if ((!strcmp(device->dev_name, dev_name)) && (dev_type == device->dev_type)) {
            return device;
        }
    }

    KPrintf("VirtioDeviceFind cannot find the %s device.return NULL\n", dev_name);
    return NONE;
}

int VirtioDeviceRegister(struct VirtioHardwareDevice *virtio_device, void *virtio_param, const char *device_name)
{
    NULL_PARAM_CHECK(virtio_device);
    NULL_PARAM_CHECK(device_name);

    x_err_t ret = EOK;    
    static x_bool dev_link_flag = RET_FALSE;

    if (!dev_link_flag) {
        VirtioDeviceLinkInit();
        dev_link_flag = RET_TRUE;
    }

    if (DEV_INSTALL != virtio_device->haldev.dev_state) {
        strncpy(virtio_device->haldev.dev_name, device_name, NAME_NUM_MAX);
        virtio_device->haldev.dev_type = TYPE_VIRTIO_DEV;
        virtio_device->haldev.dev_state = DEV_INSTALL;

        virtio_device->haldev.dev_done = (struct HalDevDone *)&virtio_dev_done;
        
        virtio_device->private_data = virtio_param;

        // NOTE: to support stm32f407 comment this, regs bind is move to connect_usart.c
        // virtio_device->regs = (VirtioRegs *)virtio_param;

        DoubleLinkListInsertNodeAfter(&virtiodev_linklist, &(virtio_device->haldev.dev_link));
    } else {
        KPrintf("VirtioDeviceRegister device has been register state%u\n", virtio_device->haldev.dev_state);        
    }

    return ret;
}

int VirtioDeviceAttachToBus(const char *dev_name, const char *bus_name)
{
    NULL_PARAM_CHECK(dev_name);
    NULL_PARAM_CHECK(bus_name);
    
    x_err_t ret = EOK;

    struct Bus *bus;
    struct HardwareDev *device;

    bus = BusFind(bus_name);
    if (NONE == bus) {
        KPrintf("VirtioDeviceAttachToBus find virtio bus error!name %s\n", bus_name);
        return ERROR;
    }
    
    if (TYPE_VIRTIO_BUS == bus->bus_type) {
        device = VirtioDeviceFind(dev_name, TYPE_VIRTIO_DEV);
        if (NONE == device) {
            KPrintf("VirtioDeviceAttachToBus find virtio device error!name %s\n", dev_name);
            return ERROR;
        }

        if (TYPE_VIRTIO_DEV == device->dev_type) {
            ret = DeviceRegisterToBus(bus, device);
            if (EOK != ret) {
                KPrintf("VirtioDeviceAttachToBus DeviceRegisterToBus error %u\n", ret);
                return ERROR;
            }
        }
    }

    return EOK;
}


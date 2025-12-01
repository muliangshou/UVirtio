/**
* @file drv_virtio.c
* @brief register virtio drv function using bus driver framework
* @version 1.0 
* @author Elliot
* @date 2021-04-24
*/

#include <bus_virtio.h>
#include <dev_virtio.h>
#include <virtio_mmio.h>
#include <virtio.h>

static DoubleLinklistType virtiodrv_linklist;

/*Create the driver linklist*/
static void VirtioDrvLinkInit()
{
    InitDoubleLinkList(&virtiodrv_linklist);
}

/* Read and check values from MagicValue & Version */
int VirtioDeviceStatusInit(struct Driver *drv)
{   
    struct VirtioDriver *vdrv = (struct VirtioDriver *)drv;

    struct HardwareDev *dev = vdrv->driver.owner_bus->owner_haldev;
    struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)dev;
    VirtioRegs *regs = vdev->regs;

    // Step 1: check MagicValue
    if (READ_ONCE(regs->MagicValue) != VIRTIO_MAGIC) {
        KPrintf("Error: virtio at 0x%x had wrong magic value 0x%x, "
		       "expected 0x%x\n",
		       regs, regs->MagicValue, VIRTIO_MAGIC);
        return -ERROR;
    }

    // Step 2: check Version
    if (READ_ONCE(regs->Version) != VIRTIO_VERSION) {
        KPrintf("error: virtio at 0x%x had wrong version 0x%x, expected "
		       "0x%x\n",
		       regs, regs->Version, VIRTIO_VERSION);
		return -ERROR;
    }

    // Step 3: check DeviceID
    if (READ_ONCE(regs->DeviceID) == 0) {
		/*On QEMU, this is pretty common, don't print a message */
		/*printf("warn: virtio at 0x%x has DeviceID=0, skipping\n",
		 * virt);*/
		return -ERROR;
	}

    // Step 4: reset the device
    WRITE_ONCE(regs->Status, 0);
    mb();

    // Step 5: Acknowledge
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_ACKNOWLEDGE);
    mb();

    // Step 6: Set Driver bit
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_DRIVER);
    mb();
}

DriverType VirtioDriverFind(const char *drv_name, enum DriverType_e drv_type)
{
    NULL_PARAM_CHECK(drv_name);
    
    struct Driver *driver = NONE;

    DoubleLinklistType *node = NONE;
    DoubleLinklistType *head = &virtiodrv_linklist;

    for (node = head->node_next; node != head; node = node->node_next) {
        driver = SYS_DOUBLE_LINKLIST_ENTRY(node, struct Driver, driver_link);

        if ((!strcmp(driver->drv_name, drv_name)) && (drv_type == driver->driver_type)) {
            return driver;
        }
    }

    KPrintf("VirtioDriverFind cannot find the %s driver.return NULL\n", drv_name);
    return NONE;
}

int VirtioDriverRegister(struct Driver *driver)
{
    NULL_PARAM_CHECK(driver);

    x_err_t ret = EOK;
    static x_bool driver_link_flag = RET_FALSE;

    if (!driver_link_flag) {
        VirtioDrvLinkInit();
        driver_link_flag = RET_TRUE;
    }

    DoubleLinkListInsertNodeAfter(&virtiodrv_linklist, &(driver->driver_link));

    return ret;
}
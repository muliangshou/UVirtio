#include <virtio.h>
#include <virtio_mmio.h>
#include <stdbool.h>

struct VirtioCap blk_caps[] = {
	{ "VIRTIO_BLK_F_SIZE_MAX", 1, false,
	  "Maximum size of any single segment is in size_max." },
	{ "VIRTIO_BLK_F_SEG_MAX", 2, false,
	  "Maximum number of segments in a request is in seg_max." },
	{ "VIRTIO_BLK_F_GEOMETRY", 4, false,
	  "Disk-style geometry specified in geometry." },
	{ "VIRTIO_BLK_F_RO", 5, false, "Device is read-only." },
	{ "VIRTIO_BLK_F_BLK_SIZE", 6, false,
	  "Block size of disk is in blk_size." },
	{ "VIRTIO_BLK_F_FLUSH", 9, false, "Cache flush command support." },
	{ "VIRTIO_BLK_F_TOPOLOGY", 10, false,
	  "Device exports information on optimal I/O alignment." },
	{ "VIRTIO_BLK_F_CONFIG_WCE", 11, false,
	  "Device can toggle its cache between writeback and "
	  "writethrough modes." },
	// TODO: above should be virtio serial device specific capabilities
	//		 below are virtio device shared capabilities
	VIRTIO_INDP_CAPS
};

static uint32 handler_tid;
static uint32 device_tid;
static char *block;

/************************************
 *      Static Functions	        *
 ************************************/
static void VirtioBlkSubmit(struct VirtioHardwareDevice *vdev, struct VirtioBlkReq *req);
static void VirtioBlkHandleUsed(struct VirtioHardwareDevice *vdev, uint32 idx);
static void VirtioBlkIsr(void *param);
static void VirtioBlkDeviceHandleReq(void *param);

/************************************
 *      Functions for drivers       *
 ************************************/
/**
 * VirtioBlkSubmit
 * Desricption:
 *      Submit a block request to the device.
 * @vdev: virtio block device
 * @req: virtio block request to sumbit
 */
static void VirtioBlkSubmit(struct VirtioHardwareDevice *vdev, struct VirtioBlkReq *req)
{
	uint32 desc1, desc2, desc3, datamode = 0;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_BLK);

	if (req->type == VIRTIO_BLK_T_IN) {
		datamode = VRING_DESC_F_WRITE; /* mark page writeable */
	} 

	// Fill block request header
	desc1 = VqAllocDesc(vq, (void *)req, VIRTIO_BLK_REQ_HEADER_SIZE);
	vq->vr.desc[desc1].flags = VRING_DESC_F_NEXT;

	// Fill block reqeust sector
	desc2 = VqAllocDesc(vq, (void *)req->data, VIRTIO_BLK_SECTOR_SIZE);
	vq->vr.desc[desc2].flags = datamode | VRING_DESC_F_NEXT;

	// Fill block request footer (status)
	desc3 = VqAllocDesc(vq, (void *)req + VIRTIO_BLK_REQ_HEADER_SIZE,
						  VIRTIO_BLK_REQ_FOOTER_SIZE);
	vq->vr.desc[desc3].flags = VRING_DESC_F_WRITE;

	vq->vr.desc[desc1].next = desc2;
	vq->vr.desc[desc2].next = desc3;

	// Add to avail ring
	VqAddAvail(vq, desc1);
	WRITE_ONCE(vdev->regs->QueueNotify, 0);
}

/**
 * VirtioBlkHandleUsed
 * Description:
 * 		Handle virtio blk used event from backend
 * @vdev: virtio block device
 * @idx: desc index
 */
static void VirtioBlkHandleUsed(struct VirtioHardwareDevice *vdev, uint32 idx)
{
	VirtqueueType vq;
	uint32_t desc1, desc2, desc3;
	struct VirtioBlkReq *req;

	vq = VirtioDevFindVq(vdev, VQ_BLK);

	desc1 = vq->vr.used->ring[idx].id;
	if (!(vq->vr.desc[desc1].flags & VRING_DESC_F_NEXT)){
		KPrintf("virtio-blk received malformed descriptors\n");
		return;
	}

	desc2 = vq->vr.desc[desc1].next;
	if (!(vq->vr.desc[desc2].flags & VRING_DESC_F_NEXT)){
		KPrintf("virtio-blk received malformed descriptors\n");
		return;
	}

	desc3 = vq->vr.desc[desc2].next;
	if (vq->vr.desc[desc1].len != VIRTIO_BLK_REQ_HEADER_SIZE ||
	    vq->vr.desc[desc2].len != VIRTIO_BLK_SECTOR_SIZE ||
	    vq->vr.desc[desc3].len != VIRTIO_BLK_REQ_FOOTER_SIZE){
		KPrintf("virtio-blk received malformed descriptors\n");
		return;
	}

	req = (struct VirtioBlkReq *)vq->vr.desc[desc1].addr;

	VqFreeDesc(vq, desc1);
	VqFreeDesc(vq, desc2);
	VqFreeDesc(vq, desc3);

	// TODO: maybe should abandon semaphore of device to signal that read is done
	return;
}

/**
 * VirtioBlkIsr
 * Description:
 * 		handle virtio block device interrupt by device
 * @param: Virtio Block Device
 */
static void VirtioBlkIsr(void *param)
{
	int i, len;
	uint32 stat;
	x_base lock;
	struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)param;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_BLK);

	while(1) {
		lock = CriticalAreaLock();
		if (vq->last_used_idx == vq->vr.used->idx) {
			CriticalAreaUnLock(lock);
			KSemaphoreObtain(vq->used_sem, WAITING_FOREVER);
			continue;
		}
		// if (EOK == KSemaphoreObtain(vq->used_sem, WAITING_FOREVER)) {
		// TODO: maybe should use semaphore
		stat = READ_ONCE(vdev->regs->InterruptStatus);
		if (stat != VIRTIO_USED_RING_UPDATE){
			KPrintf("VirtioBlkIsr: interrupt status wrong! get 0x%lx should be 0x%lx",
					stat, VIRTIO_USED_RING_UPDATE);
			return;
		}
		WRITE_ONCE(vdev->regs->InterruptACK, stat);

		for (i = vq->last_used_idx; i != vq->vr.used->idx; i = (i+1) & ~(vq->num_total)) {
			VirtioBlkHandleUsed(vdev, i);
		}
		vq->last_used_idx = vq->vr.used->idx;
		// }

		KPrintf("VirtioBlkDevice: handle used event\n");
		// Finish handle isr, awaken blk req
		CriticalAreaUnLock(lock);

		KSemaphoreAbandon(vdev->haldev.dev_sem);
	}
}

/**
 * VirtioBlkRead
 * Description:
 * 		read interface of virtio block device for upper apps
 * @vdev:			the virtio device object of the driver
 * @read_param:		structure of read request
 */
int VirtioBlkRead(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param)
{
	NULL_PARAM_CHECK(vdev);
    NULL_PARAM_CHECK(read_param);

	x_base lock;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_BLK);
    char *read_buffer = (char *)x_malloc(VIRTIO_BLK_SECTOR_SIZE);
    x_size_t read_length = read_param->size;
	struct VirtioBlkConfig *config = (struct VirtioBlkConfig *)&vdev->regs->Config;
	uint32 features = READ_ONCE(vdev->regs->DriverFeatures);

	// FIXME: not malloc data size
	struct VirtioBlkReq *req = (struct VirtioBlkReq *)x_malloc(sizeof(struct VirtioBlkReq));

	req->type = VIRTIO_BLK_T_IN;		// Marked as write request
	req->sector = 0; 					// FIXME: not support sector yet
	req->data = read_buffer;
	memset(read_buffer, 0, VIRTIO_BLK_SECTOR_SIZE);

	// should not submit a request to read or write beyond capacity
	if (read_length > config->capacity) {
		KPrintf("VirtioBlkRead: read beyond capacity!\n");
		return ERROR;
	}

	// should not submit a request to read or write beyond sector number
	if (features & (1 << VIRTIO_BLK_F_SEG_MAX) && req->sector > config->seg_max) {
		KPrintf("VirtioBlkRead: read beyond max number segment!\n");
		return ERROR;
	}

	// FIXME: currently not support no notify mode
	if (!handler_tid) {
		handler_tid = KTaskCreate("VirtioBlkDriverHandler",
                           VirtioBlkIsr, vdev,
                           512, SHELL_TASK_PRIORITY);
		StartupKTask(handler_tid);
	};

	// Create used handle task & device put task
	if (!device_tid) {
		device_tid = KTaskCreate("VirtioBlkDevice",
                           VirtioBlkDeviceHandleReq, vdev,
                           512, SHELL_TASK_PRIORITY);
		StartupKTask(device_tid);
	}

	// Submit the request
	lock = CriticalAreaLock();
	VirtioBlkSubmit(vdev, req);

	CriticalAreaUnLock(lock);

	// NOTIFY the device
	KSemaphoreAbandon(vq->avail_sem);
	KSemaphoreObtain(vdev->haldev.dev_sem, WAITING_FOREVER);

	memcpy(read_param->buffer, read_buffer, read_length);
	read_param->read_length = read_length;
	x_free(read_buffer);
	x_free(req);

    return EOK;
}

/**
 * VirtioBlkWrite
 * Description:
 * 		write interface of virtio block device for upper apps
 * @vdev:			the virtio device object of the driver
 * @write_param:	structure of write request
 */
int VirtioBlkWrite(struct VirtioHardwareDevice *vdev, struct BusBlockWriteParam *write_param)
{
	NULL_PARAM_CHECK(vdev);
    NULL_PARAM_CHECK(write_param);

	x_base lock;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_BLK);
    const char *write_data = (const char *)write_param->buffer;
    x_size_t write_length = write_param->size;
	struct VirtioBlkConfig *config = (struct VirtioBlkConfig *)&vdev->regs->Config;
	uint32 features = READ_ONCE(vdev->regs->DriverFeatures);

	// FIXME: not malloc data size
	struct VirtioBlkReq *req = (struct VirtioBlkReq *)x_malloc(sizeof(struct VirtioBlkReq));

	req->type = VIRTIO_BLK_T_OUT;		// Marked as write request
	req->sector = 0; 					// FIXME: not support sector yet
	req->data = (char *)write_data;
	
	// should not submit a request to read or write beyond capacity
	if (write_length > config->capacity) {
		KPrintf("VirtioBlkRead: write beyond capacity!\n");
		// FIXME: currently no capacity information
		return ERROR;
	}

	// should not submit a request to read or write beyond sector number
	if (features & (1 << VIRTIO_BLK_F_SEG_MAX) && req->sector > config->seg_max) {
		KPrintf("VirtioBlkRead: write beyond max number segment!\n");
		return ERROR;
	}

	// should not write when VIRTIO_BLK_F_RO is set
	if (features & (1 << VIRTIO_BLK_F_RO)) {
		KPrintf("VirtioBlkRead: write to READ-ONLY block device!\n");
		return ERROR;
	}

	// FIXME: currently not support no notify mode
	if (!handler_tid) {
		handler_tid = KTaskCreate("VirtioBlkDriverHandler",
                           VirtioBlkIsr, vdev,
                           512, SHELL_TASK_PRIORITY-2);
		StartupKTask(handler_tid);
	}

	// Create used handle task & device put task
	if (!device_tid) {
		device_tid = KTaskCreate("VirtioBlkDevice",
                           VirtioBlkDeviceHandleReq, vdev,
                           512, SHELL_TASK_PRIORITY-1);
		StartupKTask(device_tid);
	}

	lock = CriticalAreaLock();
	// Submit the request
	VirtioBlkSubmit(vdev, req);

	CriticalAreaUnLock(lock);

	KSemaphoreAbandon(vq->avail_sem);
	// NOTIFY the device
	KSemaphoreObtain(vdev->haldev.dev_sem, WAITING_FOREVER);

	x_free(req);

    return EOK;
}

/**
 * VirtioBlkInit
 * Description:
 * 		Virtio block device specific initialization procedure
 * @vdev: virtio block device to be initialized
 */
int VirtioBlkInit(struct VirtioHardwareDevice *vdev)
{
    // NOTE: actually, we have no virtio hardware support now, so
    //          maybe this initialization procedure is a dummy and
    //          currently useless
    // NOTE: some device initialization is done in board specific way
    //          so here do some other stuff and is not MUST DONE maybe
    struct Virtqueue *vq;
    VirtioRegs *regs = vdev->regs;

    // Step 8: feature negotiation
    VirtioFeaturesSelect(regs, blk_caps, nelem(blk_caps), "virtio-blk");

    // Step 9: feature negotiation OK
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_FEATURES_OK);
	mb();
	if (!(regs->Status & VIRTIO_STATUS_FEATURES_OK)) {
		KPrintf("VirtioBlkInit: virtio-blk did not accept our features\n");
		return -ERROR;
	}

    // Step 10: device specific initialization e.g. virtqueue 
    // TODO: implemetation needs change and parameter is temporary
    if (VirtqueueAlloc(vdev, 0, 16, VQ_BLK) != 0) {
        KPrintf("VirtioBlkInit: virtqueue allocation error\n");
		return -ERROR;
    }

    // TODO: some initialization stuff
	// FIXME: actually the Config field is set by the hardware device
	// 		 thus we can get some information from it. But depends on
	//		 features selected by regs->DeviceFeatures
    // vdev->private_data = (void *)&regs->Config;
    // TODO: since there is no block device in xiuos here omit some initialization
	handler_tid = 0;
	device_tid = 0;
	block = (char *)x_malloc(512);
	memset(block, 'a', 512);

    // Step 11: driver ok
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_DRIVER_OK);
	mb();

    return EOK;
}

/************************************
 *      Functions for devices       *
 ************************************/
/**
 * VirtioBlkDeviceHandleReq
 * Description:
 * 		fuction act as a virtio device backend to 
 * 		read from virtqueue and use the 'real' device
 * @param: virtio block device
 */
static void VirtioBlkDeviceHandleReq(void *param)
{
	// TODO: maybe this information should be read from device registers
	// TODO: maybe need some signal mechanism
	uint32 i;
	uint32 desc1, desc2, desc3;
	struct VirtioBlkReq *req;
	uint16 avail_flags;
	uint16 used_flags;
	uint32 stat;
	x_base lock;
	struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)param;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_BLK);

	used_flags = vq->vr.used->flags;

	if (used_flags & VRING_USED_F_NO_NOTIFY) {
		// no notify mechanism, device should polling maybe
	} else {
		while(1) {
			lock = CriticalAreaLock();
			if (vq->num_added == 0) {
				CriticalAreaUnLock(lock);
				KSemaphoreObtain(vq->avail_sem, WAITING_FOREVER);
				continue;
			}
		
		// if (EOK == KSemaphoreObtain(vq->avail_sem, WAITING_FOREVER)){
			stat = READ_ONCE(vdev->regs->QueueNotify);
			if (stat != vq->idx){
				KPrintf("VirtioSerialDevicePutChar: QueueNotify status wrong! get %d should be %d",
						stat, vq->idx);
				KPrintf("Should NOT get here!\n");
				return;
			}

			for (i = vq->last_avail_idx; i != vq->vr.avail->idx; i = (i+1) & ~(vq->num_total)) {
				avail_flags = vq->vr.avail->flags;

				// GET descs and check their form
				desc1 = vq->vr.avail->ring[i];
				if (!(vq->vr.desc[desc1].flags & VRING_DESC_F_NEXT)) {
					KPrintf("virtio-blk received malformed descriptors\n");
					return;
				}

				desc2 = vq->vr.desc[desc1].next;
				if (!(vq->vr.desc[desc2].flags & VRING_DESC_F_NEXT)) {
					KPrintf("virtio-blk received malformed descriptors\n");
					return;
				}

				desc3 = vq->vr.desc[desc2].next;
				if (vq->vr.desc[desc1].len != VIRTIO_BLK_REQ_HEADER_SIZE ||
					vq->vr.desc[desc2].len != VIRTIO_BLK_SECTOR_SIZE ||
					vq->vr.desc[desc3].len != VIRTIO_BLK_REQ_FOOTER_SIZE ||
					vq->vr.desc[desc3].flags != VRING_DESC_F_WRITE) {
					KPrintf("virtio-blk received malformed descriptors\n");
					return;
				}

				req = (struct VirtioBlkReq *)vq->vr.desc[desc1].addr;
				
				// TODO: write to block device according to req type
				switch (req->type) {
				case VIRTIO_BLK_T_IN:
					// read request
					// FIXME: currently not block device and its operations
					memcpy(req->data, block, VIRTIO_BLK_SECTOR_SIZE);
					KPrintf("VirtioBlkDevice: block read dummy\n");
					break;
				case VIRTIO_BLK_T_OUT:
					// write request
					// TODO: if VIRTIO_BLK_F_RO is set, set status to VIRTIO_BLK_S_IOERR and return
					// FIXME: currently not block device and its operations
					memcpy(block, req->data, VIRTIO_BLK_SECTOR_SIZE);
					KPrintf("VirtioBlkDevice: block write dummy\n");
					// FIXME: for debug use
					vdev->hwdev_done->put_char(vdev, '#');
					break;
				case VIRTIO_BLK_T_FLUSH:
					// flush request
					// FIXME: currently not block device and its operations
					KPrintf("VirtioBlkDevice: block flush dummy\n");
					break;
				default:
					KPrintf("VirtioDeviceHandleReq: Unhandled status in virtio_blk irq\n");
					// TODO: IOERR
					WRITE_ONCE(req->status, VIRTIO_BLK_S_IOERR);
					return;
				}
				
				// Put used desc into used ring
				VqAddUsed(vq, desc1, 3);

				// TODO: the device should interrupt the driver according to vr_avails flag 
				//		 and VIRTIO_F_EVENT_IDX bit
				if (avail_flags & ~VRING_AVAIL_F_NO_INTERRUPT) {
					// TODO: interrupt the driver
				}
			}
			vq->last_avail_idx = vq->vr.avail->idx;
			vq->num_added = 0;

			// Interrupt driver
			WRITE_ONCE(vdev->regs->InterruptStatus, VIRTIO_USED_RING_UPDATE);
			// KSemaphoreAbandon(vq->used_sem);
			CriticalAreaUnLock(lock);

			KSemaphoreAbandon(vq->used_sem);
		// }
		}
	}
}
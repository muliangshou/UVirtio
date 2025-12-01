#include <virtio.h>
#include <virtio_mmio.h>

// #define DEBUG_TEST_1
// #define DEBUG_TEST_3
// #define DEBUG_TEST_4

struct VirtioCap serial_caps[] = {
	// TODO: above should be virtio serial device specific capabilities
	//		 below are virtio device shared capabilities
	VIRTIO_INDP_CAPS
};

static int32 handler_tid;
static int32 device_tid;

#ifdef DEBUG_TEST_2
static int32 t1 = 0;
static int32 t2 = 0;
static int32 t3 = 0;
#endif

/************************************
 *      Static Functions	        *
 ************************************/

static void VirtioSerialPutChar(struct VirtioHardwareDevice *vdev, const char* ch);
static void VirtioSerialGetChar(struct VirtioHardwareDevice *vdev, char* ch);
static void VirtioSerialHandleUsed(VirtqueueType vq, uint32 idx);
static void VirtioSerialTxUsed(void *param);
static void VirtioSerialDevicePutChar(void *param);

/************************************
 *      Functions for drivers       *
 ************************************/
/**
 * VirtioSerialPutChar
 * Description:
 * 		Virtio serial driver function to create a desc in vq of put char request
 * @vdev:	the virtio device object of the driver
 * @ch:		pointer to the char to put	
 */
static void VirtioSerialPutChar(struct VirtioHardwareDevice *vdev, const char* ch)
{
	x_base lock;
    uint32 desc = 0;
    VirtqueueType vq = VirtioDevFindVq(vdev, VQ_SERIAL_TX);
    
	lock = CriticalAreaLock();
#ifdef DEBUG_TEST_2
	unsigned long start_time_1, end_time_1;
	start_time_1 = virtio_get_time();
#endif	
	// FIXME: set to 1 to avoid idx overlap
	while (vq->num_free == 1) {
#ifdef DEBUG_TEST_2
		end_time_1 = virtio_get_time();
		t1 += end_time_1 - start_time_1;
		KPrintf("1 start schedule: %ld\n", end_time_1);
#endif	
#ifndef DEBUG_TEST_3
		if (vq->num_added != 0){
			CriticalAreaUnLock(lock);
			KSemaphoreSetValue(vq->avail_sem, 0);
			KSemaphoreAbandon(vq->avail_sem);
		} else {
			CriticalAreaUnLock(lock);
			KSemaphoreSetValue(vq->used_sem, 0);
			KSemaphoreAbandon(vq->used_sem);
		}
		
		lock = CriticalAreaLock();
#endif
#ifdef DEBUG_TEST_4
		VirtioSerialDevicePutChar(vdev);
#endif
#ifdef DEBUG_TEST_2
		start_time_1 = virtio_get_time();
		KPrintf("1 end schedule: %ld\n", start_time_1);
#endif	
	}
	desc = VqAllocDesc(vq, (void *)ch, 1);
	VqAddAvail(vq, desc);

    // Notify
    WRITE_ONCE(vdev->regs->QueueNotify, 0);
#ifdef DEBUG_TEST_3
#ifndef DEBUG_TEST_4
		VirtioSerialDevicePutChar(vdev);
#endif
#endif
#ifdef DEBUG_TEST_2
		end_time_1 = virtio_get_time();
		t1 += end_time_1 - start_time_1;
#endif	
	CriticalAreaUnLock(lock);
}

/**
 * VirtioSerialGetChar
 * Description:
 * 		Virtio serial driver function to create a desc in vq of get char request
 * @vdev:	the virtio device object of the driver
 * @ch:		pointer to the char buffer to get into	
 */
static void VirtioSerialGetChar(struct VirtioHardwareDevice *vdev, char* ch)
{
    uint32 desc = 0;
    VirtqueueType vq = VirtioDevFindVq(vdev, VQ_SERIAL_RX);
    
    desc = VqAllocDesc(vq, (void *)ch, 1);
    VqAddAvail(vq, desc);
    // Notify
    WRITE_ONCE(vdev->regs->QueueNotify, 1);
}


/**
 * VirtioSerialHandleUsed
 * Description:
 * 		virtio fronted funtion to process used desc of virtio device
 * @vq:		the vq of the used ring
 * @idx:	the index of the used desc entry
 */
static void VirtioSerialHandleUsed(VirtqueueType vq, uint32 idx)
{
	// TODO: some freeing work but here nothing need to be free
	uint32 desc = vq->vr.used->ring[idx].id;
	VqFreeDesc(vq, desc);
}

/**
 * VirtioSerialTxUsed
 * Description:
 * 		handle function of used event from virtio device backend
 * @param: virtio serial device
 */
static void VirtioSerialTxUsed(void *param)
{
#ifdef DEBUG_TEST_2
	unsigned long start_time_3, end_time_3;
	start_time_3 = virtio_get_time();
#endif
	uint32 i;
	VirtqueueType tx;
	uint32 stat;
	uint32 cnt;
	struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)param;
	x_base lock;

	tx = VirtioDevFindVq(vdev, VQ_SERIAL_TX);

#ifdef DEBUG_TEST_2
	end_time_3 = virtio_get_time();
	t3 += end_time_3 - start_time_3;
	KPrintf("3 start schedule: %ld\n", end_time_3);
#endif
#ifdef DEBUG_TEST_2
	start_time_3 = virtio_get_time();
	KPrintf("3 end schedule: %ld\n", start_time_3);
#endif
	// NOTE: here should obtain semaphore of the used ring or pending
	while (1) {
		lock = CriticalAreaLock();
		if (tx->last_used_idx == tx->vr.used->idx) {
#ifdef DEBUG_TEST_2
			end_time_3 = virtio_get_time();
			t3 += end_time_3 - start_time_3;
			KPrintf("3 start schedule: %ld\n", end_time_3);
#endif
#ifndef DEBUG_TEST_3
			CriticalAreaUnLock(lock);
			KSemaphoreObtain(tx->used_sem, WAITING_FOREVER);
#endif
#ifdef DEBUG_TEST_2
			start_time_3 = virtio_get_time();
			KPrintf("3 end schedule: %ld\n", start_time_3);
#endif
			continue;
		}
		// Interrupt status acknowledge
		// FIXME: seems useless since only checked once and not rewind
		stat = READ_ONCE(vdev->regs->InterruptStatus);
		if (stat != VIRTIO_USED_RING_UPDATE){
			KPrintf("VirtioSerialTxUsed: interrupt status wrong! get 0x%lx should be 0x%lx",
					stat, VIRTIO_USED_RING_UPDATE);
			CriticalAreaUnLock(lock);
			return;
		}
		WRITE_ONCE(vdev->regs->InterruptACK, stat);

		// Vq process
		for (i = tx->last_used_idx; i != tx->vr.used->idx; i = (i+1) & ~(tx->num_total)) {
			VirtioSerialHandleUsed(tx, i);
		}

		tx->last_used_idx = tx->vr.used->idx;

		CriticalAreaUnLock(lock);
		KSemaphoreSetValue(vdev->haldev.dev_sem, 0);
		KSemaphoreAbandon(vdev->haldev.dev_sem);
#ifdef DEBUG_TEST_2
		end_time_3 = virtio_get_time();
		t3 += end_time_3 - start_time_3;
		KPrintf("time in handler: %ld\n", t3);
#endif
	}
}

/**
 * VirtioSerialWrite
 * Description:
 * 		write interface of virtio serial device for upper apps
 * @vdev:			the virtio device object of the driver
 * @write_param:	structure of write request
 */
int VirtioSerialWrite(struct VirtioHardwareDevice *vdev, struct BusBlockWriteParam *write_param)
{
	NULL_PARAM_CHECK(vdev);
    NULL_PARAM_CHECK(write_param);

#ifdef DEBUG_TEST_2
	unsigned long start_time_1, end_time_1;
	start_time_1 = virtio_get_time();
#endif

	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_SERIAL_TX);
    const char *write_data = (const char *)write_param->buffer;
    x_size_t write_length = write_param->size;

#ifndef DEBUG_TEST_3
	// FIXME: currently not support no notify mode
	if (!handler_tid) {
		handler_tid = KTaskCreate("VirtioSerialDriver Handler",
                           VirtioSerialTxUsed, vdev,
                           512, SHELL_TASK_PRIORITY);
		StartupKTask(handler_tid);
	}
	// Create used handle task & device put task
	if (!device_tid) {
		device_tid = KTaskCreate("VirtioSerialDevice PutChar",
                           VirtioSerialDevicePutChar, vdev,
                           512, SHELL_TASK_PRIORITY);
		StartupKTask(device_tid);
	}
#endif
	// insert request into virtqueue
	// NOTE: use batching, which means insert all chars into vq and wait for
	//		 device to process
    while (write_length)
    {
#ifdef DEBUG_TEST_2
		end_time_1 = virtio_get_time();
		t1 += end_time_1 - start_time_1;
#endif
		VirtioSerialPutChar(vdev, write_data);
#ifdef DEBUG_TEST_2
		start_time_1 = virtio_get_time();
#endif
        // KPrintf("VirtioSerialWrite data %c write_length %u\n", *(char *)write_data, write_length);
        write_data++; 
        write_length--;
    }

	// NOTIFY the device
	KSemaphoreSetValue(vq->avail_sem, 0);
	KSemaphoreAbandon(vq->avail_sem);
	KSemaphoreObtain(vdev->haldev.dev_sem, WAITING_FOREVER);
#ifdef DEBUG_TEST_2
	KPrintf("time used in front end: %ld\n", t1);
#endif
    return EOK;
}

/**
 * VirtioSerialRead
 * Description:
 * 		read interface of virtio serial device for upper apps
 * @vdev:		the virtio device object of the driver
 * @read_param:	structure of read request
 */
int VirtioSerialRead(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param)
{
	NULL_PARAM_CHECK(vdev);
    NULL_PARAM_CHECK(read_param);

	uint32 i;
	uint32 stat;
	uint32 desc;
	uint16 used_flags;
	uint32 addr;
	uint32 len;
	x_base lock;
	VirtqueueType rx = VirtioDevFindVq(vdev, VQ_SERIAL_RX);
    uint8 *read_data = (uint8 *)read_param->buffer;
	// NOTE: original serial read may be wrong cuz read_param->size should be 
	//		 assigned with the size of read buffer
    x_size_t read_size = read_param->size;

	// NOTE: wait for vq rx's semephore, signals that device has read
	while (read_param->read_length < read_size){
		if (EOK == KSemaphoreObtain(rx->used_sem, 10000)) {
			// Interrupt status acknowledge
			lock = CriticalAreaLock();
			stat = READ_ONCE(vdev->regs->InterruptStatus);
			if (stat != VIRTIO_USED_RING_UPDATE){
				KPrintf("VirtioSerialRead: interrupt status wrong! get 0x%lx should be 0x%lx",
						stat, VIRTIO_USED_RING_UPDATE);
				CriticalAreaUnLock(lock);
				return -ERROR;
			}
			WRITE_ONCE(vdev->regs->InterruptACK, stat);

			// Vq process
			for (i = rx->last_used_idx; i != rx->vr.used->idx && read_param->read_length < read_size; i = (i+1) & ~(rx->num_total)) {
				// Get data from used desc
				desc = rx->vr.used->ring[i].id;
				used_flags = rx->vr.used->flags;
				addr = rx->vr.desc[desc].addr;
				len = rx->vr.desc[desc].len;

				// Read from vring to read buffer
				// FIXME: buffer may have a limit length and read_size may be various
				read_data[read_param->read_length] = *(char *)addr;
				read_param->read_length = read_param->read_length + 1;

				// NOTE: may just add the desc back into avail ring, no need to free
				// FIXME: critical area
				VqAddAvail(rx, desc);
				// TODO: may need semaphore
			}

			// NOTE: i may not equals to rx->vr.used->idx
			rx->last_used_idx = i;
			CriticalAreaUnLock(lock);
		} else {
			KPrintf("VirtioSerialRead: timeout\n");
			break;
		}
	}
    return EOK;
}

/**
 * VirtioSerialInit
 * Description:
 * 		Virtio serial device specific initialization procedure
 * @vdev: virtio serial device to be initialized
 */
int VirtioSerialInit(struct VirtioHardwareDevice *vdev)
{
    // NOTE: actually, we have no virtio hardware support now, so
    //          maybe this initialization procedure is a dummy and
    //          currently useless
    // NOTE: some device initialization is done in board specific way
    //          so here do some other stuff and is not MUST DONE maybe
    struct Virtqueue *vq;
    VirtioRegs *regs = vdev->regs;
	uint32 rx_size = 16;
	uint32 tx_size = 16;

    // Step 8: feature negotiation
    VirtioFeaturesSelect(regs, serial_caps, nelem(serial_caps), "virtio-serial");

    // Step 9: feature negotiation OK
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_FEATURES_OK);
	mb();
	if (!(regs->Status & VIRTIO_STATUS_FEATURES_OK)) {
		KPrintf("VirtioSerialInit: virtio-serial did not accept our features\n");
		return -ERROR;
	}

    // Step 10: device specific initialization e.g. virtqueue 
    // TODO: implemetation needs change and parameter is temporary
    if (VirtqueueAlloc(vdev, 0, tx_size, VQ_SERIAL_TX) != 0) {
        KPrintf("VirtioSerialInit: virtqueue allocation error\n");
		return -ERROR;
    }

	// NOTE: create vq to receive bytes
	if (VirtqueueAlloc(vdev, 1, rx_size, VQ_SERIAL_RX) != 0) {
        KPrintf("VirtioSerialInit: virtqueue allocation error\n");
		return -ERROR;
    }
	// Virtio serial device rx buffer Init;
	// TODO: virtio device maybe further divided and the buffer should be in vdev
	char *rx_buffer = (char *)x_malloc(rx_size * sizeof(char));
	// vdev->private_data = (void *)rx_buffer;
	for (int i = 0; i < rx_size; i ++) {
		VirtioSerialGetChar(vdev, &rx_buffer[i]);
	}

    // TODO: some initialization stuff
	handler_tid = 0;
	device_tid = 0;

    // Step 11: driver ok
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_DRIVER_OK);
	mb();

    return EOK;
}

/************************************
 *      Functions for devices       *
 ************************************/
static int cnt = 0;
/**
 * VirtioSerialDevicePutChar
 * Description:
 * 		put char fuction act as a virtio device backend to 
 * 		read from virtqueue and use the 'real' device
 * @param: virtio serial device
 */
static void VirtioSerialDevicePutChar(void *param)
{
#ifdef DEBUG_TEST_2
	unsigned long start_time_2, end_time_2;
	start_time_2 = virtio_get_time();
#endif
	// TODO: maybe this information should be read from device registers
	// TODO: maybe need some signal mechanism
	uint32 i;
	uint32 desc;
	uint16 avail_flags;
	uint16 used_flags;
	uint32 addr;
	uint32 len;
	uint32 stat;
	char ch;
	uint32 write_length;
	x_base lock;
	struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)param;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_SERIAL_TX);

	used_flags = vq->vr.used->flags;

#ifdef DEBUG_TEST_2
	end_time_2 = virtio_get_time();
	t2 += end_time_2 - start_time_2;
	KPrintf("2 start schedule: %ld\n", end_time_2);
#endif
#ifdef DEBUG_TEST_2
	start_time_2 = virtio_get_time();
	KPrintf("2 end schedule: %ld\n", start_time_2);
#endif

	if (used_flags & VRING_USED_F_NO_NOTIFY) {
		// no notify mechanism, device should polling maybe
	} else {
		while (1) {
			lock = CriticalAreaLock();
			if (vq->num_added == 0) {
#ifdef DEBUG_TEST_2
				end_time_2 = virtio_get_time();
				t2 += end_time_2 - start_time_2;
				KPrintf("2 start schedule: %ld\n", end_time_2);
#endif
#ifndef DEBUG_TEST_3
				CriticalAreaUnLock(lock);
				KSemaphoreObtain(vq->avail_sem, WAITING_FOREVER);
#endif
#ifdef DEBUG_TEST_4
				break;
#endif
#ifdef DEBUG_TEST_2
				start_time_2 = virtio_get_time();
				KPrintf("2 end schedule: %ld\n", start_time_2);
#endif
				continue;
			}
			stat = READ_ONCE(vdev->regs->QueueNotify);
			if (stat != vq->idx){
				KPrintf("VirtioSerialDevicePutChar: QueueNotify status wrong! get %d should be %d",
						stat, vq->idx);
				KPrintf("Should NOT get here!\n");
				return;
			}
#ifdef DEBUG_TEST_1
			write_length = 0;
#endif
			for (i = vq->last_avail_idx; i != vq->vr.avail->idx; i = (i+1) & ~(vq->num_total)) {
				desc = vq->vr.avail->ring[i];
				avail_flags = vq->vr.avail->flags;
				addr = vq->vr.desc[desc].addr;
				len = vq->vr.desc[desc].len;

				ch = *(char *)addr;
				
				// TODO: may need semaphore
				// NOTE: put char by physical hardware function
				vdev->hwdev_done->put_char(vdev, ch);
#ifdef DEBUG_TEST_1
				write_length++;
#endif
				// Put used desc into used ring
				VqAddUsed(vq, desc, 1);
				
				if (vq->vr.used->idx == vq->last_used_idx) {
					// NOTE: used ring is already full, older data will be rewrite
					vq->last_used_idx = (vq->last_used_idx + 1) &  ~(vq->num_total);
					// TODO: may need a flag of used ring full
				}

				// TODO: the device should interrupt the driver according to vr_avails flag 
				//		 and VIRTIO_F_EVENT_IDX bit
				if (avail_flags & ~VRING_AVAIL_F_NO_INTERRUPT) {
					// TODO: interrupt the driver
				}
			}
			vq->last_avail_idx = vq->vr.avail->idx;
			vq->num_added = 0;
#ifdef DEBUG_TEST_1
			if(write_length != 0){
				cnt ++;
				KPrintf("write_lenght: %d, count = %d\n", write_length, cnt);
			}
#endif
#ifdef DEBUG_TEST_2
			end_time_2 = virtio_get_time();
			t2 += end_time_2 - start_time_2;
			KPrintf("time in backend: %ld\n", t2);
#endif
			// Interrupt driver
			WRITE_ONCE(vdev->regs->InterruptStatus, VIRTIO_USED_RING_UPDATE);
#ifdef DEBUG_TEST_3
			VirtioSerialTxUsed(vdev);
			break;
#endif
			CriticalAreaUnLock(lock);
			KSemaphoreSetValue(vq->used_sem, 0);
			KSemaphoreAbandon(vq->used_sem);
		}
	}
}

/**
 * VirtioSerialDeviceGetChar
 * Description:
 * 		get char fuction act as a virtio device backend to 
 * 		read from device and write into virtqueue
 * @vdev: virtio serial device
 * @event: interrupt event id
 */
void VirtioSerialDeviceGetChar(struct VirtioHardwareDevice *vdev, int event)
{
	// NOTE: this may should be the callback handler of device's get char interrupt
	uint32 i;
	uint32 desc;
	uint16 avail_flags;
	uint16 used_flags;
	uint32 addr;
	uint32 len;
	uint32 stat;
	x_base lock;
	int ch;
	uint32 rx_length = 0;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_SERIAL_RX);

	used_flags = vq->vr.used->flags;

	// TODO: event is not checked
#ifdef DEBUG_TEST_3
    unsigned long start_mtime;
        unsigned long tmp = get_timer_value();
    do {
        start_mtime = get_timer_value();
    } while (start_mtime == tmp);

    KPrintf("virtio\nstart: %ld\n", start_mtime);
#endif

	if (used_flags == ~VRING_USED_F_NO_NOTIFY) {
		// NOTE: as for get char, avail ring is always full and no need to notify
		vq->vr.used->flags = VRING_USED_F_NO_NOTIFY;
	}
	// NOTE: get char from physical hardware function and fill into desc
	while (1){
		// Keep get until an error occur
		ch = vdev->hwdev_done->get_char(vdev);
		if (-ERROR == ch) {
				break;
		}
		rx_length ++;

		lock = CriticalAreaLock();
		// just fetch a desc from avail ring
		i = vq->last_avail_idx;
		desc = vq->vr.avail->ring[i];
		avail_flags = vq->vr.avail->flags;
		addr = vq->vr.desc[desc].addr;
		len = vq->vr.desc[desc].len;
		
		*(uint8 *)addr = (uint8)ch;

		// Put used desc into used ring
		VqAddUsed(vq, desc, 1);
		if (vq->vr.used->idx == vq->last_used_idx) {
			vq->last_used_idx = (vq->last_used_idx + 1) &  ~(vq->num_total);
			// TODO: may need a flag of used ring full
		}

		vq->last_avail_idx = (vq->last_avail_idx + 1) &  ~(vq->num_total);
	}
	
	if (rx_length) {
		// TODO: the device should interrupt the driver according to vr_avails flag 
		//		 and VIRTIO_F_EVENT_IDX bit
		if (avail_flags & ~VRING_AVAIL_F_NO_INTERRUPT) {
			// TODO: interrupt the driver
		}
		
		// KPrintf("device read %d bytes\n", rx_length);

		// Interrupt driver
		WRITE_ONCE(vdev->regs->InterruptStatus, VIRTIO_USED_RING_UPDATE);
		// NOTE: need to reset first otherwise there may be some errors
		// 		 the times of calling this function is undecidable
		// 		 so one series of input may result in multiple calls
		// 		 of KSemaphoreAbandon
		CriticalAreaUnLock(lock);
		KSemaphoreSetValue(vq->used_sem, 0);
		KSemaphoreAbandon(vq->used_sem);
	} else {
		CriticalAreaUnLock(lock);
	}
}
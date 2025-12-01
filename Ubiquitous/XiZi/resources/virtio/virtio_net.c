#include <virtio.h>
#include <virtio_mmio.h>
#include <stdbool.h>

struct VirtioCap net_caps[] = {
	{ "VIRTIO_NET_F_CSUM", 0, true,
	  "Device handles packets with partial checksum. This “checksum "
	  "offload” is a common feature on modern network cards." },
	{ "VIRTIO_NET_F_GUEST_CSUM", 1, false,
	  "Driver handles packets with partial checksum." },
	{ "VIRTIO_NET_F_CTRL_GUEST_OFFLOADS", 2, false,
	  "Control channel offloads reconfiguration support." },
	{ "VIRTIO_NET_F_MAC", 5, true, "Device has given MAC address." },
	{ "VIRTIO_NET_F_GUEST_TSO4", 7, false, "Driver can receive TSOv4." },
	{ "VIRTIO_NET_F_GUEST_TSO6", 8, false, "Driver can receive TSOv6." },
	{ "VIRTIO_NET_F_GUEST_ECN", 9, false,
	  "Driver can receive TSO with ECN." },
	{ "VIRTIO_NET_F_GUEST_UFO", 10, false, "Driver can receive UFO." },
	{ "VIRTIO_NET_F_HOST_TSO4", 11, false, "Device can receive TSOv4." },
	{ "VIRTIO_NET_F_HOST_TSO6", 12, false, "Device can receive TSOv6." },
	{ "VIRTIO_NET_F_HOST_ECN", 13, false,
	  "Device can receive TSO with ECN." },
	{ "VIRTIO_NET_F_HOST_UFO", 14, false, "Device can receive UFO." },
	{ "VIRTIO_NET_F_MRG_RXBUF", 15, false,
	  "Driver can merge receive buffers." },
	{ "VIRTIO_NET_F_STATUS", 16, true,
	  "Configuration status field is available." },
	{ "VIRTIO_NET_F_CTRL_VQ", 17, false, "Control channel is available." },
	{ "VIRTIO_NET_F_CTRL_RX", 18, false,
	  "Control channel RX mode support." },
	{ "VIRTIO_NET_F_CTRL_VLAN", 19, false,
	  "Control channel VLAN filtering." },
	{ "VIRTIO_NET_F_GUEST_ANNOUNCE", 21, false,
	  "Driver can send gratuitous packets." },
	{ "VIRTIO_NET_F_MQ", 22, false,
	  "Device supports multiqueue with automatic receive steering." },
	{ "VIRTIO_NET_F_CTRL_MAC_ADDR", 23, false,
	  "Set MAC address through control channel" },
	// TODO: above should be virtio serial device specific capabilities
	//		 below are virtio device shared capabilities
	VIRTIO_INDP_CAPS
};

static uint32 device_tid;
static uint32 tx_handler_tid;
static uint32 rx_handler_tid;

// #define DEBUG_TEST_1

/************************************
 *      Static Functions	        *
 ************************************/
static void VirtioNetTxUsed(void *param);
static void VirtioNetRxUsed(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param);
static void VirtioNetAddRxPacket(VirtqueueType vq, struct VirtioNetHdr *hdr);
static void VirtioNetAddTxPacket(VirtqueueType vq, struct VirtioNetHdr *hdr);
static void VirtioNetRxInit(struct VirtioHardwareDevice *vdev);
static void VirtioNetDeviceTx(void *param);

/************************************
 *      Functions for drivers       *
 ************************************/
/**
 * VirtioNetAddRxPacket
 * Description:
 * 		add initial packets into (receive) vq
 * @vq: vq to add packets to
 * @hdr: header of the network packet
 * @pkt: packet to add
 */
static void VirtioNetAddRxPacket(VirtqueueType vq, struct VirtioNetHdr *hdr)
{
	int i;
	uint32 desc1, desc2;
	struct VirtioNetPkt *pkt = hdr->packet;

	desc1 = VqAllocDesc(vq, (void *)hdr, VIRTIO_NET_HDRLEN);
	desc2 = VqAllocDesc(vq, (void *)pkt->data, 16);

	vq->vr.desc[desc1].flags = VRING_DESC_F_WRITE | VRING_DESC_F_NEXT;
	vq->vr.desc[desc1].next = desc2;

	vq->vr.desc[desc2].flags = VRING_DESC_F_WRITE;
	
	VqAddAvail(vq, desc1);
	WRITE_ONCE(vq->dev->regs->QueueNotify, 0);
}

/**
 * VirtioNetAddTxPacket
 * Description:
 * 		add initial packets into (transmit) vq
 * @vq: vq to add packets to
 * @hdr: header of the network packet
 * @pkt: packet to add
 */
static void VirtioNetAddTxPacket(VirtqueueType vq, struct VirtioNetHdr *hdr)
{
	int i;
	x_base lock;
	uint32 desc1, desc2;
	struct VirtioNetPkt *pkt = hdr->packet;

	lock = CriticalAreaLock();

	while (vq->num_free <= 2){
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
	}

	desc1 = VqAllocDesc(vq, (void *)hdr, VIRTIO_NET_HDRLEN);
	desc2 = VqAllocDesc(vq, (void *)pkt->data, 16);

	vq->vr.desc[desc1].flags = VRING_DESC_F_WRITE | VRING_DESC_F_NEXT;
	vq->vr.desc[desc1].next = desc2;

	vq->vr.desc[desc2].flags = 0;
	
	VqAddAvail(vq, desc1);
	WRITE_ONCE(vq->dev->regs->QueueNotify, 0);

	CriticalAreaUnLock(lock);
}

/**
 * VirtioNetRxInit
 * Description:
 * 		Initialize the rx vq of virtio network device
 * @vdev: virtio network device
 */
static void VirtioNetRxInit(struct VirtioHardwareDevice *vdev)
{
	int i;
	struct VirtioNetHdr *hdr;
	struct VirtioNetPkt *pkt_dummy;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_NET_RX);
	uint32 num = vq->num_total;

	for (i = 0; i < num; i += 2) {
		hdr = (struct VirtioNetHdr *)x_malloc(sizeof(struct VirtioNetHdr));
		memset(hdr, 0, sizeof(struct VirtioNetHdr));
		
		// FIXME: not actually packet yet
		// pkt = packet_alloc();
		// pkt->capacity = PACKET_CAPACITY;
		// hdr->packet = pkt;
		
		pkt_dummy = (struct VirtioNetPkt *)x_malloc(sizeof(struct VirtioNetPkt));
		memset(pkt_dummy, 0, sizeof(struct VirtioNetPkt));
		pkt_dummy->data = (char *)x_malloc(16);

		hdr->packet = pkt_dummy;

		VirtioNetAddRxPacket(vq, hdr);
	}
}

/**
 * VirtioNetTxUsed
 * Description:
 * 		handle virtio network device interrupt of the Tx vq by device
 * @param: Virtio Block Device
 */
static void VirtioNetTxUsed(void *param)
{
	int i, len;
	uint32 stat;
	x_base lock;
	struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)param;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_NET_TX);

	// TODO: maybe should use infinite loop
	while(1) {
		lock = CriticalAreaLock();
		if (vq->last_used_idx == vq->vr.used->idx) {
			CriticalAreaUnLock(lock);
			KSemaphoreObtain(vq->used_sem, WAITING_FOREVER);
			continue;
		}
		// TODO: maybe should use semaphore
		stat = READ_ONCE(vdev->regs->InterruptStatus);
		if (stat != VIRTIO_USED_RING_UPDATE){
			KPrintf("VirtioNetIsr: interrupt status wrong! get 0x%lx should be 0x%lx",
					stat, VIRTIO_USED_RING_UPDATE);
			CriticalAreaUnLock(lock);
			return;
		}
		WRITE_ONCE(vdev->regs->InterruptACK, stat);

		for (i = vq->last_used_idx; i != vq->vr.used->idx; i = (i+1) & ~(vq->num_total)) {
			uint32 desc1 = vq->vr.used->ring[i].id;
			uint32 desc2 = vq->vr.desc[desc1].next;

			// free the header and packet
			struct VirtioNetHdr *hdr = (struct VirtioNetHdr *)vq->vr.desc[desc1].addr;
			struct VirtioNetPkt *pkt = hdr->packet;

			// FIXME: should not explicitly call free, lead to double free
			// x_free(hdr);
			// x_free(pkt);

			// free the tx desc
			VqFreeDesc(vq, desc1);
			VqFreeDesc(vq, desc2);
		}
		vq->last_used_idx = vq->vr.used->idx;

		CriticalAreaUnLock(lock);
		KSemaphoreSetValue(vdev->haldev.dev_sem, 0);
		KSemaphoreAbandon(vdev->haldev.dev_sem);
	}
}

/**
 * VirtioNetRxUsed
 * Description:
 * 		handle virtio network device interrupt of the Rx vq by device
 * @param: Virtio Block Device
 */
static void VirtioNetRxUsed(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param)
{
	int i, len;
	uint32 stat;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_NET_RX);

	// TODO: maybe should use infinite loop
	KPrintf("VirtioNetRxUsed: waiting for packet...\n");
	if (EOK == KSemaphoreObtain(vq->used_sem, WAITING_FOREVER)) {
		KPrintf("VirtioNetRxUsed: get packet!\n");
		// TODO: maybe should use semaphore
		stat = READ_ONCE(vdev->regs->InterruptStatus);
		if (stat != VIRTIO_USED_RING_UPDATE){
			KPrintf("VirtioNetIsr: interrupt status wrong! get 0x%lx should be 0x%lx",
					stat, VIRTIO_USED_RING_UPDATE);
			return;
		}
		WRITE_ONCE(vdev->regs->InterruptACK, stat);

		for (i = vq->last_used_idx; i != vq->vr.used->idx; i = (i+1) & ~(vq->num_total)) {
			uint32 desc1 = vq->vr.used->ring[i].id;
			uint32 desc2 = vq->vr.desc[desc1].next;
			uint32 len = vq->vr.used->ring[i].len;

			// free the header and packet
			struct VirtioNetHdr *hdr = (struct VirtioNetHdr *)vq->vr.desc[desc1].addr;
			struct VirtioNetPkt *pkt = hdr->packet;
			pkt->data = (char *)vq->vr.desc[desc2].addr;
			
			// TODO: here should call some packet process function
			KPrintf("VirtioNetRxUsed: receivce packet\n");
			KPrintf("VirtioNetRxUsed: payload: %s\n", pkt->data);
			memcpy(read_param->buffer, pkt->data, read_param->size);

			free(pkt->data);
			free(pkt);
			
			// Add new desc into avail ring
			pkt = (struct VirtioNetPkt *)x_malloc(sizeof(struct VirtioNetPkt));
			memset(hdr, 0, sizeof(struct VirtioNetPkt));
			pkt->data = (char *)x_malloc(16);
			hdr->packet = pkt;

			vq->vr.desc[desc2].addr = (uint32)pkt->data;

			VqAddAvail(vq, desc1);
			WRITE_ONCE(vq->dev->regs->QueueNotify, 0);
		}
		vq->last_used_idx = vq->vr.used->idx;

		KPrintf("VirtioNetRxUsed: finish handling rx used event\n");
	}
}

/**
 * VirtioNetRead
 * Description:
 * 		send a virtio packet and add into vq
 * @vdev: 			virtio network device
 * @write_params: 	structure of write request
 */
int VirtioNetRead(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param)
{
	
	VirtioNetRxUsed(vdev, read_param);
    
	return EOK;
}

/**
 * VirtioNetWrite
 * Description:
 * 		send a virtio packet and add into vq
 * @vdev: 			virtio network device
 * @write_params: 	structure of write request
 */
int VirtioNetWrite(struct VirtioHardwareDevice *vdev, struct BusBlockWriteParam *write_param)
{
	NULL_PARAM_CHECK(vdev);
    NULL_PARAM_CHECK(write_param);

	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_NET_TX);
    const char *write_data = (const char *)write_param->buffer;
    x_size_t write_length = write_param->size;
	struct VirtioNetConfig *config = (struct VirtioNetConfig *)&vdev->regs->Config;
	uint32 features = READ_ONCE(vdev->regs->DriverFeatures);

	// TODO: checksum

	// form a request header
	// FIXME: not really header
	struct VirtioNetHdr *hdr = (struct VirtioNetHdr *)x_malloc(sizeof(struct VirtioNetHdr));
	memset(hdr, 0, sizeof(struct VirtioNetHdr));

	hdr->flags = 0;
	hdr->gso_type = VIRTIO_NET_HDR_GSO_NONE;
	hdr->hdr_len = 0;
	hdr->gso_size = 0;
	hdr->csum_start = 0;
	hdr->csum_offset = 0;
	hdr->num_buffers = 0xDEAD;
	
	// form a request packet
	// FIXME: not really packet
	struct VirtioNetPkt *pkt = (struct VirtioNetPkt *)x_malloc(sizeof(struct VirtioNetPkt));
	pkt->data = (char *)write_data;
	hdr->packet = pkt;

	if (!device_tid) {
		device_tid = KTaskCreate("VirtioNetDeviceTx",
                           VirtioNetDeviceTx, vdev,
                           512, SHELL_TASK_PRIORITY);
		StartupKTask(device_tid);
	};

	// Create device task to transmit and handle used 
	if (!tx_handler_tid) {
		tx_handler_tid = KTaskCreate("VirtioNetTxHandler",
                           VirtioNetTxUsed, vdev,
                           512, SHELL_TASK_PRIORITY);
		StartupKTask(tx_handler_tid);
	};
    
	// Add into tx vq
	VirtioNetAddTxPacket(vq, hdr);

	KSemaphoreSetValue(vq->avail_sem, 0);
	KSemaphoreAbandon(vq->avail_sem);
	KSemaphoreObtain(vdev->haldev.dev_sem, WAITING_FOREVER);
	// x_free(pkt);
	// x_free(hdr);

	return EOK;
}

/**
 * VirtioNetInit
 * Description:
 * 		Virtio network device specific initialization procedure
 * @vdev: virtio network device to be initialized
 */
int VirtioNetInit(struct VirtioHardwareDevice *vdev)
{
    // NOTE: actually, we have no virtio hardware support now, so
    //          maybe this initialization procedure is a dummy and
    //          currently useless
    // NOTE: some device initialization is done in board specific way
    //          so here do some other stuff and is not MUST DONE maybe
	uint16 mq = 1;
    VirtioRegs *regs = vdev->regs;
	struct VirtioNetConfig *config = (struct VirtioNetConfig *)regs->Config;

    // Step 8: feature negotiation
    VirtioFeaturesSelect(regs, net_caps, nelem(net_caps), "virtio-net");

    // Step 9: feature negotiation OK
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_FEATURES_OK);
	mb();
	if (!(regs->Status & VIRTIO_STATUS_FEATURES_OK)) {
		KPrintf("VirtioNetInit: virtio-net did not accept our features\n");
		return -ERROR;
	}

    // Step 10: device specific initialization e.g. virtqueue
	// For virtio-net, number of vqs is decided by VIRTIO_NET_F_MQ 
    // TODO: implemetation needs change and parameter is temporary
	if (READ_ONCE(regs->DeviceFeatures) & (1 << VIRTIO_NET_F_MQ)) {
		mq = config->max_virtqueue_pairs;
	}

	for (int i = 0; i < mq; i++) {
		if (i >= 1) {
			KPrintf("VirtioNetInit: currently not support multi vq!\n");
			return -ERROR;
		}
		if (VirtqueueAlloc(vdev, 0, 8, VQ_NET_RX) != 0) {
			KPrintf("VirtioNetInit: virtqueue allocation error\n");
			return -ERROR;
		}
		if (VirtqueueAlloc(vdev, 0, 8, VQ_NET_TX) != 0) {
			KPrintf("VirtioNetInit: virtqueue allocation error\n");
			return -ERROR;
		}
	}

	if (READ_ONCE(regs->DeviceFeatures) & (1 << VIRTIO_NET_F_CTRL_VQ)) {
		// need an extra control vq
		if (VirtqueueAlloc(vdev, 0, 8, VQ_NET_CTRL) != 0) {
			KPrintf("VirtioNetInit: virtqueue allocation error\n");
			return -ERROR;
		}
	}

	// TODO: Fill the receive queues with buffers
	VirtioNetRxInit(vdev);
	tx_handler_tid = 0;
	rx_handler_tid = 0;
	device_tid = 0;

    // TODO: some initialization stuff

	// TODO: mac check
	
	// TODO: status check

	// TODO: checksum

	// TODO: TCP or UDP

	// NOTE: actually the Config field is set by the hardware device
	// 		 thus we can get some information from it. But depends on
	//		 features selected by regs->DeviceFeatures
    // vdev->private_data = (void *)&regs->Config;

    // Step 11: driver ok
    WRITE_ONCE(regs->Status, READ_ONCE(regs->Status) | VIRTIO_STATUS_DRIVER_OK);
	mb();

    return EOK;
}

/************************************
 *      Functions for devices       *
 ************************************/
// static int cnt = 0;
/**
 * VirtioNetDeviceTx
 * Description:
 * 		virtio network device function to fetch a packet from
 * 		vq and submit to physical device
 * @param: virtio serial device
 */
static void VirtioNetDeviceTx(void *param)
{
	// TODO: maybe this information should be read from device registers
	// TODO: maybe need some signal mechanism
	uint32 i;
	uint32 desc1, desc2;
	uint16 avail_flags;
	uint16 used_flags;
	uint32 addr;
	uint32 len;
	uint32 stat;
	x_base lock;
	struct VirtioHardwareDevice *vdev = (struct VirtioHardwareDevice *)param;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_NET_TX);
	struct VirtioNetConfig *config = (struct VirtioNetConfig *)&vdev->regs->Config;

	used_flags = vq->vr.used->flags;

	if (used_flags & VRING_USED_F_NO_NOTIFY) {
		// no notify mechanism, device should polling maybe
	} else {
		while (1) {
			lock = CriticalAreaLock();
			if (vq->num_added == 0) {
				CriticalAreaUnLock(lock);
				KSemaphoreObtain(vq->avail_sem, WAITING_FOREVER);
				continue;
			}

			stat = READ_ONCE(vdev->regs->QueueNotify);
			if (stat != vq->idx){
				KPrintf("VirtioSerialDevicePutChar: QueueNotify status wrong! get %d should be %d",
						stat, vq->idx);
				KPrintf("Should NOT get here!\n");
				CriticalAreaUnLock(lock);
				return;
			}

			for (i = vq->last_avail_idx; i != vq->vr.avail->idx; i = (i+1) & ~(vq->num_total)) {
				desc1 = vq->vr.avail->ring[i];
				desc2 = vq->vr.desc[desc1].next;
				avail_flags = vq->vr.avail->flags;

				struct VirtioNetHdr *hdr = (struct VirtioNetHdr *)vq->vr.desc[desc1].addr;
				struct VirtioNetPkt *pkt = hdr->packet;

				// TODO: checksum
				
				// TODO: may need semaphore

				// TODO: submit packet to the really network device, not implemented
				KPrintf("VirtioNetDeviceTx: transmitting packet...\n");
				KPrintf("VirtioNetDeviceTx: payload: %s\n", pkt->data);
				// FIXME: for debug use
				vdev->hwdev_done->put_char(vdev, '#');

				// Put used desc into used ring
				VqAddUsed(vq, desc1, 1);

				// TODO: the device should interrupt the driver according to vr_avails flag 
				//		 and VIRTIO_F_EVENT_IDX bit
				if (avail_flags & ~VRING_AVAIL_F_NO_INTERRUPT) {
					// NOTE: normally, it is no need to inform the driver
				}
			}
			vq->last_avail_idx = vq->vr.avail->idx;
			vq->num_added = 0;

			// Interrupt driver
			WRITE_ONCE(vdev->regs->InterruptStatus, VIRTIO_USED_RING_UPDATE);
			// FIXME: normally, it is no need to inform the driver
			CriticalAreaUnLock(lock);
			KSemaphoreSetValue(vq->used_sem, 0);
			KSemaphoreAbandon(vq->used_sem);
		}
		
	}
}

/**
 * VirtioNetDeviceRx
 * Description:
 * 		virtio network device function to get a packet from
 * 		physical device and add to rx vq
 * @vdev: virtio serial device
 * @event: irq event id
 */
void VirtioNetDeviceRx(struct VirtioHardwareDevice *vdev, int event)
{
#ifdef DEBUG_TEST_1
	unsigned long start_time_2 = virtio_get_time();
	KPrintf("start rx: %ld\n", start_time_2);
#endif

	// TODO: maybe this information should be read from device registers
	// TODO: maybe need some signal mechanism
	uint32 i;
	uint32 desc1, desc2;
	uint16 avail_flags;
	uint16 used_flags;
	uint32 addr;
	uint32 len;
	uint32 stat;
	int ch;
	VirtqueueType vq = VirtioDevFindVq(vdev, VQ_NET_RX);
	struct VirtioNetConfig *config = (struct VirtioNetConfig *)&vdev->regs->Config;
	char *dummy = "DummyNetPacket";

	used_flags = vq->vr.used->flags;

	if (used_flags & VRING_USED_F_NO_NOTIFY) {
		// no notify mechanism, device should polling maybe
	} else {
		// Clear the FIFO character
		while (-ERROR !=(ch = vdev->hwdev_done->get_char(vdev))){
			KPrintf("get char: %c\n", ch);
		}

		stat = READ_ONCE(vdev->regs->QueueNotify);
		if (stat != vq->idx){
			KPrintf("VirtioSerialDevicePutChar: QueueNotify status wrong! get %d should be %d",
					stat, vq->idx);
			KPrintf("Should NOT get here!\n");
			return;
		}

		// TODO: should call device specific function to get network packet

		i = vq->last_avail_idx;
		if (i == vq->vr.avail->idx){
			// wait for avail descs
		}
		
		desc1 = vq->vr.avail->ring[i];
		desc2 = vq->vr.desc[desc1].next;
		avail_flags = vq->vr.avail->flags;

		struct VirtioNetHdr *hdr = (struct VirtioNetHdr *)vq->vr.desc[desc1].addr;
		struct VirtioNetPkt *pkt = hdr->packet;

		// number of buffers containing the single packet
		// decided by VIRTIO_NET_F_MRG_RXBUF
		if (READ_ONCE(vdev->regs->DeviceFeatures) & (1 << VIRTIO_NET_F_MRG_RXBUF)){
			// TODO: set the num_buffers to actual number of descs
		} else {
			hdr->num_buffers = 1;
		}

		// TODO: checksum
		
		// TODO: may need semaphore
		// TODO: receive packet to the really network device, not implemented
		KPrintf("VirtioNetDeviceRx: receiving packet...\n");
		KPrintf("VirtioNetDeviceRx: %d byte payload: %s\n", strlen(dummy), dummy);

		memcpy(pkt->data, dummy, strlen(dummy));

		// Put used desc into used ring
		// FIXME: len is with wrong meaning
		VqAddUsed(vq, desc1, 1);

		// TODO: the device should interrupt the driver according to vr_avails flag 
		//		 and VIRTIO_F_EVENT_IDX bit
		if (avail_flags & ~VRING_AVAIL_F_NO_INTERRUPT) {
			// NOTE: normally, it is no need to inform the driver
		}
		
		vq->last_avail_idx = (vq->last_avail_idx + 1) & ~(vq->num_total);

		// Interrupt driver
		WRITE_ONCE(vdev->regs->InterruptStatus, VIRTIO_USED_RING_UPDATE);
		// FIXME: normally, it is no need to inform the driver
		KSemaphoreSetValue(vq->used_sem, 0);
		KSemaphoreAbandon(vq->used_sem);
	}
}
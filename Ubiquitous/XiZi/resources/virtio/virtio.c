#include "virtio.h"
#include "virt_ring.h"
#include "dev_virtio.h"

#define VQ_ALIGN	64
#define ROUND_PAGE(size)	(((size) + MM_PAGE_SIZE - 1) & ~(MM_PAGE_SIZE - 1))
#define POWEROF2(n)			~(n & (n - 1))

/*
 * The maximum virtqueue size is 2^15. Use that value as the end of
 * descriptor chain terminator since it will never be a valid index
 * in the descriptor table. This is used to verify we are correctly
 * handling vq_free_cnt.
 */
#define VQ_RING_DESC_CHAIN_END 32768

/* initialize vq's vring */
static int VqRingInit(VirtqueueType);

/* Create the vq linklist */
static void VqLinkInit(VirtqueueType);

/*****************************************
 *  Shadow vring support functions  *
 *****************************************/
/*
 * Vring initialization
 */
static int VqRingInit(VirtqueueType vq)
{
	struct vring *vr;
	int i, vq_size;

	vq_size = vq->num_free;

	// FIXME: data area may be too small ?
	vq->vring_mem_size = vring_size(vq_size, VQ_ALIGN);
	if (vq->vring_mem_size > MM_PAGE_SIZE) {
		KPrintf("virtq_create: error, too big for a page\n");
		x_free(vq);
		return -ERROR;
	}

	vq->vring_mem = (char *)x_malloc(vq->vring_mem_size);
	if (NONE == vq->vring_mem) {
		KPrintf("Virtqueue x_malloc error\n");
        x_free(vq->vring_mem);
        return -ERROR;
	}
    memset(vq->vring_mem, 0, vq->vring_mem_size);

	vr = &vq->vr;

	vring_init(vr, vq_size, vq->vring_mem, VQ_ALIGN);

	for (i = 0; i < vq_size - 1; i++)
		vr->desc[i].next = i + 1;
	vr->desc[i].next = VQ_RING_DESC_CHAIN_END;

	return EOK;
}

/*
 * Set regs bits
 */
static int VqSetRegs(VirtioRegs *regs, VirtqueueType vq)
{
	WRITE_ONCE(regs->QueueSel, vq->idx);
	mb();
	WRITE_ONCE(regs->QueueNum, vq->num_free);
	WRITE_ONCE(regs->QueueDescLow, vq->vr.desc);
	WRITE_ONCE(regs->QueueDescHigh, 0);
	WRITE_ONCE(regs->QueueAvailLow, vq->vr.avail);
	WRITE_ONCE(regs->QueueAvailHigh, 0);
	WRITE_ONCE(regs->QueueUsedLow, vq->vr.used);
	WRITE_ONCE(regs->QueueUsedHigh, 0);
	mb();
	WRITE_ONCE(regs->QueueReady, 1);
}

/*
 * Virtqueue initialization
 */
int VirtqueueAlloc(struct VirtioHardwareDevice *dev, uint32 queue, uint32 size, char *vq_name)
{
	int ret = EOK;
	struct virtqueue *vq;

	if (size == 0) {
		KPrintf("virtqueue %d (%s) does not exist (size is zero)\n",
		    queue, vq_name);
		return ERROR;
	} else if (!POWEROF2(size)) {
		// FIXME: need to reverse, size must be power of 2
		KPrintf("virtqueue %d (%s) size is not a power of 2: %d\n",
		    queue, vq_name, size);
		return ERROR;
	} 

	vq = x_malloc(sizeof(struct virtqueue));
	if (NONE == vq) {
		KPrintf("Virtqueue x_malloc error\n");
        x_free(vq);
        return ERROR;
	}
    memset(vq, 0, sizeof(struct virtqueue));

	vq->dev = dev;
    vq->idx = queue;
	vq->name = vq_name;
	vq->num_free = size;
	vq->num_total = size;
	vq->avail_sem = KSemaphoreCreate(0);
	vq->used_sem = KSemaphoreCreate(0);

	if (EOK != (ret = VqRingInit(vq))){
		x_free(vq);
		return ret;
	}
	VqLinkInit(vq);		// Init vq's vqlink and add vq into dev's link
	VqSetRegs(dev->regs, vq);	// Set regs bits according to the vq inited
	// FIXME: not implemented
	// virtqueue_disable_intr(vq);

    return ret;
}

/*****************************************
 *  Vq list help functions  *
 *****************************************/
/* Create the vq linklist */
static void VqLinkInit(VirtqueueType vq)
{
	struct VirtioHardwareDevice *vdev = vq->dev;

	/*Create the hardware device of the bus linklist*/
	if(!vdev->vq_link_flag) {
		InitDoubleLinkList(&vdev->vqs);
		vdev->vq_link_flag = RET_TRUE;
  	}

	vdev->vq_cnt++;

  	DoubleLinkListInsertNodeAfter(&vdev->vqs, &(vq->vq_list));
}

/*****************************************
 *  IRQ support functions  *
 *****************************************/
// Start receiving avail event
static void VqEnableAvailIrq(VirtqueueType vq)
{
  	vring_avail_event(&vq->vr) = vq->last_avail_idx;
  	mb();
}

// Start receiving used event
static void VqEnableUsedIrq(VirtqueueType vq)
{
  	vring_used_event(&vq->vr) = vq->last_avail_idx;
  	mb();
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/
uint32 VqAllocDesc(VirtqueueType vq, void *addr, uint32 len)
{
	uint32 desc = vq->free_head;
	uint32 next = vq->vr.desc[desc].next;
	if (desc == vq->num_total || vq->num_free == 0) {
		KPrintf("ERROR: ran out of virtqueue descriptors\n");
	}
		
	vq->free_head = next;

	vq->vr.desc[desc].addr = (uint32)addr;
	vq->vr.desc[desc].len = len;

	// TODO: should check VIRTIO_DESC_F_WRITE

	vq->num_free--;

	return desc;
}

void VqFreeDesc(VirtqueueType vq, uint32 desc)
{
	vq->vr.desc[desc].next = vq->free_head;
	vq->free_head = desc;
	// virtq->desc_virt[desc] = NULL;

	vq->num_free++;
}

void VqAddAvail(VirtqueueType vq, uint32 desc)
{
	vq->vr.avail->ring[vq->vr.avail->idx] = desc;
	mb();
	vq->vr.avail->idx = (vq->vr.avail->idx + 1) &  ~(vq->num_total);
	mb();
	vq->num_added++;
	mb();
}

void VqAddUsed(VirtqueueType vq, uint32 desc, uint32 len)
{
	vq->vr.used->ring[vq->vr.used->idx].id = desc;
	vq->vr.used->ring[vq->vr.used->idx].len = len;
	mb();
	vq->vr.used->idx = (vq->vr.used->idx + 1) &  ~(vq->num_total);
	mb();
}
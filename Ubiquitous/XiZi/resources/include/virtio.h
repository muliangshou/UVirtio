#ifndef VIRTIO_H
#define VIRTIO_H

#include <virt_ring.h>
#include "list.h"
#include "dev_virtio.h"

typedef struct virtqueue *VirtqueueType;

/*
 * Generate interrupt when the virtqueue ring is
 * completely used, even if we've suppressed them.
 */
#define VIRTIO_F_NOTIFY_ON_EMPTY (1 << 24)

/*
 * The guest should never negotiate this feature; it
 * is used to detect faulty drivers.
 */
#define VIRTIO_F_BAD_FEATURE (1 << 30)

/*
 * Some VirtIO feature bits (currently bits 28 through 31) are
 * reserved for the transport being used (eg. virtio_ring), the
 * rest are per-device feature bits.
 */
#define VIRTIO_TRANSPORT_F_START	28
#define VIRTIO_TRANSPORT_F_END		32

/*
 * Each virtqueue indirect descriptor list must be physically contiguous.
 * To allow us to malloc(9) each list individually, limit the number
 * supported to what will fit in one page. With 4KB pages, this is a limit
 * of 256 descriptors. If there is ever a need for more, we can switch to
 * contigmalloc(9) for the larger allocations, similar to what
 * bus_dmamem_alloc(9) does.
 *
 * Note the sizeof(struct vring_desc) is 16 bytes.
 */
#define VIRTIO_MAX_INDIRECT ((int) (PAGE_SIZE / 16))

/*
 * Virtqueue
 */

/* Support for indirect buffer descriptors. */
#define VIRTIO_F_INDIRECT_DESC	(1 << 28)

/* Support to suppress interrupt until specific index is reached. */
#define VIRTIO_F_EVENT_IDX		(1 << 29)

struct virtqueue {
	struct vring vr;

	const char *name;
	uint16 idx;
	DoubleLinklistType vq_list;

	uint32 free_head;
	uint32 num_free;
	uint32 num_added;
	uint32 num_total;
	
	uint32 last_avail_idx;
	uint32 last_used_idx;

	struct VirtioHardwareDevice *dev;

	// NOTE: vring_mem is for both vr metadata and data
	void *vring_mem;
	uint32 vring_mem_size;
	// NOTE: Below is for vring data
	// NOTE: version 2, buffer is allocated by driver not device
	// uint32 vring_data;
	// uint32 vring_end;
	// uint32 vring_data_size;

	// uint32 vq_data_head;		// Track the head of allocated memory region, for future allocation
	// uint32 vq_data_tail;		// Track the tail of allocated memory region, for reclaimation
	int32 avail_sem;			// Semaphore to inform device of incoming avail descs
	int32 used_sem;				// Semaphore to inform driver of incoming used descs
};

int	 VirtqueueAlloc(struct VirtioHardwareDevice *dev, uint32 idx, uint32 size, char *vq_name);
uint32 VqAllocDesc(VirtqueueType vq, void *addr, uint32 len);
void VqFreeDesc(VirtqueueType vq, uint32 desc);
void VqAddAvail(VirtqueueType vq, uint32 desc);
void VqAddUsed(VirtqueueType vq, uint32 desc, uint32 len);

#endif /* VIRTRING_H */

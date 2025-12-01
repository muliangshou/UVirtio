/* An interface for efficient virtio implementation, currently for use by KVM,
 * but hopefully others soon.  Do NOT change this since it will
 * break existing servers and clients.
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of IBM nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL IBM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Copyright Rusty Russell IBM Corporation 2007. */

#ifndef VIRT_RING_H
#define VIRT_RING_H

#include <xizi.h>

/* We support indirect buffer descriptors */
#define VRING_F_INDIRECT_DESC	28

/* The Guest publishes the used index for which it expects an interrupt
 * at the end of the avail ring. Host should ignore the avail->flags field.
 */
 /* The Host publishes the avail index for which it expects a kick
  * at the end of the used ring. Guest should ignore the used->flags field.
  */
#define VRING_F_EVENT_IDX	29

/* VirtIO ring descriptors: 16 bytes.
 * These can chain together via "next". */
struct vring_desc {
        /* Address (guest-physical). */
        /* NOTE: riscv-32 */
        uint32 addr;
        /* Length. */
        uint32 len;
/* This marks a buffer as continuing via the next field. */
#define VRING_DESC_F_NEXT       1
/* This marks a buffer as write-only (otherwise read-only). */
#define VRING_DESC_F_WRITE      2
/* This means the buffer contains a list of buffer descriptors. */
#define VRING_DESC_F_INDIRECT	4
        /* The flags as indicated above. */
        uint16 flags;
        /* We chain unused descriptors via this, too. */
        uint16 next;
};

struct vring_avail {
/* The Guest uses this in avail->flags to advise the Host: don't
 * interrupt me when you consume a buffer.  It's unreliable, so it's
 * simply an optimization.  */
#define VRING_AVAIL_F_NO_INTERRUPT      1
        uint16 flags;
        uint16 idx;
        uint16 ring[];
        /* NOTE: at the tail of vring_avail is used_event
        /* uint16 used_event */
};

/* uint32 is used here for ids for padding reasons. */
struct vring_used_elem {
        /* Index of start of used descriptor chain. */
        uint32 id;
        /* Total length of the descriptor chain which was written to. */
        uint32 len;
};

struct vring_used {
/* The Host uses this in used->flags to advise the Guest: don't kick me
 * when you add a buffer.  It's unreliable, so it's simply an
 * optimization.  Guest will still kick if it's out of buffers. */
#define VRING_USED_F_NO_NOTIFY  1
        uint16 flags;
        uint16 idx;
        struct vring_used_elem ring[];
        /* NOTE: at the tail of vring_used is avail_event
        /* uint16 avail_event */
};

struct vring { 
    unsigned int num;               // Number of vring_desc

    struct vring_desc *desc;        // The actual descriptors (16 bytes each) 
    struct vring_avail *avail;      // A ring of available descriptor heads with free-running index. 
        
    // u8 pad[ Padding ];           // Padding to the next Queue Align boundary. 
        
    struct vring_used *used;        // A ring of used descriptor heads with free-running index. 
};

/* The standard layout for the ring is a continuous chunk of memory which
 * looks like this.  We assume num is a power of 2.
 *
 * struct vring {
 *      // The actual descriptors (16 bytes each)
 *      struct vring_desc desc[num];
 *
 *      // A ring of available descriptor heads with free-running index.
 *      __u16 avail_flags;
 *      __u16 avail_idx;
 *      __u16 available[num];
 *      __u16 used_event_idx;
 *
 *      // Padding to the next align boundary.
 *      char pad[];
 *
 *      // A ring of used descriptor heads with free-running index.
 *      __u16 used_flags;
 *      __u16 used_idx;
 *      struct vring_used_elem used[num];
 *      __u16 avail_event_idx;
 * };
 *
 * NOTE: for VirtIO PCI, align is 4096.
 */

/*
 * We publish the used event index at the end of the available ring, and vice
 * versa. They are at the end for backwards compatibility.
 */

#define vring_used_event(vr)	((vr)->avail->ring[(vr)->num])
#define vring_avail_event(vr)	(*(uint16 *)&(vr)->used->ring[(vr)->num])

static inline int
vring_size(unsigned int num, unsigned long align)
{
	int size;

	size = num * sizeof(struct vring_desc);
	size += (3 + num) * sizeof(uint16);
	size = (size + align - 1) & ~(align - 1);
	size += (num * sizeof(struct vring_used_elem)) + 3 * sizeof(uint16);
	return (size);
}

static inline void
vring_init(struct vring *vr, unsigned int num, void *p,
    unsigned long align)
{
        vr->num = num;
        vr->desc = (struct vring_desc *) p;
        vr->avail = (struct vring_avail *) (p +
	    num * sizeof(struct vring_desc));
        vr->used = (void *)
	    (((unsigned long) &vr->avail->ring[num] + sizeof(uint16) 
            + align-1) & ~(align-1));
}

/*
 * The following is used with VIRTIO_RING_F_EVENT_IDX.
 *
 * Assuming a given event_idx value from the other size, if we have
 * just incremented index from old to new_idx, should we trigger an
 * event?
 */
static inline int
vring_need_event(uint16 event_idx, uint16 new_idx, uint16 old)
{

	return (uint16)(new_idx - event_idx - 1) < (uint16)(new_idx - old);
}

#define WRITE_ONCE(var, val) \
        (*((volatile typeof(val) *)(&(var))) = (val))

#define READ_ONCE(var) (*((volatile typeof(var) *)(&(var))))

#ifdef ARCH_RISCV
#define mb()                          \
    {                                 \
        asm volatile("fence" ::       \
                         : "memory"); \
    }
#endif
#ifdef ARCH_ARM
#define mb()                          \
    {                                 \
        asm volatile("DSB ISH"); \
    }
#endif

#define virt_store_release(p, v)\
    do {\
        mb();\
        WRITE_ONCE(*p, v);\
    } while (0)

#define virt_load_acquire(p)\
    ({\
        typeof(*p) ___p1 = READ_ONCE(*p);\
        mb();\
        ___p1;\
     })

#endif /* VIRTIO_RING_H */
/**
 * Copyright (c) 2013 Pranav Sawargaonkar.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * @file virtio_mmio.h
 * @author Elliot
 * @brief VirtIO MMIO Transport Interface
 */

#include <xs_base.h>

#ifndef VIRTIO_MMIO_H
#define VIRTIO_MMIO_H

#define VIRTIO_MAGIC   0x74726976
#define VIRTIO_VERSION 0x2

// TODO: see virtio document
/* VirtIO device IDs. */
#define VIRTIO_ID_NETWORK	0x01
#define VIRTIO_ID_BLOCK		0x02
#define VIRTIO_ID_CONSOLE	0x03
#define VIRTIO_ID_ENTROPY	0x04
#define VIRTIO_ID_BALLOON	0x05
#define VIRTIO_ID_IOMEMORY	0x06
#define VIRTIO_ID_SCSI		0x08
#define VIRTIO_ID_9P		0x09
#define VIRTIO_ID_SERIAL	0X13

#define VIRTIO_STATUS_ACKNOWLEDGE        (1)
#define VIRTIO_STATUS_DRIVER             (2)
#define VIRTIO_STATUS_FAILED             (128)
#define VIRTIO_STATUS_FEATURES_OK        (8)
#define VIRTIO_STATUS_DRIVER_OK          (4)
#define VIRTIO_STATUS_DEVICE_NEEDS_RESET (64)

#define wrap(x, len)   ((x) & ~(len))

#define VIRTIO_USED_RING_UPDATE		0
#define VIRTIO_CONFIG_CHANGE		1

/*
 * Control registers (offset) --> Copied from linux's virtio_mmio.h
 */

/* Magic value ("virt" string) - Read Only */
#define VIRTIO_MMIO_MAGIC_VALUE		0x000

/* Virtio device version - Read Only */
#define VIRTIO_MMIO_VERSION			0x004

/* Virtio device ID - Read Only */
#define VIRTIO_MMIO_DEVICE_ID		0x008

/* Virtio vendor ID - Read Only */
#define VIRTIO_MMIO_VENDOR_ID		0x00c

/* Bitmask of the features supported by the host (device)
 * (32 bits per set) - Read Only */
#define VIRTIO_MMIO_HOST_FEATURES		0x010
#define VIRTIO_MMIO_DEVICE_FEATURES		\
				VIRTIO_MMIO_HOST_FEATURES

/* Host (device) features set selector - Write Only */
#define VIRTIO_MMIO_HOST_FEATURES_SEL	0x014
#define VIRTIO_MMIO_DEVICE_FEATURES_SEL	\
				VIRTIO_MMIO_HOST_FEATURES_SEL

/* Bitmask of features activated by the guest (driver)
 * (32 bits per set) - Write Only */
#define VIRTIO_MMIO_GUEST_FEATURES		0x020
#define VIRTIO_MMIO_DRIVER_FEATURES		\
				VIRTIO_MMIO_GUEST_FEATURES

/* Activated features set selector by the guest (driver) - Write Only */
#define VIRTIO_MMIO_GUEST_FEATURES_SEL	0x024
#define VIRTIO_MMIO_DRIVER_FEATURES_SEL	\
				VIRTIO_MMIO_GUEST_FEATURES_SEL

/* Guest's memory page size in bytes - Write Only */
#define VIRTIO_MMIO_GUEST_PAGE_SIZE		0x028

/* Queue selector - Write Only */
#define VIRTIO_MMIO_QUEUE_SEL		0x030

/* Maximum size of the currently selected queue - Read Only */
#define VIRTIO_MMIO_QUEUE_NUM_MAX		0x034

/* Queue size for the currently selected queue - Write Only */
#define VIRTIO_MMIO_QUEUE_NUM		0x038

/* Used Ring alignment for the currently selected queue - Write Only */
#define VIRTIO_MMIO_QUEUE_ALIGN		0x03c

/* PFN for the currently selected queue - Read Write */
#define VIRTIO_MMIO_QUEUE_PFN		0x040

/* Ready bit for the currently selected queue - Read Write */
#define VIRTIO_MMIO_QUEUE_READY		0x044

/* Queue notifier - Write Only */
#define VIRTIO_MMIO_QUEUE_NOTIFY		0x050

/* Interrupt status - Read Only */
#define VIRTIO_MMIO_INTERRUPT_STATUS	0x060

/* Interrupt acknowledge - Write Only */
#define VIRTIO_MMIO_INTERRUPT_ACK		0x064

/* Device status register - Read Write */
#define VIRTIO_MMIO_STATUS			0x070

/* Selected queue's Descriptor Table address, 64 bits in two halves */
#define VIRTIO_MMIO_QUEUE_DESC_LOW		0x080
#define VIRTIO_MMIO_QUEUE_DESC_HIGH		0x084

/* Selected queue's Available Ring address, 64 bits in two halves */
#define VIRTIO_MMIO_QUEUE_AVAIL_LOW		0x090
#define VIRTIO_MMIO_QUEUE_AVAIL_HIGH	0x094

/* Selected queue's Used Ring address, 64 bits in two halves */
#define VIRTIO_MMIO_QUEUE_USED_LOW		0x0a0
#define VIRTIO_MMIO_QUEUE_USED_HIGH		0x0a4

/* Configuration atomicity value */
#define VIRTIO_MMIO_CONFIG_GENERATION	0x0fc

/* The config space is defined by each driver as
 * the per-driver configuration space - Read Write */
#define VIRTIO_MMIO_CONFIG			0x100


/*
 * Interrupt flags (re: interrupt status & acknowledge registers)
 */

#define VIRTIO_MMIO_INT_VRING		(1 << 0)
#define VIRTIO_MMIO_INT_CONFIG		(1 << 1)

#define VIRTIO_MMIO_MAX_VQ			3
#define VIRTIO_MMIO_MAX_CONFIG		1
#define VIRTIO_MMIO_IO_SIZE			0x200

// MMIO Device Register Layout according to virtio spec
typedef volatile struct __attribute__((packed)) {
	uint32 MagicValue;
	uint32 Version;
	uint32 DeviceID;
	uint32 VendorID;
	uint32 DeviceFeatures;
	uint32 DeviceFeaturesSel;
	uint32 _reserved0[2];
	uint32 DriverFeatures;
	uint32 DriverFeaturesSel;
	uint32 _reserved1[2];
	uint32 QueueSel;
	uint32 QueueNumMax;
	uint32 QueueNum;
	uint32 _reserved2[2];
	uint32 QueueReady;
	uint32 _reserved3[2];
	uint32 QueueNotify;
	uint32 _reserved4[3];
	uint32 InterruptStatus;
	uint32 InterruptACK;
	uint32 _reserved5[2];
	uint32 Status;
	uint32 _reserved6[3];
	uint32 QueueDescLow;
	uint32 QueueDescHigh;
	uint32 _reserved7[2];
	uint32 QueueAvailLow;
	uint32 QueueAvailHigh;
	uint32 _reserved8[2];
	uint32 QueueUsedLow;
	uint32 QueueUsedHigh;
	uint32 _reserved9[21];
	uint32 ConfigGeneration;
	uint32 Config[0];
} VirtioRegs;

#endif /* VIRTIO_MMIO_H */
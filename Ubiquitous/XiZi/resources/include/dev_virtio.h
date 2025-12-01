/**
* @file dev_virtio.h
* @brief define virtio dev function using bus driver framework
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2021-04-24
*/

#ifndef DEV_VIRTIO_H
#define DEV_VIRTIO_H

#include <bus.h>
#include <virtio_mmio.h>

// #define DEBUG_TIMER

// Virtio general macro
#define nelem(x) (sizeof(x) / sizeof(x[0]))
struct VirtioCap {
	char *name;
	uint32 bit;
	x_bool support;
	char *help;
};

// Virtio Serial Device
#define VQ_SERIAL_TX "VIRTIO_SERIAL_TX"
#define VQ_SERIAL_RX "VIRTIO_SERIAL_RX"

// Virtio Block Device
#define VQ_BLK "VIRTIO_BLK"

#define VIRTIO_BLK_F_SIZE_MAX 	1
#define VIRTIO_BLK_F_SEG_MAX	2
#define VIRTIO_BLK_F_GEOMETRY	4
#define VIRTIO_BLK_F_RO			5
#define	VIRTIO_BLK_F_BLK_SIZE	6
#define VIRTIO_BLK_F_FLUSH		9
#define VIRTIO_BLK_F_TOPOLOGY	10
#define VIRTIO_BLK_F_CONFIG_WCE	11

struct VirtioBlkConfig {
	uint64 capacity;
	uint32 size_max;
	uint32 seg_max;
	struct {
		uint16 cylinders;
		uint8 heads;
		uint8 sectors;
	} geometry;
	uint32 blk_size;
	struct {
		uint8 physical_block_exp;
		uint8 alignment_offset;
		uint16 min_io_size;
		uint32 opt_io_size;
	} topology;
	uint8 writeback;
} __attribute__((packed));

#define VIRTIO_BLK_REQ_HEADER_SIZE 16
#define VIRTIO_BLK_REQ_FOOTER_SIZE 1
#define VIRTIO_BLK_SECTOR_SIZE 512

struct VirtioBlkReq {
#define VIRTIO_BLK_T_IN    0
#define VIRTIO_BLK_T_OUT   1
#define VIRTIO_BLK_T_SCSI  2
#define VIRTIO_BLK_T_FLUSH 4
	uint32 type;
	uint32 reserved;
	uint64 sector;
#define VIRTIO_BLK_S_OK 0
#define VIRTIO_BLK_S_IOERR 1
#define VIRTIO_BLK_S_UNSUPP 2
	uint8 status;
	// pointer to data to write or buffer to fill
	uint8 *data;
};

// Virtio Network Device
#define VQ_NET_RX "VIRTIO_NET_RX"
#define VQ_NET_TX "VIRTIO_NET_TX"
#define VQ_NET_CTRL "VIRTIO_NET_CTRL"

// virtio network device feature bits
#define VIRTIO_NET_F_CSUM			0
#define VIRTIO_NET_F_GUEST_CSUM		1
#define VIRTIO_NET_F_CTRL_GUEST_OFFLOADS 	2
#define VIRTIO_NET_F_MAC			5
#define VIRTIO_NET_F_GUEST_TSO4		7
#define VIRTIO_NET_F_GUEST_TSO6		8
#define VIRTIO_NET_F_GUEST_ECN		9
#define VIRTIO_NET_F_GUEST_UFO		10
#define VIRTIO_NET_F_HOST_TSO4		11
#define VIRTIO_NET_F_HOST_TSO6		12
#define VIRTIO_NET_F_HOST_ECN		13
#define VIRTIO_NET_F_HOST_UFO		14
#define VIRTIO_NET_F_MRG_RXBUF		15
#define VIRTIO_NET_F_STATUS			16
#define VIRTIO_NET_F_CTRL_VQ		17
#define VIRTIO_NET_F_CTRL_RX		18
#define VIRTIO_NET_F_CTRL_VLAN		19
#define VIRTIO_NET_F_GUEST_ANNOUNCE 21
#define VIRTIO_NET_F_MQ				22
#define VIRTIO_NET_F_CTRL_MAC_ADDR	23

struct VirtioNetConfig {
	uint8 mac[6];		// Only valid when VIRTIO_NET_F_MAC is set
#define VIRTIO_NET_S_LINK_UP  1
#define VIRTIO_NET_S_ANNOUNCE 2
	uint16 status;		// Only valid when VIRTIO_NET_F_STATUS is set
	uint16 max_virtqueue_pairs;	// Only exists if VIRTIO_NET_F_MQ is set
} __attribute__((packed));

struct VirtioNetPkt {
	char *data;
};

struct VirtioNetHdr {
#define VIRTIO_NET_HDR_F_NEEDS_CSUM 1
	uint8 flags;
#define VIRTIO_NET_HDR_GSO_NONE 0
#define VIRTIO_NET_HDR_GSO_TCPV4 1
#define VIRTIO_NET_HDR_GSO_UDP 3
#define VIRTIO_NET_HDR_GSO_TCPV6 4
#define VIRTIO_NET_HDR_GSO_ECN 0x80
	uint8 gso_type;
	uint16 hdr_len;
	uint16 gso_size;
	uint16 csum_start;
	uint16 csum_offset;
	uint16 num_buffers;
	struct VirtioNetPkt *packet;
};

#define VIRTIO_NET_HDRLEN 10

#define VIRTIO_NET_Q_RX 0
#define VIRTIO_NET_Q_TX 1

// General Virtio Structures

struct VirtioHardwareDevice;

// NOTE: seems interface for internal functions
// TODO: need change
struct VirtioHwDevDone
{
    int (*put_char) (struct VirtioHardwareDevice *virtio_dev, char c);
    int (*get_char) (struct VirtioHardwareDevice *virtio_dev);
    int (*dmatransfer) (struct VirtioHardwareDevice *virtio_dev, uint8 *buf, x_size_t size, int direction);
};

// NOTE: seems interface to outside
// TODO: need change
struct VirtioDevDone
{
    uint32 (*open) (void *dev);
    uint32 (*close) (void *dev);
    uint32 (*write) (void *dev, struct BusBlockWriteParam *datacfg);
    uint32 (*read) (void *dev, struct BusBlockReadParam *datacfg);
};

// TODO: need change
struct VirtioHardwareDevice
{
    struct HardwareDev haldev;
    struct VirtioHwDevDone *hwdev_done;

    VirtioRegs *regs;                       // registers of virtio device see virtio_mmio.h
    uint8 vq_cnt;
    uint8 vq_link_flag;
    DoubleLinklistType vqs;                 // pointer to the list of virtqueue of virtio_device

    const struct VirtioDevDone *dev_done;   // TODO: this may change accoding to different virtio device type

    void *private_data;
};

/*Register the serial device*/
int VirtioDeviceRegister(struct VirtioHardwareDevice *virtio_device, void *virtio_param, const char *device_name);

/*Register the serial device to the serial bus*/
int VirtioDeviceAttachToBus(const char *dev_name, const char *bus_name);

/*Find the register serial device*/
HardwareDevType VirtioDeviceFind(const char *dev_name, enum DevType dev_type);

/*Set serial isr function*/
void VirtioSetIsr(void *dev, int event);

/*Find vq of virtio device*/
struct virtqueue *VirtioDevFindVq(struct VirtioHardwareDevice *dev, const char *vq_name);

void VirtioFeaturesSelect(VirtioRegs *regs, struct VirtioCap *caps,
                               uint32 n, char *whom);

// general virtio device capabilities
#define VIRTIO_INDP_CAPS                                                       \
	{ "VIRTIO_F_INDIRECT_DESC", 28, RET_FALSE,                            \
	  "Negotiating this feature indicates that the driver can use"         \
	  " descriptors with the VIRTQ_DESC_F_INDIRECT flag set, as"           \
	  " described in 2.4.5.3 Indirect Descriptors." },                     \
	        { "VIRTIO_F_EVENT_IDX", 29, RET_FALSE,                        \
		  "This feature enables the used_event and the avail_event "   \
		  "fields"                                                     \
		  " as described in 2.4.7 and 2.4.8." },                       \
	        { "VIRTIO_F_VERSION_1", 32, RET_FALSE,                             \
		  "This indicates compliance with this specification, giving " \
		  "a"                                                          \
		  " simple way to detect legacy devices or drivers." },

/*
 * Virtio Device Initialization and function (temporarily)
 */
int VirtioSerialInit(struct VirtioHardwareDevice *vdev);
int VirtioSerialWrite(struct VirtioHardwareDevice *vdev, struct BusBlockWriteParam *write_param);
int VirtioSerialRead(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param);
void VirtioSerialDeviceGetChar(struct VirtioHardwareDevice *vdev, int event);

int VirtioBlkInit(struct VirtioHardwareDevice *vdev);
int VirtioBlkWrite(struct VirtioHardwareDevice *vdev, struct BusBlockWriteParam *write_param);
int VirtioBlkRead(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param);

int VirtioNetInit(struct VirtioHardwareDevice *vdev);
int VirtioNetWrite(struct VirtioHardwareDevice *vdev, struct BusBlockWriteParam *write_param);
int VirtioNetRead(struct VirtioHardwareDevice *vdev, struct BusBlockReadParam *read_param);
void VirtioNetDeviceRx(struct VirtioHardwareDevice *vdev, int event);

/*
 * Helper for test
 */
#ifdef DEBUG_TIMER
static unsigned long virtio_get_time()
{
	unsigned long tmp, t;
	tmp = get_timer_value();
    do {
        t = get_timer_value();
    } while (t == tmp);
	return t;
}
#endif

#endif
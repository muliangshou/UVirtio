#ifndef ETHERCAT_H
#define ETHERCAT_H

#include <stdint.h>

#define BIG_ENDIAN_PACK __attribute__((packed, scalar_storage_order("big-endian")))
#define LITTLE_ENDIAN_PACK __attribute__((packed, scalar_storage_order("little-endian")))
#define ETHERCAT_PORT "34980" // 0x88A4
#define MAX_FRAME_LEN 1500
#define MIN_FRAME_LEN 14

typedef union BIG_ENDIAN_PACK
{
    struct BIG_ENDIAN_PACK
    {
        uint16_t length : 11;
        uint16_t res : 1;
        uint16_t type : 4;
    };
    uint16_t header;
} EcatHeader, *EcatHeaderPtr;

#define ECAT_HEADER_LENGTH 2

typedef union BIG_ENDIAN_PACK
{
    struct BIG_ENDIAN_PACK
    {
        uint16_t position;
        uint16_t pos_offset;
    }; // Position Addressing
    struct BIG_ENDIAN_PACK
    {
        uint16_t address;
        uint16_t addr_offset;
    }; // Node Addressing
    uint32_t logical;
} EcatAddress, *EcatAddressPtr;

#define ECAT_ADDRESS_LENGTH 4

typedef struct
{
    uint8_t cmd;
    uint8_t idx;
    EcatAddress address;
    union BIG_ENDIAN_PACK
    {
        struct BIG_ENDIAN_PACK
        {
            uint16_t length : 11;
            uint16_t r : 3;
            uint16_t c : 1;
            uint16_t m : 1; // followed by more datagrams or not
        };
        uint16_t suffix; // for easy parse
    };
    uint16_t irq : 16;
} EcatDataHeader, *EcatDataHeaderPtr;

#define ECAT_DATA_HEADER_LENGTH 10

typedef struct
{
    // frame data
    EcatDataHeader header; // 10 bytes
    uint8_t *data;         // 0-1486 bytes
    uint16_t work_counter; // 2bytes
    // utilities
    uint8_t self; // indicating this datagram is mine
    void *next;   // EcatDatagramPtr
} EcatDatagram, *EcatDatagramPtr;

typedef struct
{
    EcatHeader header;        // 2 bytes
    EcatDatagramPtr datagram; // 12-1470 bytes
} EcatFrame, *EcatFramePtr;

extern EcatFrame ecat_data;

void EcatClear(EcatFramePtr frame);
int EcatAppend(EcatFramePtr frame, EcatAddress address, uint8_t *data, uint16_t data_len);
int EcatUpdate(EcatFramePtr frame, uint8_t *data, uint16_t data_len);

#define READ8(buffer, offset) (((uint8_t *)buffer)[offset++])
#define READ16(buffer, offset) ((uint16_t)((uint8_t *)buffer)[offset++] << 8 | (uint16_t)((uint8_t *)buffer)[offset++])
#define READ32(buffer, offset) (uint32_t)((uint8_t *)buffer)[offset++] << 24 |     \
                                   (uint32_t)((uint8_t *)buffer)[offset++] << 16 | \
                                   (uint32_t)((uint8_t *)buffer)[offset++] << 8 |  \
                                   (uint32_t)((uint8_t *)buffer)[offset++]

#define WRITE8(buffer, offset, value) ((uint8_t *)buffer)[offset++] = (uint8_t)(value & 0xFF);
#define WRITE16(buffer, offset, value)                            \
    ((uint8_t *)buffer)[offset++] = (uint8_t)(value >> 8 & 0xFF); \
    ((uint8_t *)buffer)[offset++] = (uint8_t)(value & 0xFF);
#define WRITE32(buffer, offset, value)                             \
    ((uint8_t *)buffer)[offset++] = (uint8_t)(value >> 24 & 0xFF); \
    ((uint8_t *)buffer)[offset++] = (uint8_t)(value >> 16 & 0xFF); \
    ((uint8_t *)buffer)[offset++] = (uint8_t)(value >> 8 & 0xFF);  \
    ((uint8_t *)buffer)[offset++] = (uint8_t)(value & 0xFF);

#endif
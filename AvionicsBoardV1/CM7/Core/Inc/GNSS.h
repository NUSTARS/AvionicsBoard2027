/*
 * ubx_gnss.h
 *
 * Minimal UBX binary protocol driver for u-blox GNSS modules on STM32 HAL,
 * UART only.
 *
 * Architecturally modeled after SparkFun_u-blox_GNSS_v3: packets are
 * checksum-validated before being trusted, and each message type is decoded
 * into a named struct that mirrors the u-blox interface description, rather
 * than being read out of a raw buffer at hardcoded byte offsets.
 *
 * Covers UBX-NAV-PVT, UBX-ACK-ACK/NAK, and UBX-CFG-VALSET (the key-based
 * config interface u-blox introduced with M9/M10 and prefers going forward).
 * Add more message types by following the pattern in UBX_ParsePacket() /
 * UBX_ParseNavPvt() in the .c file.
 */

#ifndef UBX_GNSS_H
#define UBX_GNSS_H

#include "stm32h7xx_hal.h"   /* swap for your family, e.g. stm32l4xx_hal.h, stm32g0xx_hal.h */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_SYNC_CHAR_1     0xB5
#define UBX_SYNC_CHAR_2     0x62

#define UBX_CLASS_NAV       0x01
#define UBX_CLASS_ACK       0x05
#define UBX_CLASS_CFG       0x06

#define UBX_ID_NAV_PVT      0x07
#define UBX_ID_ACK_NAK      0x00
#define UBX_ID_ACK_ACK      0x01
#define UBX_ID_CFG_VALSET   0x8A
#define UBX_ID_CFG_VALGET   0x8B

/* CFG-NAVSPG-DYNMODEL key, RAM layer, 1-byte value. Values match the legacy
 * CFG-NAV5 dynModel enum: 0 Portable, 2 Stationary, 3 Pedestrian,
 * 4 Automotive, 5 Sea, 6/7/8 Airborne <1g/<2g/<4g, 9 Wrist, 10 Bike. */
#define UBX_KEY_CFG_NAVSPG_DYNMODEL 0x20110021UL

#define UBX_MAX_PAYLOAD_LEN 128   /* NAV-PVT is 92 bytes; raise this if you add bigger messages */

typedef enum {
    UBX_STATE_SYNC1 = 0,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B
} UBX_RxState_t;

/* UBX-NAV-PVT payload, 92 bytes, unchanged across M8/M9/M10 firmware.
 * STM32 and the UBX protocol are both little-endian, so a direct packed
 * overlay onto the raw payload bytes is safe (unlike on big-endian targets). */
typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;       /* 1e-7 deg */
    int32_t  lat;       /* 1e-7 deg */
    int32_t  height;    /* mm, above ellipsoid */
    int32_t  hMSL;      /* mm, above mean sea level */
    uint32_t hAcc;       /* mm */
    uint32_t vAcc;       /* mm */
    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;     /* mm/s */
    int32_t  headMot;    /* 1e-5 deg */
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t  flags3;
    uint8_t  reserved0[5];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} __attribute__((packed)) UBX_NAV_PVT_t;   /* sizeof == 92 */

typedef struct {
    UART_HandleTypeDef *huart;

    /* byte-wise receive state machine, fed from the UART RX ISR */
    UBX_RxState_t rxState;
    uint8_t  rxByte;               /* landing zone for HAL_UART_Receive_IT */
    uint8_t  rxClass;
    uint8_t  rxId;
    uint16_t rxLen;
    uint16_t rxPayloadIndex;
    uint8_t  rxPayload[UBX_MAX_PAYLOAD_LEN];
    uint8_t  ckA, ckB;
    uint8_t  rollingCkA, rollingCkB;

    /* set by the ISR, consumed by UBX_Update() in the main loop */
    volatile uint8_t packetAvailable;

    /* DEBUG: counts every byte the ISR receives, valid packet or not.
     * If this stays at 0, nothing is arriving on the wire at all - a
     * wiring/baud/power problem upstream of the parser. If it climbs but
     * packetsValid stays at 0, bytes are arriving but never forming a
     * checksum-valid UBX packet (wrong baud, swapped TX/RX, noise, or
     * the module is in NMEA-only mode instead of outputting UBX). */
    volatile uint32_t rxByteCount;
    volatile uint32_t rxPacketValidCount;

    /* decoded data */
    UBX_NAV_PVT_t pvt;
    volatile uint8_t pvtUpdated;

    /* ACK/NAK tracking for config commands */
    volatile int8_t ackResult;      /* -1 none pending, 0 NAK/timeout, 1 ACK */
    uint8_t ackClass;
    uint8_t ackId;

} UBX_HandleTypeDef;

void    UBX_Init(UBX_HandleTypeDef *ubx, UART_HandleTypeDef *huart);
void    UBX_RxByteCallback(UBX_HandleTypeDef *ubx, uint8_t incoming);
void    UBX_Update(UBX_HandleTypeDef *ubx);

void    UBX_SendPacket(UBX_HandleTypeDef *ubx, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
void    UBX_PollMessage(UBX_HandleTypeDef *ubx, uint8_t cls, uint8_t id);

uint8_t UBX_GetPVT(UBX_HandleTypeDef *ubx, uint32_t timeoutMs);
uint8_t UBX_WaitAck(UBX_HandleTypeDef *ubx, uint32_t timeoutMs);
uint8_t UBX_SetDynamicModel(UBX_HandleTypeDef *ubx, uint8_t model, uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* UBX_GNSS_H */

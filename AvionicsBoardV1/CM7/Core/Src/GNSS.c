/*
 * ubx_gnss.c
 *
 * See ubx_gnss.h for design notes.
 */

#include "GNSS.h"
#include <string.h>

static void UBX_ParsePacket(UBX_HandleTypeDef *ubx);
static void UBX_ParseNavPvt(UBX_HandleTypeDef *ubx);
static void UBX_ParseAck(UBX_HandleTypeDef *ubx);
static void UBX_ChecksumUpdate(UBX_HandleTypeDef *ubx, uint8_t b);

void UBX_Init(UBX_HandleTypeDef *ubx, UART_HandleTypeDef *huart)
{
    memset(ubx, 0, sizeof(UBX_HandleTypeDef));
    ubx->huart = huart;
    ubx->rxState = UBX_STATE_SYNC1;
    ubx->ackResult = -1;

    /* Continuous single-byte interrupt reception. HAL_UART_RxCpltCallback
     * must call UBX_RxByteCallback() for this huart instance - see the
     * glue code in the usage example. */
    HAL_UART_Receive_IT(ubx->huart, &ubx->rxByte, 1);
}

static void UBX_ChecksumUpdate(UBX_HandleTypeDef *ubx, uint8_t b)
{
    ubx->rollingCkA = (uint8_t)(ubx->rollingCkA + b);
    ubx->rollingCkB = (uint8_t)(ubx->rollingCkB + ubx->rollingCkA);
}

/* Call this from HAL_UART_RxCpltCallback() for the matching huart instance.
 * Kept deliberately light (no memcpy, no float math) since it runs in
 * interrupt context; actual field decoding happens later in UBX_Update(). */
void UBX_RxByteCallback(UBX_HandleTypeDef *ubx, uint8_t incoming)
{
    ubx->rxByteCount++;   /* DEBUG: counts every byte regardless of parse outcome */

    switch (ubx->rxState)
    {
    case UBX_STATE_SYNC1:
        if (incoming == UBX_SYNC_CHAR_1)
            ubx->rxState = UBX_STATE_SYNC2;
        break;

    case UBX_STATE_SYNC2:
        if (incoming == UBX_SYNC_CHAR_2)
        {
            ubx->rollingCkA = 0;
            ubx->rollingCkB = 0;
            ubx->rxState = UBX_STATE_CLASS;
        }
        else
        {
            ubx->rxState = UBX_STATE_SYNC1; /* resync */
        }
        break;

    case UBX_STATE_CLASS:
        ubx->rxClass = incoming;
        UBX_ChecksumUpdate(ubx, incoming);
        ubx->rxState = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        ubx->rxId = incoming;
        UBX_ChecksumUpdate(ubx, incoming);
        ubx->rxState = UBX_STATE_LEN1;
        break;

    case UBX_STATE_LEN1:
        ubx->rxLen = incoming;
        UBX_ChecksumUpdate(ubx, incoming);
        ubx->rxState = UBX_STATE_LEN2;
        break;

    case UBX_STATE_LEN2:
        ubx->rxLen |= (uint16_t)((uint16_t)incoming << 8);
        UBX_ChecksumUpdate(ubx, incoming);
        ubx->rxPayloadIndex = 0;

        if (ubx->rxLen == 0)
            ubx->rxState = UBX_STATE_CK_A;
        else if (ubx->rxLen > UBX_MAX_PAYLOAD_LEN)
            ubx->rxState = UBX_STATE_SYNC1; /* bigger than our buffer, drop and resync */
        else
            ubx->rxState = UBX_STATE_PAYLOAD;
        break;

    case UBX_STATE_PAYLOAD:
        ubx->rxPayload[ubx->rxPayloadIndex++] = incoming;
        UBX_ChecksumUpdate(ubx, incoming);
        if (ubx->rxPayloadIndex >= ubx->rxLen)
            ubx->rxState = UBX_STATE_CK_A;
        break;

    case UBX_STATE_CK_A:
        ubx->ckA = incoming;
        ubx->rxState = UBX_STATE_CK_B;
        break;

    case UBX_STATE_CK_B:
        ubx->ckB = incoming;
        if (ubx->ckA == ubx->rollingCkA && ubx->ckB == ubx->rollingCkB)
        {
            /* Valid packet - hand off to the main loop. */
            ubx->packetAvailable = 1;
            ubx->rxPacketValidCount++;   /* DEBUG */
        }
        /* else: checksum mismatch, silently dropped - this is the check
         * SimpleMethod/STM32-GNSS does not do. */
        ubx->rxState = UBX_STATE_SYNC1;
        break;

    default:
        ubx->rxState = UBX_STATE_SYNC1;
        break;
    }

    /* Re-arm for the next byte */
    HAL_UART_Receive_IT(ubx->huart, &ubx->rxByte, 1);
}

void UBX_Update(UBX_HandleTypeDef *ubx)
{
    if (!ubx->packetAvailable)
        return;

    UBX_ParsePacket(ubx);
    ubx->packetAvailable = 0;
}

/* Class/ID dispatch table. Add new message types here following the
 * NAV-PVT pattern: define a struct in the header matching the interface
 * description, add a case, add a parse function that memcpy's rxPayload
 * into that struct. */
static void UBX_ParsePacket(UBX_HandleTypeDef *ubx)
{
    switch (ubx->rxClass)
    {
    case UBX_CLASS_NAV:
        if (ubx->rxId == UBX_ID_NAV_PVT && ubx->rxLen == sizeof(UBX_NAV_PVT_t))
            UBX_ParseNavPvt(ubx);
        break;

    case UBX_CLASS_ACK:
        UBX_ParseAck(ubx);
        break;

    default:
        break;
    }
}

static void UBX_ParseNavPvt(UBX_HandleTypeDef *ubx)
{
    memcpy(&ubx->pvt, ubx->rxPayload, sizeof(UBX_NAV_PVT_t));
    ubx->pvtUpdated = 1;
}

static void UBX_ParseAck(UBX_HandleTypeDef *ubx)
{
    ubx->ackClass = ubx->rxPayload[0];
    ubx->ackId    = ubx->rxPayload[1];
    ubx->ackResult = (ubx->rxId == UBX_ID_ACK_ACK) ? 1 : 0;
}

void UBX_SendPacket(UBX_HandleTypeDef *ubx, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t header[6];
    uint8_t ckA = 0, ckB = 0;
    uint8_t footer[2];
    uint16_t i;

    header[0] = UBX_SYNC_CHAR_1;
    header[1] = UBX_SYNC_CHAR_2;
    header[2] = cls;
    header[3] = id;
    header[4] = (uint8_t)(len & 0xFF);
    header[5] = (uint8_t)((len >> 8) & 0xFF);

    for (i = 2; i < 6; ++i) { ckA = (uint8_t)(ckA + header[i]); ckB = (uint8_t)(ckB + ckA); }
    for (i = 0; i < len; ++i) { ckA = (uint8_t)(ckA + payload[i]); ckB = (uint8_t)(ckB + ckA); }

    footer[0] = ckA;
    footer[1] = ckB;

    HAL_UART_Transmit(ubx->huart, header, sizeof(header), 100);
    if (len > 0)
        HAL_UART_Transmit(ubx->huart, (uint8_t *)payload, len, 100);
    HAL_UART_Transmit(ubx->huart, footer, sizeof(footer), 100);
}

void UBX_PollMessage(UBX_HandleTypeDef *ubx, uint8_t cls, uint8_t id)
{
    UBX_SendPacket(ubx, cls, id, NULL, 0);
}

uint8_t UBX_GetPVT(UBX_HandleTypeDef *ubx, uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();

    ubx->pvtUpdated = 0;
    UBX_PollMessage(ubx, UBX_CLASS_NAV, UBX_ID_NAV_PVT);

    while (!ubx->pvtUpdated)
    {
        UBX_Update(ubx);
        if ((HAL_GetTick() - start) > timeoutMs)
            return 0;
    }
    return 1;
}

uint8_t UBX_WaitAck(UBX_HandleTypeDef *ubx, uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();
    uint8_t result;

    while (ubx->ackResult == -1)
    {
        UBX_Update(ubx);
        if ((HAL_GetTick() - start) > timeoutMs)
            return 0; /* timed out - treat like a NAK */
    }
    result = (uint8_t)ubx->ackResult;
    ubx->ackResult = -1;
    return result; /* 1 = ACK, 0 = NAK/timeout */
}

uint8_t UBX_SetDynamicModel(UBX_HandleTypeDef *ubx, uint8_t model, uint32_t timeoutMs)
{
    /* CFG-VALSET, RAM layer only, single key: CFG-NAVSPG-DYNMODEL.
     * This is the config interface M9/M10 prefer; it's the fix for the
     * "legacy CFG-NAV5 gets NAK'd on M10" problem flagged for the
     * SimpleMethod/STM32-GNSS library. */
    uint8_t payload[9] = {
        0x00, /* version */
        0x01, /* layer: RAM */
        0x00, 0x00, /* reserved */
        (uint8_t)(UBX_KEY_CFG_NAVSPG_DYNMODEL & 0xFF),
        (uint8_t)((UBX_KEY_CFG_NAVSPG_DYNMODEL >> 8) & 0xFF),
        (uint8_t)((UBX_KEY_CFG_NAVSPG_DYNMODEL >> 16) & 0xFF),
        (uint8_t)((UBX_KEY_CFG_NAVSPG_DYNMODEL >> 24) & 0xFF),
        model
    };

    ubx->ackResult = -1;
    UBX_SendPacket(ubx, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, sizeof(payload));
    return UBX_WaitAck(ubx, timeoutMs);
}

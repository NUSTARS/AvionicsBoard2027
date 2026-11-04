#ifndef UART_TELEMETRY_FORMAT_H
#define UART_TELEMETRY_FORMAT_H
#include "flight_log_format.h"
/* Fixed compact wire schema. All multibyte data are little-endian. */
#define UART_TELEMETRY_VERSION 1U
#define UART_TELEMETRY_DATA_SIZE 46U
#define UART_TELEMETRY_BLOCKS 7U
#define UART_TELEMETRY_PACKET_SIZE (4U + 9U * UART_TELEMETRY_BLOCKS)
#pragma pack(push, 1)
typedef struct {
 uint8_t version;
 uint32_t packet_counter;
 uint32_t timestamp_us_delta;
 uint8_t nav_flightMode_enum;
 float nav_phiEst_rad, nav_thetaEst_rad, nav_psiEst_rad;
 float nav_xVelEstNed_mps, nav_yVelEstNed_mps, nav_zVelEstNed_mps;
 float nav_xPosEst_m, nav_yPosEst_m, nav_zPosEst_m;
} UartTelemetryPayload_t;
#pragma pack(pop)
_Static_assert(sizeof(UartTelemetryPayload_t)==UART_TELEMETRY_DATA_SIZE, "UART payload packing changed");
_Static_assert(sizeof(float)==4U, "UART floats must be IEEE-754 binary32");
static void UartTelemetry_Encode(const navigation_bus *nav, uint32_t sequence,
                                  uint32_t delta_us, uint8_t out[UART_TELEMETRY_PACKET_SIZE])
{
 UartTelemetryPayload_t p;
 uint8_t data[UART_TELEMETRY_BLOCKS*8U];
 memset(&p,0,sizeof(p));
 p.version=UART_TELEMETRY_VERSION;
 p.packet_counter=sequence; p.timestamp_us_delta=delta_us;
 p.nav_flightMode_enum=(uint8_t)nav->flightMode_enum;
 p.nav_phiEst_rad=(float)nav->phiEst_rad;
 p.nav_thetaEst_rad=(float)nav->thetaEst_rad;
 p.nav_psiEst_rad=(float)nav->psiEst_rad;
 p.nav_xVelEstNed_mps=(float)nav->xVelEstNed_mps;
 p.nav_yVelEstNed_mps=(float)nav->yVelEstNed_mps;
 p.nav_zVelEstNed_mps=(float)nav->zVelEstNed_mps;
 p.nav_xPosEst_m=(float)nav->xPosEst_m;
 p.nav_yPosEst_m=(float)nav->yPosEst_m;
 p.nav_zPosEst_m=(float)nav->zPosEst_m;
 memset(data,0,sizeof(data)); memcpy(data,&p,sizeof(p));
 FlightLog_Put32(data+sizeof(p),FlightLog_Crc32(data,sizeof(p)));
 memcpy(out,"NTL1",4U);
 for(unsigned i=0;i<UART_TELEMETRY_BLOCKS;i++)
  FlightLog_EncodeWord(data+8U*i,out+4U+9U*i);
}
#endif

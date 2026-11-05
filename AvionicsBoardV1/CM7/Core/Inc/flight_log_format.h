#ifndef FLIGHT_LOG_FORMAT_H
#define FLIGHT_LOG_FORMAT_H
/* NUFLIGHT v2. Include after main.h (FlightData_t). See flight_log_standard.md. */
#include <stdint.h>
#include "autocodeModel.h"
#include <stddef.h>
#include <string.h>
#define FLIGHT_LOG_PREFIX_SIZE 20U
#define FLIGHT_LOG_DESCRIPTOR_SIZE 28U
#define FLIGHT_LOG_FIELD_COUNT 67U
#define FLIGHT_LOG_HEADER_SIZE (20U + 28U * FLIGHT_LOG_FIELD_COUNT + 4U)
/* Preserve generated bus layout/padding; descriptors expose each actual field. */
#pragma pack(push, 1)
typedef struct {
 FlightData_t base;
 navigation_bus navBus;
 controller_bus controllerBus;
 uint8_t gps_valid;
 uint32_t gps_age_ms;
 uint8_t gnssFixOK; /* Raw latest NAV-PVT flags bit 0; check gps_valid for freshness. */
 uint16_t lsm_accel_samples;
 uint16_t lsm_gyro_samples;
} FlightLogRecord_t;
#pragma pack(pop)
#define FLIGHT_LOG_PAYLOAD_SIZE ((uint16_t)sizeof(FlightLogRecord_t))
_Static_assert(sizeof(navigation_bus)==248U && sizeof(controller_bus)==152U,
               "Generated buses changed: refresh the complete log schema");
_Static_assert(sizeof(real_T)==8U, "real_T width");
_Static_assert(sizeof(boolean_T)==1U, "boolean_T width");
_Static_assert(sizeof(flightMode)==1U || sizeof(flightMode)==4U, "flightMode width");
#define FLIGHT_LOG_BLOCKS ((FLIGHT_LOG_PAYLOAD_SIZE + 4U + 7U) / 8U)
#define FLIGHT_LOG_PACKET_SIZE (4U + FLIGHT_LOG_BLOCKS * 9U)
enum { FLT_U8=1, FLT_U16=2, FLT_U32=3, FLT_I32=4, FLT_F32=5, FLT_F64=6 };
typedef struct { const char *name; uint8_t type, count; uint16_t offset; } FlightLogField;
#define FLT_FIELD(name,type,count) {#name,type,count,(uint16_t)offsetof(FlightLogRecord_t,base.name)}
static const FlightLogField flight_log_fields[FLIGHT_LOG_FIELD_COUNT] = {
 FLT_FIELD(timestamp_us_delta,FLT_U32,1), FLT_FIELD(packet_counter,FLT_U32,1),
 FLT_FIELD(bmi_accel,FLT_F64,3), FLT_FIELD(bmi_gyro,FLT_F64,3),
 FLT_FIELD(lsm_accel,FLT_F64,3), FLT_FIELD(lsm_gyro,FLT_F64,3),
 FLT_FIELD(bmp581_pressure,FLT_F32,1), FLT_FIELD(bmp581_temperature,FLT_F32,1),
 FLT_FIELD(bmp585_pressure,FLT_F32,1), FLT_FIELD(bmp585_temperature,FLT_F32,1),
 FLT_FIELD(latitude,FLT_F64,1), FLT_FIELD(longitude,FLT_F64,1),
 FLT_FIELD(altitude,FLT_F32,1), FLT_FIELD(velocity_north,FLT_F32,1),
 FLT_FIELD(velocity_east,FLT_F32,1), FLT_FIELD(velocity_down,FLT_F32,1),
 FLT_FIELD(gnss_fix_type,FLT_U8,1), FLT_FIELD(gnss_satellites,FLT_U8,1),
 {"nav_flightMode_enum",(sizeof(flightMode)==1U?FLT_U8:FLT_I32),1,(uint16_t)offsetof(FlightLogRecord_t,navBus.flightMode_enum)},
 {"nav_qEst_na",FLT_F64,4,(uint16_t)offsetof(FlightLogRecord_t,navBus.qEst_na)},
 {"nav_phiEst_rad",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.phiEst_rad)},
 {"nav_thetaEst_rad",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.thetaEst_rad)},
 {"nav_psiEst_rad",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.psiEst_rad)},
 {"nav_pEst_rps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.pEst_rps)},
 {"nav_qEst_rps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.qEst_rps)},
 {"nav_rEst_rps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.rEst_rps)},
 {"nav_xAccEst_mps2",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.xAccEst_mps2)},
 {"nav_yAccEst_mps2",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.yAccEst_mps2)},
 {"nav_zAccEst_mps2",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.zAccEst_mps2)},
 {"nav_accelBias_mps2",FLT_F64,3,(uint16_t)offsetof(FlightLogRecord_t,navBus.accelBias_mps2)},
 {"nav_gyroBias_rps",FLT_F64,3,(uint16_t)offsetof(FlightLogRecord_t,navBus.gyroBias_rps)},
 {"nav_altitudeBias_m",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.altitudeBias_m)},
 {"nav_llaBias_na",FLT_F64,3,(uint16_t)offsetof(FlightLogRecord_t,navBus.llaBias_na)},
 {"nav_dynPressureEst_Pa",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.dynPressureEst_Pa)},
 {"nav_xVelEstNed_mps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.xVelEstNed_mps)},
 {"nav_yVelEstNed_mps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.yVelEstNed_mps)},
 {"nav_zVelEstNed_mps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.zVelEstNed_mps)},
 {"nav_xPosEst_m",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.xPosEst_m)},
 {"nav_yPosEst_m",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.yPosEst_m)},
 {"nav_zPosEst_m",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,navBus.zPosEst_m)},
 {"ctl_o_pitch_rateCmd_rps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.pitchOuterLoop_bus.rateCmd_rps)},
 {"ctl_o_pitch_rateLimited",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.pitchOuterLoop_bus.rateLimited_b)},
 {"ctl_o_pitch_saturated",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.pitchOuterLoop_bus.saturated_b)},
 {"ctl_i_pitch_control_deg",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.pitchInnerLoop_bus.control_deg)},
 {"ctl_i_pitch_moment_Nm",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.pitchInnerLoop_bus.controlMoment_Nm)},
 {"ctl_i_pitch_rateLimited",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.pitchInnerLoop_bus.rateLimted_b)},
 {"ctl_i_pitch_saturated",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.pitchInnerLoop_bus.saturated_b)},
 {"ctl_o_yaw_rateCmd_rps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.yawOuterLoop_bus.rateCmd_rps)},
 {"ctl_o_yaw_rateLimited",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.yawOuterLoop_bus.rateLimited_b)},
 {"ctl_o_yaw_saturated",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.yawOuterLoop_bus.saturated_b)},
 {"ctl_i_yaw_control_deg",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.yawInnerLoop_bus.control_deg)},
 {"ctl_i_yaw_moment_Nm",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.yawInnerLoop_bus.controlMoment_Nm)},
 {"ctl_i_yaw_rateLimited",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.yawInnerLoop_bus.rateLimted_b)},
 {"ctl_i_yaw_saturated",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.yawInnerLoop_bus.saturated_b)},
 {"ctl_o_roll_rateCmd_rps",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.rollOuterLoop_bus.rateCmd_rps)},
 {"ctl_o_roll_rateLimited",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.rollOuterLoop_bus.rateLimited_b)},
 {"ctl_o_roll_saturated",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.outerLoop_bus.rollOuterLoop_bus.saturated_b)},
 {"ctl_i_roll_control_deg",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.rollInnerLoop_bus.control_deg)},
 {"ctl_i_roll_moment_Nm",FLT_F64,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.rollInnerLoop_bus.controlMoment_Nm)},
 {"ctl_i_roll_rateLimited",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.rollInnerLoop_bus.rateLimted_b)},
 {"ctl_i_roll_saturated",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.innerLoop_bus.rollInnerLoop_bus.saturated_b)},
 {"ctl_control_deg",FLT_F64,4,(uint16_t)offsetof(FlightLogRecord_t,controllerBus.control_deg)},
 {"gps_valid",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,gps_valid)},
 {"gps_age_ms",FLT_U32,1,(uint16_t)offsetof(FlightLogRecord_t,gps_age_ms)},
 {"gnssFixOK",FLT_U8,1,(uint16_t)offsetof(FlightLogRecord_t,gnssFixOK)},
 {"lsm_accel_samples",FLT_U16,1,(uint16_t)offsetof(FlightLogRecord_t,lsm_accel_samples)},
 {"lsm_gyro_samples",FLT_U16,1,(uint16_t)offsetof(FlightLogRecord_t,lsm_gyro_samples)}
};
#undef FLT_FIELD
static void FlightLog_Put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void FlightLog_Put32(uint8_t *p, uint32_t v) {
 for (unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8U*i));
}
static uint32_t FlightLog_Crc32(const uint8_t *p, size_t n) {
 uint32_t crc=0xFFFFFFFFU;
 for(size_t i=0;i<n;i++) {
  crc^=p[i];
  for(unsigned bit=0;bit<8;bit++) crc=(crc>>1)^((crc&1U)?0xEDB88320U:0U);
 }
 return crc^0xFFFFFFFFU;
}
static uint16_t FlightLog_BuildHeader(uint8_t out[FLIGHT_LOG_HEADER_SIZE]) {
 memset(out,0,FLIGHT_LOG_HEADER_SIZE); memcpy(out,"NUFLIGHT",8);
 out[8]=2U; out[9]=1U; out[10]=1U; out[11]=1U;
 FlightLog_Put16(out+12,FLIGHT_LOG_HEADER_SIZE);
 FlightLog_Put16(out+14,FLIGHT_LOG_PAYLOAD_SIZE);
 FlightLog_Put16(out+16,FLIGHT_LOG_PACKET_SIZE);
 FlightLog_Put16(out+18,FLIGHT_LOG_FIELD_COUNT);
 for(unsigned i=0;i<FLIGHT_LOG_FIELD_COUNT;i++) {
  uint8_t *d=out+FLIGHT_LOG_PREFIX_SIZE+i*FLIGHT_LOG_DESCRIPTOR_SIZE;
  size_t len=strlen(flight_log_fields[i].name);
  if(len>=24U) return 0U;
  memcpy(d,flight_log_fields[i].name,len);
  d[24]=flight_log_fields[i].type; d[25]=flight_log_fields[i].count;
  FlightLog_Put16(d+26,flight_log_fields[i].offset);
 }
 FlightLog_Put32(out+FLIGHT_LOG_HEADER_SIZE-4U,FlightLog_Crc32(out,FLIGHT_LOG_HEADER_SIZE-4U));
 return FLIGHT_LOG_HEADER_SIZE;
}
/* Positions 1..71: parity at 1,2,4,8,16,32,64; position 72 is overall parity.
 * Positions and data bits are stored least-significant-bit first. */
static void FlightLog_EncodeWord(const uint8_t in[8],uint8_t out[9]) {
 unsigned data_bit=0,syndrome=0,overall=0;
 memset(out,0,9);
 for(unsigned pos=1;pos<=71;pos++) {
  if((pos&(pos-1U))==0U) continue;
  unsigned bit=(in[data_bit/8U]>>(data_bit%8U))&1U; data_bit++;
  if(bit) { out[(pos-1U)/8U]|=(uint8_t)(1U<<((pos-1U)%8U)); syndrome^=pos; overall^=1U; }
 }
 for(unsigned parity=1;parity<=64;parity<<=1) {
  if(syndrome&parity) {out[(parity-1U)/8U]|=(uint8_t)(1U<<((parity-1U)%8U));overall^=1U;}
 }
 if(overall) out[8]|=0x80U;
}
static uint16_t FlightLog_EncodePacket(const FlightLogRecord_t *record,uint8_t out[FLIGHT_LOG_PACKET_SIZE]) {
 uint8_t data[FLIGHT_LOG_BLOCKS*8U];
 memset(data,0,sizeof(data)); memcpy(data,record,FLIGHT_LOG_PAYLOAD_SIZE);
 FlightLog_Put32(data+FLIGHT_LOG_PAYLOAD_SIZE,FlightLog_Crc32(data,FLIGHT_LOG_PAYLOAD_SIZE));
 memcpy(out,"PKT2",4);
 for(unsigned i=0;i<FLIGHT_LOG_BLOCKS;i++) FlightLog_EncodeWord(data+i*8U,out+4U+i*9U);
 return FLIGHT_LOG_PACKET_SIZE;
}
#endif

#ifndef TMC5072_H_
#define TMC5072_H_

#define TMC5072_READ 0x00
#define TMC5072_WRITE 0x80

//General Configuration Register
#define TMC5072_GCONF 0x00
#define TMC5072_GSTAT 0x01

//Ramp Generator Motion Control Register
#define TMC5072_RAMPMODE1 0x20
#define TMC5072_RAMPMODE2 0x40
#define TMC5072_XACTUAL1 0x21
#define TMC5072_XACTUAL2 0x41
#define TMC5072_VACTUAL1 0x22  //Read only
#define TMC5072_VACTUAL2 0x42  //Read only
#define TMC5072_VSTART1 0x23
#define TMC5072_VSTART2 0x43
#define TMC5072_A11 0x24
#define TMC5072_A12 0x44
#define TMC5072_V11 0x25
#define TMC5072_V12 0x45
#define TMC5072_AMAX1 0x26
#define TMC5072_AMAX2 0x46
#define TMC5072_VMAX1 0x27
#define TMC5072_VMAX2 0x47
#define TMC5072_DMAX1 0x28
#define TMC5072_DMAX2 0x48
#define TMC5072_D11 0x2A
#define TMC5072_D12 0x4A
#define TMC5072_VSTOP1 0x2B
#define TMC5072_VSTOP2 0x4B
#define TMC5072_TZEROWAIT1 0x2C
#define TMC5072_TZEROWAIT2 0x4C
#define TMC5072_XTARGET1 0x2D
#define TMC5072_XTARGET2 0x4D

//Ramp Generator Driver Feature Control Register
#define TMC5072_IHOLD_IRUN1 0x30
#define TMC5072_IHOLD_IRUN2 0x50
#define TMC5072_VCOOLTHRS1 0x31
#define TMC5072_VCOOLTHRS2 0x51
#define TMC5072_VHIGH1 0x32
#define TMC5072_VHIGH2 0x52
#define TMC5072_VDCMIN1 0x33
#define TMC5072_VDCMIN2 0x53
#define TMC5072_SW_MODE1 0x34
#define TMC5072_SW_MODE2 0x54
#define TMC5072_RAMP_STAT1 0x35
#define TMC5072_RAMP_STAT2 0x55
#define TMC5072_XLATCH1 0x36
#define TMC5072_XLATCH2 0x56

///Motor Driver Register
#define TMC5072_MSCNT1 0x6A
#define TMC5072_MSCNT2 0x7A
#define TMC5072_MSURACT1 0x6B
#define TMC5072_MSURACT2 0x7B
#define TMC5072_CHOPCONG1 0x6C
#define TMC5072_CHOPCONG2 0x7C
#define TMC5072_COOLCONF1 0x6D
#define TMC5072_COOLCONF2 0x7D
#define TMC5072_DRVSTATUS1 0x6F
#define TMC5072_DRVSTATUS2 0x7F

typedef enum { STEPDIR, SIXPOINT, VELOCITY } t_TMC5072Mode;

#define TMC5072_PULSE (TIRE_DIAMETER * PI / (200.0 * 256.0))  //　1/256 0.003
#define TMC5072_FCLK (13000000.0)
#define TMC5072_2_24 (16777216.0)
#define TMC5072_2_17 (131072.0)
#define TMC5072_VELOCITY (TMC5072_PULSE * (TMC5072_FCLK / TMC5072_2_24))
#define TMC5072_ACCELE \
  (TMC5072_PULSE * (TMC5072_FCLK / TMC5072_2_24) * (TMC5072_FCLK / TMC5072_2_17))

// Register access permissions:
//   0x00: none (reserved)
//   0x01: read
//   0x02: write
//   0x03: read/write
//   0x13: read/write, separate functions/values for reading or writing
//   0x21: read, flag register (read to clear)
//   0x42: write, has hardware presets on reset
#define ____ 0x00
static const uint8_t tmc5072_defaultRegisterAccess[] = {
  //	0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
  0x03, 0x01, 0x01, 0x02, 0x13, 0x02, ____, ____,
  ____, ____, ____, ____, ____, ____, ____, ____,  // 0x00 - 0x0F
  0x02, 0x01, ____, ____, ____, ____, ____, ____,
  0x02, 0x01, ____, ____, ____, ____, ____, ____,  // 0x10 - 0x1F
  0x03, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02,
  0x02, ____, 0x02, 0x02, 0x02, 0x03, ____, ____,  // 0x20 - 0x2F
  0x02, 0x02, 0x02, 0x02, 0x03, 0x01, 0x01, ____,
  0x03, 0x03, 0x02, 0x01, 0x01, ____, ____, ____,  // 0x30 - 0x3F
  0x03, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02,
  0x02, ____, 0x02, 0x02, 0x02, 0x03, ____, ____,  // 0x40 - 0x4F
  0x02, 0x02, 0x02, 0x02, 0x03, 0x01, 0x01, ____,
  0x03, 0x03, 0x02, 0x01, 0x01, ____, ____, ____,  // 0x50 - 0x5F
  0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42,
  0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01,  // 0x60 - 0x6F
  ____, ____, ____, ____, ____, ____, ____, ____,
  ____, ____, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01  // 0x70 - 0x7F
};

typedef struct
{
  float init_speed;    //単位は[mm/s]
  float v1_speed;      //加速度A1のトップスピード[mm/s]
  float a1_accel;      //V1までの加速度[mm/s^2]
  float vmax_speed;    //トップスピード[mm/s]
  float amax_aceel;    //V1からVMAXまでの加速度[mm/s^2]
  float finish_speed;  //停止速度[mm/s]
  float len;           //距離[mm]
} t_TMC5072_position;

#endif  // TMC5072_H_
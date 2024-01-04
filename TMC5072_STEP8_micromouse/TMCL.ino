// Copyright 2024 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "TMC5072.h"
#include "stdio.h"

// these addresses are fixed
#define SERIAL_MODULE_ADDRESS 1
#define SERIAL_HOST_ADDRESS 2

const int BUILD_VERSION = 30824;
const char * VersionString = "0016V308";
#define ID_TMC5072 7
#define TMC_ADDRESS_MASK 0x7F
#define TMC_ADDRESS(x) ((x) & (TMC_ADDRESS_MASK))
#define CAST_Sn_TO_S32(value, n) \
  ((value) | (((value) & ((uint32_t)1 << ((n)-1))) ? ~(((uint32_t)1 << (n)) - 1) : 0))

// id detection state definitions
#define ID_STATE_WAIT_LOW 0   // id detection waiting for first edge (currently low)
#define ID_STATE_WAIT_HIGH 1  // id detection waiting for second edge (currently high)
#define ID_STATE_DONE 2       // id detection finished successfully
#define ID_STATE_INVALID \
  3  // id detection failed - we got an answer, but no corresponding ID (invalid ID pulse length)
#define ID_STATE_NO_ANSWER 4  // id detection failed - board doesn't answer
#define ID_STATE_TIMEOUT 5    // id detection failed - board id pulse went high but not low
#define ID_STATE_NOT_IN_FW \
  6  // id detection detected a valid id that is not supported in this firmware

// todo CHECK 2: these are unused - delete? (LH) #11
// tmcl interpreter states
#define TM_IDLE 0
#define TM_RUN 1
#define TM_STEP 2
#define TM_RESET 3  // unused
#define TM_DOWNLOAD 4
#define TM_DEBUG 5  // wie TM_IDLE, es wird jedoch der Akku nicht modifiziert bei GAP etc.

// todo CHECK 2: these are unused - delete? (LH) #12
#define TCS_IDLE 0
#define TCS_CAN7 1
#define TCS_CAN8 2
#define TCS_UART 3
#define TCS_UART_ERROR 4
#define TCS_UART_II 5
#define TCS_UART_II_ERROR 6
#define TCS_USB 7
#define TCS_USB_ERROR 8
#define TCS_MEM 9

// TMCL commands
#define TMCL_ROR 1
#define TMCL_ROL 2
#define TMCL_MST 3
#define TMCL_MVP 4
#define TMCL_SAP 5
#define TMCL_GAP 6
#define TMCL_STAP 7
#define TMCL_RSAP 8
#define TMCL_SGP 9
#define TMCL_GGP 10
#define TMCL_STGP 11
#define TMCL_RSGP 12
#define TMCL_RFS 13
#define TMCL_SIO 14
#define TMCL_GIO 15
#define TMCL_CALC 19
#define TMCL_COMP 20
#define TMCL_JC 21
#define TMCL_JA 22
#define TMCL_CSUB 23
#define TMCL_RSUB 24
#define TMCL_EI 25
#define TMCL_DI 26
#define TMCL_WAIT 27
#define TMCL_STOP 28
#define TMCL_SAC 29
#define TMCL_SCO 30
#define TMCL_GCO 31
#define TMCL_CCO 32
#define TMCL_CALCX 33
#define TMCL_AAP 34
#define TMCL_AGP 35
#define TMCL_CLE 36
#define TMCL_VECT 37
#define TMCL_RETI 38
#define TMCL_ACO 39

#define TMCL_UF0 64
#define TMCL_UF1 65
#define TMCL_UF2 66
#define TMCL_UF3 67
#define TMCL_UF4 68
#define TMCL_UF5 69
#define TMCL_UF6 70
#define TMCL_UF7 71
#define TMCL_UF8 72

#define TMCL_ApplStop 128
#define TMCL_ApplRun 129
#define TMCL_ApplStep 130
#define TMCL_ApplReset 131
#define TMCL_DownloadStart 132
#define TMCL_DownloadEnd 133
#define TMCL_ReadMem 134
#define TMCL_GetStatus 135
#define TMCL_GetVersion 136
#define TMCL_FactoryDefault 137
#define TMCL_SetEvent 138
#define TMCL_SetASCII 139
#define TMCL_SecurityCode 140
#define TMCL_Breakpoint 141
#define TMCL_RamDebug 142
#define TMCL_GetIds 143
#define TMCL_UF_CH1 144
#define TMCL_UF_CH2 145
#define TMCL_writeRegisterChannel_1 146
#define TMCL_writeRegisterChannel_2 147
#define TMCL_readRegisterChannel_1 148
#define TMCL_readRegisterChannel_2 149

#define TMCL_BoardMeasuredSpeed 150
#define TMCL_BoardError 151
#define TMCL_BoardReset 152

#define TMCL_WLAN 160
#define TMCL_WLAN_CMD 160
#define TMCL_WLAN_IS_RTS 161
#define TMCL_WLAN_CMDMODE_EN 162
#define TMCL_WLAN_IS_CMDMODE 163

#define TMCL_MIN 170
#define TMCL_MAX 171
#define TMCL_OTP 172

#define TMCL_Boot 242
#define TMCL_SoftwareReset 255

// Command type variants
#define MVP_ABS 0
#define MVP_REL 1
#define MVP_PRF 2

// GetVersion() Format types
#define VERSION_FORMAT_ASCII 0
#define VERSION_FORMAT_BINARY 1
#define VERSION_BOOTLOADER \
  2  // todo CHECK 2: implemented this way in IDE - probably means getting the bootloader version. Not implemented in firmware (LH)
#define VERSION_SIGNATURE \
  3  // todo CHECK 2: implemented under "Signature" in IDE. Not sure what to return for that. Not implemented in firmware (LH)
#define VERSION_BOARD_DETECT_SRC \
  4  // todo CHECK 2: This doesn't really fit under GetVersion, but its implemented there in the IDE - change or leave this way? (LH)
#define VERSION_BUILD 5

//Statuscodes
#define REPLY_OK 100
#define REPLY_CMD_LOADED 101
#define REPLY_CHKERR 1
#define REPLY_INVALID_CMD 2
#define REPLY_INVALID_TYPE 3
#define REPLY_INVALID_VALUE 4
#define REPLY_EEPROM_LOCKED 5
#define REPLY_CMD_NOT_AVAILABLE 6
#define REPLY_CMD_LOAD_ERROR 7
#define REPLY_WRITE_PROTECTED 8
#define REPLY_MAX_EXCEEDED 9
#define REPLY_DOWNLOAD_NOT_POSSIBLE 10
#define REPLY_CHIP_READ_FAILED 11
#define REPLY_DELAYED 128
#define REPLY_ACTIVE_COMM 129

// TMCL communication status
#define TMCL_RX_ERROR_NONE 0
#define TMCL_RX_ERROR_NODATA 1
#define TMCL_RX_ERROR_CHECKSUM 2

uint32_t vmax_position[2];

// TMCL request
typedef struct
{
  uint8_t Opcode;
  uint8_t Type;
  uint8_t Motor;
  uint32_t Error;
  union {
    uint8_t Byte[4];
    uint32_t UInt32;
    int32_t Int32;
    float_t Float32;
  } Value;
} TMCLCommandTypeDef;

// TMCL reply
typedef struct
{
  uint8_t Status;
  uint8_t Opcode;
  union {
    uint8_t Byte[4];
    uint32_t UInt32;
    int32_t Int32;
    float_t Float32;
  } Value;

  uint8_t Special[9];
  uint8_t
    IsSpecial;  // next transfer will not use the serial address and the checksum bytes - instead the whole datagram is filled with data (used to transmit ASCII version string)
} TMCLReplyTypeDef;

TMCLCommandTypeDef ActualCommand;
TMCLReplyTypeDef ActualReply;
//RXTXTypeDef interfaces[4];
uint32_t numberOfInterfaces;
uint32_t resetRequest = 0;

//esp32用
char RXTX[9];
char cnt = 0;
char write_cacel;
char ids_id;

//version
static void GetVersion(void)
{
  ActualReply.IsSpecial = 0;
  if (ActualCommand.Type == VERSION_FORMAT_ASCII) {
    ActualReply.IsSpecial = 1;
    ActualReply.Special[0] = SERIAL_HOST_ADDRESS;

    for (uint8_t i = 0; i < 8; i++) ActualReply.Special[i + 1] = VersionString[i];
  } else if (ActualCommand.Type == VERSION_FORMAT_BINARY) {
    uint8_t tmpVal;

    // module version high
    tmpVal = (uint8_t)VersionString[0] - '0';  // Ascii digit - '0' = digit value
    tmpVal *= 10;
    tmpVal += (uint8_t)VersionString[1] - '0';
    ActualReply.Value.Byte[3] = tmpVal;

    // module version low
    tmpVal = (uint8_t)VersionString[2] - '0';
    tmpVal *= 10;
    tmpVal += (uint8_t)VersionString[3] - '0';
    ActualReply.Value.Byte[2] = tmpVal;

    // fw version high
    ActualReply.Value.Byte[1] = (uint8_t)VersionString[5] - '0';

    // fw version low
    tmpVal = (uint8_t)VersionString[6] - '0';
    tmpVal *= 10;
    tmpVal += (uint8_t)VersionString[7] - '0';
    ActualReply.Value.Byte[0] = tmpVal;
  }
  //how were the boards detected?	// todo CHECK 2: Doesn't fit into GetVersion. Move somewhere else? Or maybe change GetVersion to GetBoardInfo or something (LH)
  else if (ActualCommand.Type == VERSION_BOARD_DETECT_SRC) {
  } else if (ActualCommand.Type == VERSION_BUILD) {
    ActualReply.Value.UInt32 = BUILD_VERSION;
  }
}

//get_paramter
static void GetGlobalParameter()
{
  ActualReply.IsSpecial = 0;
  switch (ActualCommand.Type) {
    case 1:  //VitalSignsMonitor.errors;
      ActualReply.Value.Int32 = 0;
      break;
    case 2:  //Evalboards.driverEnable
      ActualReply.Value.Int32 = 1;
      break;
    case 3:
      //			ActualReply.Value.Int32 = VitalSignsMonitor.debugMode;
      break;
    case 4:  //Board_supported
      ActualReply.Value.Int32 = (ID_STATE_DONE << 8) | ID_TMC5072;
      break;
    case 5:                         // Get hardware ID
      ActualReply.Value.Int32 = 2;  //hwid;
      break;
    case 6:
      //			ActualReply.Value.UInt32 = HAL.IOs->config->getState(HAL.IOs->pins->pins[ActualCommand.Motor]);
      break;
    case 7:
      //			ActualReply.Value.UInt32 = spi_getFrequency(&HAL.SPI->ch1);
      break;
    case 8:
      //			ActualReply.Value.UInt32 = spi_getFrequency(&HAL.SPI->ch2);
      break;
    default:
      ActualReply.Status = REPLY_INVALID_TYPE;
      break;
  }
}

//input
static void GetInput(void)
{
  ActualReply.IsSpecial = 0;
  switch (ActualCommand.Type) {
    case 0:
      //		ActualReply.Value.Int32 = *HAL.ADCs->AIN0;
      break;
    case 1:
      //		ActualReply.Value.Int32 = *HAL.ADCs->AIN1;
      break;
    case 2:
      //		ActualReply.Value.Int32 = *HAL.ADCs->AIN2;
      break;
    case 3:
      //		ActualReply.Value.Int32 = *HAL.ADCs->DIO4;
      break;
    case 4:
      //		ActualReply.Value.Int32 = *HAL.ADCs->DIO5;
      break;
    case 5:                                              //VitalSignsMonitor.VM
      ActualReply.Value.Int32 = getBatteryVolt() / 100;  //モータの電圧
      break;
    case 6:  // Raw VM ADC value, no scaling calculation done // todo QOL 2: Switch this case with case 5? That way we have the raw Values from 0-5, then 6 for scaled VM value. Requires IDE changes (LH)
             //		ActualReply.Value.Int32 = *HAL.ADCs->VM;
      break;
    case 7:
      //		ActualReply.Value.Int32 = *HAL.ADCs->AIN_EXT;
      break;
    default:
      //		ActualReply.Status = REPLY_INVALID_TYPE;
      break;
  }
}

void boardAssignment(void) {}

//コマンド実行
void ExecuteActualCommand()
{
  int32_t Actual_Position = 0;
  int32_t Read_Value = 0;
  int32_t tmp_Value = 0;

  ActualReply.Opcode = ActualCommand.Opcode;
  ActualReply.Status = REPLY_OK;
  ActualReply.Value.Int32 = ActualCommand.Value.Int32;

  if (ActualCommand.Error == TMCL_RX_ERROR_CHECKSUM) {
    ActualReply.Value.Int32 = 0;
    ActualReply.Status = REPLY_CHKERR;
    return;
  }

  switch (ActualCommand.Opcode) {
    case TMCL_ROR:  //right
      if (ActualCommand.Motor == 0) {
        TMC5072Write(TMC5072_VMAX1, abs(ActualCommand.Value.Int32));
        if (ActualCommand.Value.Int32 > 0) {
          TMC5072Write(TMC5072_RAMPMODE1, 1);
        } else {
          TMC5072Write(TMC5072_RAMPMODE1, 2);
        }
      } else {
        TMC5072Write(TMC5072_VMAX2, abs(ActualCommand.Value.Int32));
        if (ActualCommand.Value.Int32 > 0) {
          TMC5072Write(TMC5072_RAMPMODE2, 1);
        } else {
          TMC5072Write(TMC5072_RAMPMODE2, 2);
        }
      }
      break;
    case TMCL_ROL:  //left
      if (ActualCommand.Motor == 0) {
        TMC5072Write(TMC5072_VMAX1, abs(ActualCommand.Value.Int32));
        if (ActualCommand.Value.Int32 > 0) {
          TMC5072Write(TMC5072_RAMPMODE1, 2);
        } else {
          TMC5072Write(TMC5072_RAMPMODE1, 1);
        }
      } else {
        TMC5072Write(TMC5072_VMAX2, abs(ActualCommand.Value.Int32));
        if (ActualCommand.Value.Int32 > 0) {
          TMC5072Write(TMC5072_RAMPMODE2, 2);
        } else {
          TMC5072Write(TMC5072_RAMPMODE2, 1);
        }
      }
      break;
    case TMCL_MST:  //stop
      if (ActualCommand.Motor == 0) {
        TMC5072Write(TMC5072_VMAX1, 0);
      } else {
        TMC5072Write(TMC5072_VMAX2, 0);
      }
      break;
    case TMCL_MVP:
      // if function doesn't exist for ch1 try ch2
      switch (ActualCommand.Type) {
        case MVP_ABS:  // move absolute
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_RAMPMODE1, 0);  //TMC5072_MODE_POSITION
            TMC5072Write(TMC5072_VMAX1, vmax_position[0]);
            TMC5072Write(TMC5072_XTARGET1, abs(ActualCommand.Value.Int32));
          } else {
            TMC5072Write(TMC5072_RAMPMODE2, 0);  //TMC5072_MODE_POSITION
            TMC5072Write(TMC5072_VMAX2, vmax_position[0]);
            TMC5072Write(TMC5072_XTARGET2, abs(ActualCommand.Value.Int32));
          }
          break;
        case MVP_REL:  // move relative
          if (ActualCommand.Motor == 0) {
            Actual_Position += TMC5072Read(TMC5072_XACTUAL1, &ActualReply.Status);

            TMC5072Write(TMC5072_RAMPMODE1, 0);  //TMC5072_MODE_POSITION
            TMC5072Write(TMC5072_VMAX1, ActualReply.Value.Int32);
            TMC5072Write(TMC5072_XTARGET1, abs(Actual_Position));

            ActualReply.Value.Int32 = ActualCommand.Value.Int32;
          } else {
            Actual_Position += TMC5072Read(TMC5072_XACTUAL2, &ActualReply.Status);

            TMC5072Write(TMC5072_RAMPMODE2, 0);  //TMC5072_MODE_POSITION
            TMC5072Write(TMC5072_VMAX2, ActualReply.Value.Int32);
            TMC5072Write(TMC5072_XTARGET2, abs(Actual_Position));

            ActualReply.Value.Int32 = ActualCommand.Value.Int32;
          }
          break;
        case MVP_PRF:
          break;
        default:
          ActualReply.Status = REPLY_INVALID_TYPE;
          break;
      }
    case TMCL_SAP:
      switch (ActualCommand.Type) {
        case 0:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_XTARGET1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_XTARGET2, ActualCommand.Value.Int32);
          }
        case 1:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_XACTUAL1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_XACTUAL2, ActualCommand.Value.Int32);
          }
          break;
        case 2:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VMAX1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_VMAX2, ActualCommand.Value.Int32);
          }
          break;
        case 4:
          if (ActualCommand.Motor == 0) {
            vmax_position[ActualCommand.Motor] = abs(ActualCommand.Value.Int32);
            if (TMC5072Read(TMC5072_RAMPMODE1, &ActualReply.Status) == 0) {  //position mode
              TMC5072Write(TMC5072_VMAX1, vmax_position[ActualCommand.Motor]);
            }
          } else {
            vmax_position[ActualCommand.Motor] = abs(ActualCommand.Value.Int32);
            if (TMC5072Read(TMC5072_RAMPMODE2, &ActualReply.Status) == 0) {  //position mode
              TMC5072Write(TMC5072_VMAX2, vmax_position[ActualCommand.Motor]);
            }
          }
          break;
        case 5:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_AMAX1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_AMAX2, ActualCommand.Value.Int32);
          }
          break;
        case 6:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_IHOLD_IRUN1, ((Read_Value & (~(0x00001F00))) |
                                    (((ActualCommand.Value.Int32) << 8) & (0x00001F00))));
          } else {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_IHOLD_IRUN2, ((Read_Value & (~(0x00001F00))) |
                                    (((ActualCommand.Value.Int32) << 8) & (0x00001F00))));
          }
          break;

        case 7:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_IHOLD_IRUN1,
              ((Read_Value & (~(0x1F))) | (((ActualCommand.Value.Int32)) & (0x1F))));
          } else {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_IHOLD_IRUN2,
              ((Read_Value & (~(0x1F))) | (((ActualCommand.Value.Int32)) & (0x1F))));
          }
          break;
        case 12:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE1, ((Read_Value & (~(0x02))) |
                                 ((((ActualCommand.Value.Int32 > 0) ? 1 : 0) << 1) & (0x02))));
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE1, ((Read_Value & (~(0x08))) |
                                 ((((ActualCommand.Value.Int32 == 2) ? 1 : 0) << 3) & (0x08))));
          } else {
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE2, ((Read_Value & (~(0x02))) |
                                 ((((ActualCommand.Value.Int32 > 0) ? 1 : 0) << 1) & (0x02))));
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE2, ((Read_Value & (~(0x08))) |
                                 ((((ActualCommand.Value.Int32 == 2) ? 1 : 0) << 3) & (0x08))));
          }
          break;
        case 13:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE1,
              ((Read_Value & (~(0x01))) | ((((ActualCommand.Value.Int32 > 0) ? 1 : 0)) & (0x01))));
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE1, ((Read_Value & (~(0x04))) |
                                 ((((ActualCommand.Value.Int32 == 2) ? 1 : 0) << 2) & (0x04))));
          } else {
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE2,
              ((Read_Value & (~(0x01))) | ((((ActualCommand.Value.Int32 > 0) ? 1 : 0)) & (0x01))));
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE2, ((Read_Value & (~(0x04))) |
                                 ((((ActualCommand.Value.Int32 == 2) ? 1 : 0) << 2) & (0x04))));
          }
          break;
        case 14:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_SW_MODE1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_SW_MODE2, ActualCommand.Value.Int32);
          }
          break;
        case 15:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_A11, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_A12, ActualCommand.Value.Int32);
          }
          break;
        case 16:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_V11, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_V12, ActualCommand.Value.Int32);
          }
          break;
        case 17:  //0x11
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_DMAX1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_DMAX2, ActualCommand.Value.Int32);
          }
          break;
        case 18:  //0x12
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_D11, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_D12, ActualCommand.Value.Int32);
          }
          break;
        case 19:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VSTART1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_VSTART2, ActualCommand.Value.Int32);
          }
          break;
        case 20:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VSTOP1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_VSTOP2, ActualCommand.Value.Int32);
          }
          break;
        case 21:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_TZEROWAIT1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_TZEROWAIT2, ActualCommand.Value.Int32);
          }
          break;
        case 23:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VHIGH1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_VHIGH2, ActualCommand.Value.Int32);
          }
          break;
        case 24:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VDCMIN1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_VDCMIN2, ActualCommand.Value.Int32);
          }
          break;
        case 28:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x040000))) | (((ActualCommand.Value.Int32) << 18) & (0x040000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x040000))) | (((ActualCommand.Value.Int32) << 18) & (0x040000))));
          }
          break;
        case 140:
          switch (ActualCommand.Value.Int32) {
            case 1:
              ActualCommand.Value.Int32 = 8;
              break;
            case 2:
              ActualCommand.Value.Int32 = 7;
              break;
            case 4:
              ActualCommand.Value.Int32 = 6;
              break;
            case 8:
              ActualCommand.Value.Int32 = 5;
              break;
            case 16:
              ActualCommand.Value.Int32 = 4;
              break;
            case 32:
              ActualCommand.Value.Int32 = 3;
              break;
            case 64:
              ActualCommand.Value.Int32 = 2;
              break;
            case 128:
              ActualCommand.Value.Int32 = 1;
              break;
            case 256:
              ActualCommand.Value.Int32 = 0;
              break;
            default:
              ActualCommand.Value.Int32 = -1;
              break;
          }
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1, ((Read_Value & (~(0x0F000000))) |
                                  (((ActualCommand.Value.Int32) << 24) & (0x0F000000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2, ((Read_Value & (~(0x0F000000))) |
                                  (((ActualCommand.Value.Int32) << 24) & (0x0F000000))));
          }
          break;
        case 162:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x018000))) | (((ActualCommand.Value.Int32) << 15) & (0x018000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x018000))) | (((ActualCommand.Value.Int32) << 15) & (0x018000))));
          }
          break;
        case 163:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x2000))) | (((ActualCommand.Value.Int32) << 13) & (0x2000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x2000))) | (((ActualCommand.Value.Int32) << 13) & (0x2000))));
          }
          break;
        case 164:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x1000))) | (((ActualCommand.Value.Int32) << 12) & (0x1000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x1000))) | (((ActualCommand.Value.Int32) << 12) & (0x1000))));
          }
          break;
        case 165:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            Read_Value = (Read_Value & 0x4000) >> 14;
            if (Read_Value) {
              TMC5072Write(
                TMC5072_CHOPCONG1,
                ((Read_Value & (~(0x0780))) | (((ActualCommand.Value.Int32) << 7) & (0x0780))));
            } else {
              TMC5072Write(
                TMC5072_CHOPCONG1,
                ((((Read_Value & (~(0x0800))) | (((ActualCommand.Value.Int32) << 11) & (0x0800))) &
                  (1 << 3))
                   ? 1
                   : 0));
              TMC5072Write(
                TMC5072_CHOPCONG1,
                ((Read_Value & (~(0x70))) | (((ActualCommand.Value.Int32) << 4) & (0x70))));
            }
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            Read_Value = (Read_Value & 0x4000) >> 14;
            if (Read_Value) {
              TMC5072Write(
                TMC5072_CHOPCONG2,
                ((Read_Value & (~(0x0780))) | (((ActualCommand.Value.Int32) << 7) & (0x0780))));
            } else {
              TMC5072Write(
                TMC5072_CHOPCONG2,
                ((((Read_Value & (~(0x0800))) | (((ActualCommand.Value.Int32) << 11) & (0x0800))) &
                  (1 << 3))
                   ? 1
                   : 0));
              TMC5072Write(
                TMC5072_CHOPCONG2,
                ((Read_Value & (~(0x70))) | (((ActualCommand.Value.Int32) << 4) & (0x70))));
            }
          }
          break;
        case 166:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            Read_Value = (Read_Value & 0x4000) >> 14;
            if (Read_Value) {
              TMC5072Write(
                TMC5072_CHOPCONG1,
                ((Read_Value & (~(0x70))) | (((ActualCommand.Value.Int32) << 4) & (0x70))));
            } else {
              TMC5072Write(
                TMC5072_CHOPCONG1,
                ((Read_Value & (~(0x0780))) | (((ActualCommand.Value.Int32) << 7) & (0x0780))));
            }
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            Read_Value = (Read_Value & 0x4000) >> 14;
            if (Read_Value) {
              TMC5072Write(
                TMC5072_CHOPCONG2,
                ((Read_Value & (~(0x70))) | (((ActualCommand.Value.Int32) << 4) & (0x70))));
            } else {
              TMC5072Write(
                TMC5072_CHOPCONG2,
                ((Read_Value & (~(0x0780))) | (((ActualCommand.Value.Int32) << 7) & (0x0780))));
            }
          }
          break;
        case 167:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x0F))) | (((ActualCommand.Value.Int32)) & (0x0F))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x0F))) | (((ActualCommand.Value.Int32)) & (0x0F))));
          }
          break;
        case 168:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1,
              ((Read_Value & (~(0x8000))) | (((ActualCommand.Value.Int32) << 15) & (0x8000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2,
              ((Read_Value & (~(0x8000))) | (((ActualCommand.Value.Int32) << 15) & (0x8000))));
          }
          break;
        case 169:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1,
              ((Read_Value & (~(0x6000))) | (((ActualCommand.Value.Int32) << 13) & (0x6000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2,
              ((Read_Value & (~(0x6000))) | (((ActualCommand.Value.Int32) << 13) & (0x6000))));
          }
          break;
        case 170:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1,
              ((Read_Value & (~(0x6000))) | (((ActualCommand.Value.Int32) << 13) & (0x6000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2,
              ((Read_Value & (~(0x6000))) | (((ActualCommand.Value.Int32) << 13) & (0x6000))));
          }
          break;
        case 171:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1,
              ((Read_Value & (~(0x60))) | (((ActualCommand.Value.Int32) << 5) & (0x60))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2,
              ((Read_Value & (~(0x60))) | (((ActualCommand.Value.Int32) << 5) & (0x60))));
          }
          break;
        case 172:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1,
              ((Read_Value & (~(0x0F))) | (((ActualCommand.Value.Int32)) & (0x0F))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2,
              ((Read_Value & (~(0x0F))) | (((ActualCommand.Value.Int32)) & (0x0F))));
          }
          break;
        case 173:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1, ((Read_Value & (~(0x01000000))) |
                                  (((ActualCommand.Value.Int32) << 15) & (0x01000000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2, ((Read_Value & (~(0x01000000))) |
                                  (((ActualCommand.Value.Int32) << 15) & (0x01000000))));
          }
          break;
        case 174:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF1,
              ((Read_Value & (~(0x7F0000))) | (((ActualCommand.Value.Int32) << 16) & (0x7F0000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_COOLCONF2,
              ((Read_Value & (~(0x7F0000))) | (((ActualCommand.Value.Int32) << 16) & (0x7F0000))));
          }
          break;
        case 179:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x020000))) | (((ActualCommand.Value.Int32) << 17) & (0x020000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x020000))) | (((ActualCommand.Value.Int32) << 17) & (0x020000))));
          }
          break;
        case 181:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VCOOLTHRS1, ActualCommand.Value.Int32);
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE1,
              (((Read_Value & (~(0x0400))) | (((ActualCommand.Value.Int32) << 10) & (0x0400)))
                 ? 1
                 : 0));
          } else {
            TMC5072Write(TMC5072_VCOOLTHRS2, ActualCommand.Value.Int32);
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_SW_MODE2,
              (((Read_Value & (~(0x0400))) | (((ActualCommand.Value.Int32) << 10) & (0x0400)))
                 ? 1
                 : 0));
          }
          break;
        case 182:
          if (ActualCommand.Motor == 0) {
            TMC5072Write(TMC5072_VCOOLTHRS1, ActualCommand.Value.Int32);
          } else {
            TMC5072Write(TMC5072_VCOOLTHRS2, ActualCommand.Value.Int32);
          }
          break;
        case 184:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG1,
              ((Read_Value & (~(0x2000))) | (((ActualCommand.Value.Int32) << 13) & (0x2000))));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            TMC5072Write(
              TMC5072_CHOPCONG2,
              ((Read_Value & (~(0x2000))) | (((ActualCommand.Value.Int32) << 13) & (0x2000))));
          }
          break;
        default:
          break;
      }
      break;
    case TMCL_GAP:
      switch (ActualCommand.Type) {
        case 0:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_XTARGET1, &ActualReply.Status);
            ActualReply.Status &= 0xfe;  //reset　解除
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_XTARGET2, &ActualReply.Status);
          }
          break;
        case 1:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_XACTUAL1, &ActualReply.Status);
            ActualReply.Status &= 0xfe;  //reset　解除
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_XACTUAL2, &ActualReply.Status);
          }
          break;
        case 2:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VMAX1, &ActualReply.Status);
            ActualReply.Status &= 0xfe;  //reset　解除
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VMAX2, &ActualReply.Status);
          }
          break;
        case 3:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 =
              CAST_Sn_TO_S32(TMC5072Read(TMC5072_VACTUAL1, &ActualReply.Status), 24);
          } else {
            ActualReply.Value.Int32 =
              CAST_Sn_TO_S32(TMC5072Read(TMC5072_VACTUAL2, &ActualReply.Status), 24);
          }
          break;
        case 4:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = vmax_position[ActualCommand.Motor];
          } else {
            ActualReply.Value.Int32 = vmax_position[ActualCommand.Motor];
          }
          break;
        case 5:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_AMAX1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_AMAX2, &ActualReply.Status);
          }
          break;
        case 6:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1F00)) >> 8);
          } else {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1F00)) >> 8);
          }
          break;
        case 7:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1F)));
          } else {
            Read_Value = TMC5072Read(TMC5072_IHOLD_IRUN2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1F)));
          }

          break;
        case 8:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_RAMP_STAT1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x0200)) >> 9);
          } else {
            Read_Value = TMC5072Read(TMC5072_RAMP_STAT2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x0200)) >> 9);
          }
          break;
        case 10:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_RAMP_STAT1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x02)) >> 1);
          } else {
            Read_Value = TMC5072Read(TMC5072_RAMP_STAT2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x02)) >> 1);
          }
          break;
        case 11:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_RAMP_STAT1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x01)));
          } else {
            Read_Value = TMC5072Read(TMC5072_RAMP_STAT2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x01)));
          }
          break;
        case 12:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x02)) >> 1);
          } else {
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x02)) >> 1);
          }
          break;
        case 13:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x01)));
          } else {
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x01)));
          }
          break;
        case 14:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
          }
          break;
        case 15:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_A11, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_A12, &ActualReply.Status);
          }
          break;
        case 16:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_V11, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_V12, &ActualReply.Status);
          }
          break;
        case 17:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_DMAX1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_DMAX2, &ActualReply.Status);
          }
          break;
        case 18:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_D11, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_D12, &ActualReply.Status);
          }
          break;
        case 19:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VSTART1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VSTART2, &ActualReply.Status);
          }
          break;
        case 20:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VSTOP1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VSTOP2, &ActualReply.Status);
          }
          break;
        case 21:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_TZEROWAIT1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_TZEROWAIT2, &ActualReply.Status);
          }
          break;
        case 22:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VCOOLTHRS1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VCOOLTHRS2, &ActualReply.Status);
          }
          break;
        case 23:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VHIGH1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VHIGH2, &ActualReply.Status);
          }
          break;
        case 24:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VDCMIN1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VDCMIN2, &ActualReply.Status);
          }
          break;
        case 28:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x040000)) >> 18);
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x040000)) >> 18);
          }
          break;
        case 140:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = 256 >> ((Read_Value & (0x0F)));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = 256 >> ((Read_Value & (0x0F)));
          }
          break;
        case 162:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x018000)) >> 15);
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x018000)) >> 15);
          }
          break;
        case 163:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x2000)) >> 13);
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x2000)) >> 13);
          }
          break;
        case 164:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1000)) >> 12);
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1000)) >> 12);
          }
          break;
        case 165:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            Read_Value = ((Read_Value & (0x4000)) >> 14);
            if (Read_Value) {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
              ActualReply.Value.Int32 = ((Read_Value & (0x0780)) >> 7);
            } else {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
              Read_Value |= ((((Read_Value & (0x70)) >> 4) & (1 << 11)) << 3);
              ActualReply.Value.Int32 = Read_Value;
            }
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            Read_Value = ((Read_Value & (0x4000)) >> 14);
            if (Read_Value) {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
              ActualReply.Value.Int32 = ((Read_Value & (0x0780)) >> 7);
            } else {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
              Read_Value |= ((((Read_Value & (0x70)) >> 4) & (1 << 11)) << 3);
              ActualReply.Value.Int32 = Read_Value;
            }
          }
          break;
        case 166:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            Read_Value = ((Read_Value & (0x4000)) >> 14);
            if (Read_Value) {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
              ActualReply.Value.Int32 = ((Read_Value & (0x70)) >> 4);
            } else {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
              Read_Value = ((Read_Value & (0x0780)) >> 7);
              tmp_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
              tmp_Value = ((Read_Value & (0x0800)) >> 11);
              Read_Value |= tmp_Value << 3;
              ActualReply.Value.Int32 = Read_Value;
            }
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            Read_Value = ((Read_Value & (0x4000)) >> 14);
            if (Read_Value) {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
              ActualReply.Value.Int32 = ((Read_Value & (0x70)) >> 4);
            } else {
              Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
              Read_Value = ((Read_Value & (0x0780)) >> 7);
              tmp_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
              tmp_Value = ((Read_Value & (0x0800)) >> 11);
              Read_Value |= tmp_Value << 3;
              ActualReply.Value.Int32 = Read_Value;
            }
          }
          break;
        case 167:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x0F)));
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x0F)));
          }
          break;
        case 168:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x8000)) >> 15);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x8000)) >> 15);
          }
          break;
        case 169:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x6000)) >> 13);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x6000)) >> 13);
          }
          break;
        case 170:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x0F00)) >> 8);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x0F00)) >> 8);
          }
          break;
        case 171:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x60)) >> 5);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x60)) >> 5);
          }
          break;
        case 172:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x8000)) >> 15);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x8000)) >> 15);
          }
          break;
        case 173:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x01000000)) >> 24);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x01000000)) >> 24);
          }
          break;
        case 174:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_COOLCONF1, &ActualReply.Status);
            ActualReply.Value.Int32 = CAST_Sn_TO_S32(((Read_Value & (0x7F0000)) >> 16), 7);
          } else {
            Read_Value = TMC5072Read(TMC5072_COOLCONF2, &ActualReply.Status);
            ActualReply.Value.Int32 = CAST_Sn_TO_S32(((Read_Value & (0x7F0000)) >> 16), 7);
          }
          break;
        case 179:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x020000)) >> 17);
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x020000)) >> 17);
          }
          break;
        case 180:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_DRVSTATUS1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1F0000)) >> 16);
          } else {
            Read_Value = TMC5072Read(TMC5072_DRVSTATUS2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x1F0000)) >> 16);
          }
          break;
        case 181:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_SW_MODE1, &ActualReply.Status);
            if ((Read_Value & (0x0400)) >> 10) {
              ActualReply.Value.Int32 = TMC5072Read(TMC5072_VCOOLTHRS1, &ActualReply.Status);
            } else {
              ActualReply.Value.Int32 = 0;
            }
          } else {
            Read_Value = TMC5072Read(TMC5072_SW_MODE2, &ActualReply.Status);
            if ((Read_Value & (0x0400)) >> 10) {
              ActualReply.Value.Int32 = TMC5072Read(TMC5072_VCOOLTHRS2, &ActualReply.Status);
            } else {
              ActualReply.Value.Int32 = 0;
            }
          }
          break;
        case 182:
          if (ActualCommand.Motor == 0) {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VCOOLTHRS1, &ActualReply.Status);
          } else {
            ActualReply.Value.Int32 = TMC5072Read(TMC5072_VCOOLTHRS2, &ActualReply.Status);
          }
          break;
        case 184:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x2000)) >> 13);
          } else {
            Read_Value = TMC5072Read(TMC5072_CHOPCONG2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x2000)) >> 13);
          }
          break;
        case 206:
          if (ActualCommand.Motor == 0) {
            Read_Value = TMC5072Read(TMC5072_DRVSTATUS1, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x03FF)));
          } else {
            Read_Value = TMC5072Read(TMC5072_DRVSTATUS2, &ActualReply.Status);
            ActualReply.Value.Int32 = ((Read_Value & (0x03FF)));
          }
          break;
        default:
          break;
      }
      break;
    case TMCL_SGP:

      break;
    case TMCL_GGP:
      GetGlobalParameter();
      break;
    case TMCL_GIO:
      GetInput();
      break;
    case TMCL_UF0:

      break;
    case TMCL_UF1:

      break;
    case TMCL_UF2:

      break;
    case TMCL_UF4:

      break;
    case TMCL_UF5:
      break;
    case TMCL_UF6:
      break;
    case TMCL_UF8:
      break;
    case TMCL_GetVersion:
      GetVersion();
      break;
    case TMCL_GetIds:
      boardAssignment();
      break;
    case TMCL_UF_CH1:
      break;
    case TMCL_UF_CH2:
      break;
    case TMCL_writeRegisterChannel_1:
      TMC5072Write(ActualCommand.Type, ActualCommand.Value.Int32);
      break;
    case TMCL_writeRegisterChannel_2:
      break;
    case TMCL_readRegisterChannel_1:
      ActualReply.Value.Int32 = TMC5072Read_no_status(ActualCommand.Type);
      break;
    case TMCL_readRegisterChannel_2:
      break;
    case TMCL_BoardMeasuredSpeed:
      break;
    case TMCL_BoardError:
      break;
    case TMCL_BoardReset:
      break;
    case TMCL_WLAN:
      break;
    case TMCL_RamDebug:
      break;
    case TMCL_OTP:
      break;
    case TMCL_MIN:
      break;
    case TMCL_MAX:
      break;
    case TMCL_Boot:
      if (ActualCommand.Type != 0x81) break;
      if (ActualCommand.Motor != 0x92) break;
      if (ActualCommand.Value.Byte[3] != 0xA3) break;
      if (ActualCommand.Value.Byte[2] != 0xB4) break;
      if (ActualCommand.Value.Byte[1] != 0xC5) break;
      if (ActualCommand.Value.Byte[0] != 0xD6) break;
      break;
    case TMCL_SoftwareReset:
      break;
    default:
      ActualReply.Status = REPLY_INVALID_CMD;
      break;
  }
}

//データ送信
void tx(void)
{
  uint8_t checkSum = 0;

  uint8_t reply[9];

  if (ActualReply.IsSpecial) {
    for (uint8_t i = 0; i < 9; i++) reply[i] = ActualReply.Special[i];
  } else {
    checkSum += SERIAL_HOST_ADDRESS;
    checkSum += SERIAL_MODULE_ADDRESS;
    checkSum += ActualReply.Status;
    checkSum += ActualReply.Opcode;
    checkSum += ActualReply.Value.Byte[3];
    checkSum += ActualReply.Value.Byte[2];
    checkSum += ActualReply.Value.Byte[1];
    checkSum += ActualReply.Value.Byte[0];

    reply[0] = SERIAL_HOST_ADDRESS;
    reply[1] = SERIAL_MODULE_ADDRESS;
    reply[2] = ActualReply.Status;
    reply[3] = ActualReply.Opcode;
    reply[4] = ActualReply.Value.Byte[3];
    reply[5] = ActualReply.Value.Byte[2];
    reply[6] = ActualReply.Value.Byte[1];
    reply[7] = ActualReply.Value.Byte[0];
    reply[8] = checkSum;
  }
  for (int i = 0; i < 9; i++) {
    Serial.write(reply[i]);  //to TMCL-IDE
  }
  ActualCommand.Error = TMCL_RX_ERROR_NODATA;
}

//データ受信
void rx(char * RXTX)
{
  uint8_t checkSum = 0;
  uint8_t cmd[9];

  for (int i = 0; i < 9; i++) {
    cmd[i] = RXTX[i];
  }
  for (uint8_t i = 0; i < 8; i++)  //受信したデータでchecksumを計算
    checkSum += cmd[i];

  if (checkSum != cmd[8])  //チェックサムの確認
  {
    ActualCommand.Error = TMCL_RX_ERROR_CHECKSUM;
    return;
  }

  ActualCommand.Opcode = cmd[1];
  ActualCommand.Type = cmd[2];
  ActualCommand.Motor = cmd[3];
  ActualCommand.Value.Byte[3] = cmd[4];
  ActualCommand.Value.Byte[2] = cmd[5];
  ActualCommand.Value.Byte[1] = cmd[6];
  ActualCommand.Value.Byte[0] = cmd[7];
  ActualCommand.Error = TMCL_RX_ERROR_NONE;
}

void tmcl_init(void)
{
  ActualCommand.Error = TMCL_RX_ERROR_NODATA;
  TMC5072Setting(VELOCITY);
  //  TMC5072Setting(STEPDIR);
  TMC5072Write(
    TMC5072_CHOPCONG1, 0x00000001);  //MRES=0 1/256step TOFFTime=1 1以上でないとモータが動作しない
  TMC5072Write(TMC5072_CHOPCONG2, 0x00000001);
  TMC5072Write(TMC5072_GCONF, 0x08);  //エンコーダー未使用
  TMC5072Write(
    TMC5072_AMAX1,
    2000 / TMC5072_VELOCITY);  //TMCL_RORで速度指令だけで回るように加速度を0から任意値に設定
  TMC5072Write(TMC5072_AMAX2, 2000 / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_A11, 100);  //Evalboad default value
  TMC5072Write(TMC5072_A12, 100);  //Evalboad default value
  TMC5072Write(TMC5072_D11, 200);
  TMC5072Write(TMC5072_D12, 200);

  TMC5072Write(TMC5072_VMAX1, 0 / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2, 0 / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VSTART1, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VSTART2, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
  TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化
}

//データ受信、コマンド実行
void tmcl_process(void)
{
  char temp, pre_temp;
  static char view_flag = 0;

  if (ActualCommand.Error != TMCL_RX_ERROR_NODATA) {
    tx();
  }

  if (Serial.available()) {  //from TMCL-IDE
    RXTX[cnt] = Serial.read();
    cnt++;
    if (cnt == 9) {
      cnt = 0;
      view_flag = 0;
      rx(&RXTX[0]);
      if (ActualCommand.Error != TMCL_RX_ERROR_NODATA) {
        ExecuteActualCommand();
      }
    }
  }
}
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

#define TMC_ACCESS_READ        0x01
#define TMC_IS_READABLE(x)    ((x) & TMC_ACCESS_READ)


int shadowRegister[0x80]={0};

unsigned int TMC5072Read_no_status(unsigned char add)
{
  unsigned char data[5];

  if(!TMC_IS_READABLE(tmc5072_defaultRegisterAccess[add])){
    return shadowRegister[add];
  }


  data[0] = add | TMC5072_READ;
  data[1] = 0xff;
  data[2] = 0xff;
  data[3] = 0xff;
  data[4] = 0xff;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI.pinSS(), LOW);
  SPI.transfer(data, 5);
  digitalWrite(SPI.pinSS(), HIGH);
  SPI.endTransaction();

  data[0] = add | TMC5072_READ;
  data[1] = 0xff;
  data[2] = 0xff;
  data[3] = 0xff;
  data[4] = 0xff;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI.pinSS(), LOW);
  SPI.transfer(data, 5);
  digitalWrite(SPI.pinSS(), HIGH);
  SPI.endTransaction();

  return ((unsigned int)data[1] << 24) | ((unsigned int)data[2] << 16) |
         ((unsigned int)data[3] << 8) | ((unsigned int)data[4]);
}

unsigned int TMC5072Read(unsigned char add, uint8_t * status)
{
  unsigned char data[5];

  data[0] = add | TMC5072_READ;
  data[1] = 0xff;
  data[2] = 0xff;
  data[3] = 0xff;
  data[4] = 0xff;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI.pinSS(), LOW);
  SPI.transfer(data, 5);
  digitalWrite(SPI.pinSS(), HIGH);
  SPI.endTransaction();

  data[0] = add | TMC5072_READ;
  data[1] = 0xff;
  data[2] = 0xff;
  data[3] = 0xff;
  data[4] = 0xff;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI.pinSS(), LOW);
  SPI.transfer(data, 5);
  digitalWrite(SPI.pinSS(), HIGH);
  SPI.endTransaction();

  *status = data[0];

  return ((unsigned int)data[1] << 24) | ((unsigned int)data[2] << 16) |
         ((unsigned int)data[3] << 8) | ((unsigned int)data[4]);
}

void TMC5072Write(unsigned char add, unsigned int writedata)
{
  unsigned char data[5];

  data[0] = add | TMC5072_WRITE;
  data[1] = (writedata >> 24) & 0x0ff;
  data[2] = (writedata >> 16) & 0xff;
  data[3] = (writedata >> 8) & 0xff;
  data[4] = writedata;

  shadowRegister[add] = writedata;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI.pinSS(), LOW);
  SPI.transfer(data, 5);
  digitalWrite(SPI.pinSS(), HIGH);
  SPI.endTransaction();
}

void TMC5072Setting(t_TMC5072Mode mode)
{
  switch (mode) {
    case STEPDIR:  //Ramp Generator velocity-modeに近い使い方
      TMC5072Write(TMC5072_IHOLD_IRUN1, 0x00071703);  //IHOLDDELAY=7 IRUN=17/32 IHOLD=3/32
      TMC5072Write(TMC5072_IHOLD_IRUN2, 0x00071703);
      TMC5072Write(TMC5072_CHOPCONG1, 0x07000001);  //MRES=7 1/2step TOFFTime=1 1以上でないとモータが動作しない
      TMC5072Write(TMC5072_CHOPCONG2, 0x07000001);
      TMC5072Write(TMC5072_GCONF, 0xe);  //step1dir,step2dir enable
      break;
    case SIXPOINT:  //position-mode 加速度と速度を指定する
      TMC5072Write(TMC5072_IHOLD_IRUN1, 0x00071703);  //IHOLDDELAY=7 IRUN=17/32 IHOLD=3/32
      TMC5072Write(TMC5072_IHOLD_IRUN2, 0x00071703);
      TMC5072Write(TMC5072_CHOPCONG1,0x00000001);  //MRES=0 1/256step TOFFTime=1 1以上でないとモータが動作しない
      TMC5072Write(TMC5072_CHOPCONG2, 0x00000001);
      TMC5072Write(TMC5072_RAMPMODE1, 0x00);  //Position mode
      TMC5072Write(TMC5072_RAMPMODE2, 0x00);  //Position mode
      TMC5072Write(TMC5072_GCONF, 0x308);  //エンコーダー未使用 shaft inverse
      break;
    case VELOCITY:
      TMC5072Write(TMC5072_IHOLD_IRUN1, 0x00071703);  //IHOLDDELAY=7 IRUN=17/32 IHOLD=3/32
      TMC5072Write(TMC5072_IHOLD_IRUN2, 0x00071703);
      TMC5072Write(TMC5072_CHOPCONG1,0x00000001);  //MRES=0 1/256step TOFFTime=1 1以上でないとモータが動作しない
      TMC5072Write(TMC5072_CHOPCONG2, 0x00000001);
      TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
      TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)
      TMC5072Write(TMC5072_GCONF, 0x308);  //エンコーダー未使用  shaft inverse
      TMC5072Write(TMC5072_A11,    0);      
      TMC5072Write(TMC5072_A12,    0);
      TMC5072Write(TMC5072_V11,    0);
      TMC5072Write(TMC5072_V12,    0);
      TMC5072Write(TMC5072_DMAX1,  0);
      TMC5072Write(TMC5072_DMAX2,  0);
      TMC5072Write(TMC5072_AMAX1,  5000/TMC5072_ACCELE);
      TMC5072Write(TMC5072_AMAX2,  5000/TMC5072_ACCELE);
      TMC5072Write(TMC5072_VSTART1,MIN_SPEED/TMC5072_VELOCITY);
      TMC5072Write(TMC5072_VSTART2,MIN_SPEED/TMC5072_VELOCITY);      
      break;
  }
}

void TMC5072Init(void)
{
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);
  pinMode(SPI.pinSS(), OUTPUT);

  TMC5072Write(TMC5072_XACTUAL1,0);//初期化
  TMC5072Write(TMC5072_XACTUAL2,0);//初期化
  TMC5072Write(TMC5072_VSTART1,0);
  TMC5072Write(TMC5072_VSTART2,0);
  TMC5072Write(TMC5072_A11,    0);      
  TMC5072Write(TMC5072_A12,    0);
  TMC5072Write(TMC5072_V11,    0);
  TMC5072Write(TMC5072_V12,    0);
  TMC5072Write(TMC5072_AMAX1,  0);
  TMC5072Write(TMC5072_AMAX2,  0);
  TMC5072Write(TMC5072_VMAX1,  0);
  TMC5072Write(TMC5072_VMAX2,  0);
  TMC5072Write(TMC5072_DMAX1,  0);
  TMC5072Write(TMC5072_DMAX2,  0);
  TMC5072Write(TMC5072_D11,    0);      
  TMC5072Write(TMC5072_D12,    0);
  TMC5072Write(TMC5072_VSTOP1, 0);
  TMC5072Write(TMC5072_VSTOP2, 0);
  TMC5072Write(TMC5072_XTARGET1,0);
  TMC5072Write(TMC5072_XTARGET2,0);
  TMC5072Setting(VELOCITY);
}

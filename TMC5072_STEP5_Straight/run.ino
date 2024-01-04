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

void TMC5072Straignt(t_TMC5072_position tmc5072_position){
  TMC5072Write(TMC5072_XACTUAL1,0);//初期化
  TMC5072Write(TMC5072_XACTUAL2,0);//初期化
  TMC5072Write(TMC5072_VSTART1,(int)(tmc5072_position.init_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_VSTART2,(int)(tmc5072_position.init_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_A11,    (int)(tmc5072_position.a1_accel/TMC5072_ACCELE));      
  TMC5072Write(TMC5072_A12,    (int)(tmc5072_position.a1_accel/TMC5072_ACCELE));
  TMC5072Write(TMC5072_V11,    (int)(tmc5072_position.v1_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_V12,    (int)(tmc5072_position.v1_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_AMAX1,  (int)(tmc5072_position.amax_aceel/TMC5072_ACCELE));
  TMC5072Write(TMC5072_AMAX2,  (int)(tmc5072_position.amax_aceel/TMC5072_ACCELE));
  TMC5072Write(TMC5072_VMAX1,  (int)(tmc5072_position.vmax_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_VMAX2,  (int)(tmc5072_position.vmax_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_DMAX1,  (int)(tmc5072_position.amax_aceel/TMC5072_ACCELE));
  TMC5072Write(TMC5072_DMAX2,  (int)(tmc5072_position.amax_aceel/TMC5072_ACCELE));
  TMC5072Write(TMC5072_D11,    (int)(tmc5072_position.a1_accel/TMC5072_ACCELE));      
  TMC5072Write(TMC5072_D12,    (int)(tmc5072_position.a1_accel/TMC5072_ACCELE));
  TMC5072Write(TMC5072_VSTOP1, (int)(tmc5072_position.finish_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_VSTOP2, (int)(tmc5072_position.finish_speed/TMC5072_VELOCITY));
  TMC5072Write(TMC5072_XTARGET1,(int)(tmc5072_position.len/TMC5072_PULSE));
  TMC5072Write(TMC5072_XTARGET2,(int)(tmc5072_position.len/TMC5072_PULSE));

  while( (int)((abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) + abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2)))/2.0*TMC5072_PULSE ) < ((int)(tmc5072_position.len)-1)){
    continue;
  };
  TMC5072Write(TMC5072_V11,    0);
  TMC5072Write(TMC5072_V12,    0);
  TMC5072Write(TMC5072_VMAX1,  0);
  TMC5072Write(TMC5072_VMAX2,  0);
}

void accelerate_STEPDIR(int len, int tar_speed)
{
  int obj_step;

  g_max_speed = tar_speed;
  g_accel = 1.5;
  g_step_r = g_step_l = 0;
  g_speed = g_min_speed = MIN_SPEED;  
  g_step_hz_r = g_step_hz_l = (unsigned short)(g_speed / PULSE);

  obj_step = (int)((float)len * 2.0 / PULSE);
  digitalWrite(CW_R, HIGH);
  digitalWrite(CW_L, HIGH);
  g_motor_move = 1;

  while  ((g_step_r + g_step_l) < obj_step)  {
    continue;
  }
}

void accelerate_VELOCITY(int len, int tar_speed)
{
  int obj_step;

  g_max_speed = tar_speed;
  g_accel = 1.5;
  TMC5072Write(TMC5072_XACTUAL1,0);//初期化
  TMC5072Write(TMC5072_XACTUAL2,0);//初期化

  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);
  TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)
  g_motor_move = 1;

  TMC5072Write(TMC5072_VMAX1,  g_max_speed/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2,  g_max_speed/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_AMAX1,  g_accel*1000/TMC5072_ACCELE);
  TMC5072Write(TMC5072_AMAX2,  g_accel*1000/TMC5072_ACCELE);
  TMC5072Write(TMC5072_VSTART1,MIN_SPEED/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VSTART2,MIN_SPEED/TMC5072_VELOCITY);

  while( (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) + abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2)))<obj_step) {
    continue;
  }
}



void oneStep_STEPDIR(int len, int tar_speed)
{
  int obj_step;
  g_max_speed = tar_speed;
  g_accel = 0.0;
  g_step_r = g_step_l = 0;
  g_speed = g_min_speed= tar_speed;
  g_step_hz_r = g_step_hz_l = (unsigned short)(g_speed / PULSE);  
  obj_step = (int)((float)len * 2.0 / PULSE);
  digitalWrite(CW_R, HIGH);
  digitalWrite(CW_L, HIGH);

  while ((g_step_r + g_step_l) < obj_step) {
    continue;
  }
}


void oneStep_VELOCITY(int len, int tar_speed)
{
  int obj_step;
  g_max_speed = tar_speed;
  g_accel = 0.0;
  TMC5072Write(TMC5072_XACTUAL1,0);//初期化
  TMC5072Write(TMC5072_XACTUAL2,0);//初期化
  TMC5072Write(TMC5072_VMAX1,  g_max_speed/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2,  g_max_speed/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_AMAX1,  g_accel*1000/TMC5072_ACCELE);
  TMC5072Write(TMC5072_AMAX2,  g_accel*1000/TMC5072_ACCELE);    
  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);
  TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)


  while( (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) + abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2)))<obj_step) {
    continue;
  }
}


void decelerate_STEPDIR(int len, int tar_speed)
{
  int obj_step;
  g_max_speed = tar_speed;
  g_accel = 1.5;
  g_step_r = g_step_l = 0;
  g_speed = g_min_speed = tar_speed;
  g_step_hz_r = g_step_hz_l = (unsigned short)(g_speed / PULSE);
  obj_step = (int)((float)len * 2.0 / PULSE);
  digitalWrite(CW_R, HIGH);
  digitalWrite(CW_L, HIGH);

  while ((len - (g_step_r + g_step_l) / 2.0 * PULSE) >
          (((g_speed * g_speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * g_accel))) {
    continue;
  }
  g_accel = -1.5;
  g_min_speed = MIN_SPEED;

  while ((g_step_r + g_step_l) < obj_step) {
    continue;
  }

  g_motor_move = 0;

}

void decelerate_VELOCITY(int len, int tar_speed)
{
  int obj_step;
  g_max_speed = tar_speed;
  g_accel = 1.5;
  TMC5072Write(TMC5072_XACTUAL1,0);//初期化
  TMC5072Write(TMC5072_XACTUAL2,0);//初期化
  TMC5072Write(TMC5072_VMAX1,  g_max_speed/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2,  g_max_speed/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_AMAX1,  g_accel*1000/TMC5072_ACCELE);
  TMC5072Write(TMC5072_AMAX2,  g_accel*1000/TMC5072_ACCELE);    
  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);
  TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)  

  while(len - (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) + abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2)))/2.0 *TMC5072_PULSE 
        > ((((int)(TMC5072Read_no_status(TMC5072_VACTUAL1)*TMC5072_VELOCITY*TMC5072Read_no_status(TMC5072_VACTUAL1)*TMC5072_VELOCITY) ) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * g_accel)))  {
    continue;          
  }

  
  TMC5072Write(TMC5072_VMAX1,  MIN_SPEED/TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2,  MIN_SPEED/TMC5072_VELOCITY);

  while( (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) + abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2)))<obj_step) {
    continue;
  }

  g_motor_move = 0;

}

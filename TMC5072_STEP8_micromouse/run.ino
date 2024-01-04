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

void straight(int len, int init_speed, int max_speed, int finish_speed)
{
  int obj_step;
  controlInterruptStop();
  g_max_speed = max_speed;
  g_accel = SEARCH_ACCEL;

  if (init_speed < MIN_SPEED) {
    g_speed = MIN_SPEED;
    TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
    TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化
  } else {
    g_speed = init_speed;
  }
  if (finish_speed < MIN_SPEED) {
    finish_speed = MIN_SPEED;
  }
  if (init_speed < finish_speed) {
    g_min_speed = MIN_SPEED;
  } else {
    g_min_speed = finish_speed;
  }

  g_con_wall.enable = true;

  TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_VMAX1, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_AMAX1, 10000 / TMC5072_ACCELE);
  TMC5072Write(TMC5072_AMAX2, 10000 / TMC5072_ACCELE);
  TMC5072Write(TMC5072_VSTART1, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VSTART2, MIN_SPEED / TMC5072_VELOCITY);
  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);

  controlInterruptStart();

  g_motor_move = 1;

  while (len - (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
                abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) /
                 2.0 * TMC5072_PULSE >
         (((g_speed * g_speed) - (finish_speed * finish_speed)) / (2.0 * 1000.0 * SEARCH_ACCEL))) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }

  g_accel = -1.0 * SEARCH_ACCEL;

  while ((abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
          abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) < obj_step) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }

  if (finish_speed == SEARCH_SPEED) {
    controlInterruptStop();
    g_max_speed = g_min_speed = g_speed = finish_speed;
    g_accel = 0.0;
    TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
    TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化
    controlInterruptStart();
  } else {
    TMC5072Write(TMC5072_VMAX1, 0);
    TMC5072Write(TMC5072_VMAX2, 0);
    g_motor_move = 0;
  }
}

void accelerate(int len, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  g_con_wall.enable = true;
  g_motor_move = 1;
  g_max_speed = finish_speed;
  g_accel = SEARCH_ACCEL;
  g_speed = g_min_speed = MIN_SPEED;

  TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)
  TMC5072Write(TMC5072_VMAX1, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
  TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化

  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);
  controlInterruptStart();

  while ((abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
          abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) < obj_step) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }
  controlInterruptStop();
  TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
  TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化
  TMC5072Write(TMC5072_VMAX1, finish_speed / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2, finish_speed / TMC5072_VELOCITY);
  g_max_speed = g_min_speed = g_speed = finish_speed;
  g_accel = 0.0;
  controlInterruptStart();
}

void oneStep(int len, int tar_speed)
{
  int obj_step;
  controlInterruptStop();
  g_speed = g_min_speed = g_max_speed = tar_speed;
  g_accel = 0.0;
  g_con_wall.enable = true;

  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);
  controlInterruptStart();

  while ((abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
          abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) < obj_step) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }
  controlInterruptStop();
  TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
  TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化
  TMC5072Write(TMC5072_VMAX1, tar_speed / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2, tar_speed / TMC5072_VELOCITY);
  g_max_speed = g_min_speed = g_speed = tar_speed;
  g_accel = 0.0;
  controlInterruptStart();
}

void decelerate(int len, int tar_speed)
{
  int obj_step;
  controlInterruptStop();
  g_max_speed = tar_speed;
  g_accel = 0.0;
  g_speed = g_min_speed = tar_speed;
  g_con_wall.enable = true;

  obj_step = (int)((float)len * 2.0 / TMC5072_PULSE);
  controlInterruptStart();
  while (len - (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
                abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) /
                 2.0 * TMC5072_PULSE >
         (((g_speed * g_speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * SEARCH_ACCEL))) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }
  g_accel = -1.0 * SEARCH_ACCEL;
  g_min_speed = MIN_SPEED;

  while ((abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
          abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) < obj_step) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }
  TMC5072Write(TMC5072_VMAX1, 0);
  TMC5072Write(TMC5072_VMAX2, 0);

  g_motor_move = 0;

  delay(300);
}

void rotate(t_direction dir, int times)  //姿勢制御がないためvelocity-modeでなくpostion-mode
{
  int obj_step;
  controlInterruptStop();
  g_max_speed = SEARCH_SPEED;
  g_accel = TURN_ACCEL;
  g_speed = g_min_speed = MIN_SPEED;
  g_con_wall.enable = false;

  switch (dir) {
    case right:
      TMC5072Write(TMC5072_RAMPMODE1, 0x02);  //velocity mode(negative)
      TMC5072Write(TMC5072_RAMPMODE2, 0x01);  //velocity mode(positive)
      g_motor_move = 1;
      break;
    case left:
      TMC5072Write(TMC5072_RAMPMODE1, 0x01);  //velocity mode(positive)
      TMC5072Write(TMC5072_RAMPMODE2, 0x02);  //velocity mode(negative)
      g_motor_move = 1;
      break;
    default:
      g_motor_move = 0;
      break;
  }
  TMC5072Write(TMC5072_VMAX1, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_VMAX2, MIN_SPEED / TMC5072_VELOCITY);
  TMC5072Write(TMC5072_XACTUAL1, 0);  //初期化
  TMC5072Write(TMC5072_XACTUAL2, 0);  //初期化
  obj_step = (int)(TREAD_WIDTH * PI / 4.0 * (float)times * 2.0 / TMC5072_PULSE);
  controlInterruptStart();

  while (((obj_step - (abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
                       abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2)))) /
          2.0 * TMC5072_PULSE) >
         (((g_speed * g_speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * TURN_ACCEL))) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }

  g_accel = -1.0 * TURN_ACCEL;

  while ((abs((int)TMC5072Read_no_status(TMC5072_XACTUAL1)) +
          abs((int)TMC5072Read_no_status(TMC5072_XACTUAL2))) < obj_step) {
    TMC5072Write(TMC5072_VMAX1, spd_r / TMC5072_VELOCITY);
    TMC5072Write(TMC5072_VMAX2, spd_l / TMC5072_VELOCITY);
    delay(1);
  }
  TMC5072Write(TMC5072_VMAX1, 0);
  TMC5072Write(TMC5072_VMAX2, 0);

  g_motor_move = 0;
  delay(300);
}

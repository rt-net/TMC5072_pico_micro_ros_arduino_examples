
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

hw_timer_t * g_timer0 = NULL;
hw_timer_t * g_timer1 = NULL;
hw_timer_t * g_timer2 = NULL;  //for STEP/DIR
hw_timer_t * g_timer3 = NULL;  //for STEP/DIR

volatile unsigned short g_step_hz_r, g_step_hz_l;  //for STEP/DIR
volatile unsigned int g_step_r, g_step_l;          //for STEP/DIR

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void setRStepHz(short data) { g_step_hz_r = data; }  //for STEP/DIR

void setLStepHz(short data) { g_step_hz_l = data; }  //for STEP/DIR

void clearStepR(void) { g_step_r = 0; }  //for STEP/DIR

void clearStepL(void) { g_step_l = 0; }  //for STEP/DIR

unsigned int getStepR(void) { return g_step_r; }  //for STEP/DIR

unsigned int getStepL(void) { return g_step_l; }  //for STEP/DIR

void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR onTimer1(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  sensorInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR isrR(void)  //for STEP/DIR
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    if (g_step_hz_r < 30) g_step_hz_r = 30;
    timerAlarmWrite(g_timer2, 2000000 / g_step_hz_r, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    g_step_r++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR isrL(void)  //for STEP/DIR
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    if (g_step_hz_l < 30) g_step_hz_l = 30;
    timerAlarmWrite(g_timer3, 2000000 / g_step_hz_l, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    g_step_l++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void controlInterruptStart(void) { timerAlarmEnable(g_timer0); }
void controlInterruptStop(void) { timerAlarmDisable(g_timer0); }

void sensorInterruptStart(void) { timerAlarmEnable(g_timer1); }
void sensorInterruptStop(void) { timerAlarmDisable(g_timer1); }

void PWMInterruptStart(void)  //for STEP/DIR
{
  timerAlarmEnable(g_timer2);
  timerAlarmEnable(g_timer3);
}
void PWMInterruptStop(void)  //for STEP/DIR
{
  timerAlarmDisable(g_timer2);
  timerAlarmDisable(g_timer3);
}

void initAll(void)
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(BLED0, OUTPUT);
  pinMode(BLED1, OUTPUT);

  pinMode(SW_L, INPUT);
  pinMode(SW_C, INPUT);
  pinMode(SW_R, INPUT);

  ledcSetup(0, 440, 10);
  ledcAttachPin(BUZZER, 0);
  ledcWrite(0, 1024);

  pinMode(SLED_FR, OUTPUT);
  pinMode(SLED_FL, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_FR, LOW);
  digitalWrite(SLED_FL, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  pinMode(MOTOR_EN, OUTPUT);
  pinMode(CW_R, OUTPUT);   //for STEP/DIR
  pinMode(CW_L, OUTPUT);   //for STEP/DIR
  pinMode(PWM_R, OUTPUT);  //for STEP/DIR
  pinMode(PWM_L, OUTPUT);  //for STEP/DIR
  disableMotor();

  moveDir(MOT_FORWARD, MOT_FORWARD);  //for STEP/DIR
  digitalWrite(PWM_R, LOW);           //for STEP/DIR
  digitalWrite(PWM_L, LOW);           //for STEP/DIR

  if (!SPIFFS.begin(true)) {
    while (1) {
      Serial.println("SPIFFS Mount Failed");
      delay(100);
    }
  }

  g_timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer0, &onTimer0, false);
  timerAlarmWrite(g_timer0, 1000, true);
  timerAlarmEnable(g_timer0);

  g_timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(g_timer1, &onTimer1, false);
  timerAlarmWrite(g_timer1, 250, true);
  //  timerAlarmEnable(g_timer1);//TMC5072のSPI設定が終わるまで保留

  g_timer2 = timerBegin(2, 40, true);            //for STEP/DIR
  timerAttachInterrupt(g_timer2, &isrR, false);  //for STEP/DIR
  timerAlarmWrite(g_timer2, 13333, true);        //for STEP/DIR
  timerAlarmEnable(g_timer2);                    //for STEP/DIR

  g_timer3 = timerBegin(3, 40, true);            //for STEP/DIR
  timerAttachInterrupt(g_timer3, &isrL, false);  //for STEP/DIR
  timerAlarmWrite(g_timer3, 13333, true);        //for STEP/DIR
  timerAlarmEnable(g_timer3);                    //for STEP/DIR

  //  Serial.begin(115200);
  Serial.begin(921600);

  //モータがEnable時でないと設定が反映されない
  enableMotor();
  TMC5072Init();
  disableMotor();

  sensorInterruptStart();

  g_sen_r.ref = REF_SEN_R;
  g_sen_l.ref = REF_SEN_L;
  g_sen_r.th_wall = TH_SEN_R;
  g_sen_l.th_wall = TH_SEN_L;

  g_sen_fr.th_wall = TH_SEN_FR;
  g_sen_fl.th_wall = TH_SEN_FL;

  g_sen_r.th_control = CONTROL_TH_SEN_R;
  g_sen_l.th_control = CONTROL_TH_SEN_L;

  g_con_wall.kp = CON_WALL_KP;

  g_map_control.setGoalX(GOAL_X);
  g_map_control.setGoalY(GOAL_Y);

  g_motor_move = false;
  setRStepHz(MIN_SPEED);  //for STEP/DIR
  setLStepHz(MIN_SPEED);  //for STEP/DIR

  enableBuzzer(INC_FREQ);
  delay(80);
  disableBuzzer();
}

//LED
void setLED(unsigned char data)
{
  digitalWrite(LED0, data & 0x01);
  digitalWrite(LED1, (data >> 1) & 0x01);
  digitalWrite(LED2, (data >> 2) & 0x01);
  digitalWrite(LED3, (data >> 3) & 0x01);
}
void setBLED(char data)
{
  if (data & 0x01) {
    digitalWrite(BLED0, HIGH);
  } else {
    digitalWrite(BLED0, LOW);
  }
  if (data & 0x02) {
    digitalWrite(BLED1, HIGH);
  } else {
    digitalWrite(BLED1, LOW);
  }
}

//Buzzer
void enableBuzzer(short f) { ledcWriteTone(BUZZER_CH, f); }
void disableBuzzer(void)
{
  ledcWrite(BUZZER_CH, 1024);  //duty 100% Buzzer OFF
}

//motor
void enableMotor(void)
{
  digitalWrite(MOTOR_EN, LOW);  //Power ON TMC5072
}
void disableMotor(void) { digitalWrite(MOTOR_EN, HIGH); }

void moveDir(t_CW_CCW left_CW, t_CW_CCW right_CW)  //for STEP/DIR
{  //左右のモータの回転方向を指示する
  if (right_CW == MOT_FORWARD) {
    digitalWrite(CW_R, HIGH);
  } else {
    digitalWrite(CW_R, LOW);
  }

  if (left_CW == MOT_FORWARD) {
    digitalWrite(CW_L, HIGH);
  } else {
    digitalWrite(CW_L, LOW);
  }
}

//SWITCH
unsigned char getSW(void)
{
  unsigned char ret = 0;
  if (digitalRead(SW_R) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_R) == LOW);
    ret |= SW_RM;
  }
  if (digitalRead(SW_C) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_C) == LOW);
    ret |= SW_CM;
  }
  if (digitalRead(SW_L) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_L) == LOW);
    ret |= SW_LM;
  }
  return ret;
}

//sensor
unsigned short getSensorR(void)
{
  digitalWrite(SLED_R, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD3);
  digitalWrite(SLED_R, LOW);
  return tmp;
}
unsigned short getSensorL(void)
{
  digitalWrite(SLED_L, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD4);
  digitalWrite(SLED_L, LOW);
  return tmp;
}
unsigned short getSensorFL(void)
{
  digitalWrite(SLED_FL, HIGH);  //LED点灯
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD2);
  digitalWrite(SLED_FL, LOW);  //LED消灯
  return tmp;
}
unsigned short getSensorFR(void)
{
  digitalWrite(SLED_FR, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD1);
  digitalWrite(SLED_FR, LOW);
  return tmp;
}
short getBatteryVolt(void)
{
  short inputVoltage = (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0);
  return inputVoltage;
}

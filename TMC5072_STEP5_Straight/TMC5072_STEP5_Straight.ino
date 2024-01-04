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

#include "stdlib.h"
#include "SPI.h"
#include "TMC5072.h"

#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41

#define SW_L 10
#define SW_C 11
#define SW_R 12

#define MOTOR_EN 9
#define CW_R 21
#define CW_L 14
#define PWM_R 45
#define PWM_L 13

#define SPI_CS 3
#define SPI_CLK 46
#define SPI_MISO 47
#define SPI_MOSI 48

#define MIN_HZ 80
#define TIRE_DIAMETER (48.0)
#define TREAD_CIRCUIT (TREAD_WIDTH * PI / 4)
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#define MIN_SPEED (MIN_HZ * PULSE)

hw_timer_t * g_timer0 = NULL;
hw_timer_t * g_timer2 = NULL;
hw_timer_t * g_timer3 = NULL;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

unsigned short g_step_hz_r = MIN_HZ;
unsigned short g_step_hz_l = MIN_HZ;

volatile unsigned int g_step_r, g_step_l;
double g_max_speed;
double g_min_speed;
double g_accel = 0.0;
volatile double g_speed = MIN_SPEED;

volatile bool g_motor_move = 0;;

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);//割り込み禁止
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);//割り込み許可
}

//Rモータの周期数割り込み
void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  if (g_motor_move) {
    timerAlarmWrite(g_timer2, 2000000 / g_step_hz_r, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    g_step_r++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}

//Lモータの周期数割り込み
void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  if (g_motor_move) {
    timerAlarmWrite(g_timer3, 2000000 / g_step_hz_l, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    g_step_l++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}


void setup()
{
  // put your setup code here, to run once:
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW_L, INPUT);
  pinMode(SW_C, INPUT);
  pinMode(SW_R, INPUT);

  //motor disable
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(PWM_L, LOW);

  g_timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer0, &onTimer0, false);
  timerAlarmWrite(g_timer0, 1000, true);
  timerAlarmEnable(g_timer0);

  g_timer2 = timerBegin(2, 40, true);
  timerAttachInterrupt(g_timer2, &isrR, false);
  timerAlarmWrite(g_timer2, 13333, true);
  timerAlarmEnable(g_timer2);

  g_timer3 = timerBegin(3, 40, true);
  timerAttachInterrupt(g_timer3, &isrL, false);
  timerAlarmWrite(g_timer3, 13333, true);
  timerAlarmEnable(g_timer3);

  TMC5072Init();

  digitalWrite(LED0,HIGH);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3,HIGH);
  delay(500);
  digitalWrite(LED0,LOW);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);



}

void loop()
{
  // put your main code here, to run repeatedly:
  while (digitalRead(SW_L) & digitalRead(SW_C) & digitalRead(SW_R)) {
    continue;
  }
  if(digitalRead(SW_L)==LOW){
    digitalWrite(MOTOR_EN, LOW);
    TMC5072Setting(STEPDIR);    
    delay(1000);
    digitalWrite(LED0, HIGH);
    accelerate_STEPDIR(90, 350);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    oneStep_STEPDIR(180, 350);
    digitalWrite(LED3, HIGH);
    decelerate_STEPDIR(90, 350);
    digitalWrite(LED0, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(1000);
    digitalWrite(MOTOR_EN, HIGH);    
  }else if(digitalRead(SW_C)==LOW){
    digitalWrite(MOTOR_EN, LOW);
    TMC5072Setting(VELOCITY);
    delay(1000);
    digitalWrite(LED0, HIGH);
    accelerate_VELOCITY(90, 350);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    oneStep_VELOCITY(180, 350);
    digitalWrite(LED3, HIGH);
    decelerate_VELOCITY(90, 350);
    digitalWrite(LED0, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(1000);
    digitalWrite(MOTOR_EN, HIGH);  
  }else if(digitalRead(SW_R)==LOW){
    t_TMC5072_position tmc5072_position;
    digitalWrite(MOTOR_EN, LOW);    
    TMC5072Setting(SIXPOINT);
    delay(1000);

    tmc5072_position.init_speed = 10.0;//初速度[mm/s]
    tmc5072_position.v1_speed =150.0;//加速度a1のトップスピード[mm/s]
    tmc5072_position.a1_accel = 100;//v1までの加速度[mm/s^2]
    tmc5072_position.vmax_speed = 350.0;//トップスビード[mm/s]
    tmc5072_position.amax_aceel = 3000.0;//V1からvmaxまでの加速度[mm/s^2]
    tmc5072_position.finish_speed = 20.0;//停止速度[mm/s]
    tmc5072_position.len = 180.0*2.0;//距離[mm]

    TMC5072Straignt(tmc5072_position);

    delay(1000);
    digitalWrite(MOTOR_EN, HIGH);  
  }
}

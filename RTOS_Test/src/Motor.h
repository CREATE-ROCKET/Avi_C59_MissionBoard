/*
 Author : Yuta Takayasu
 Date : 2022/9/12

    Library for operating Maxon motor with ESP32.
*/


#pragma once
#ifndef Motor_H
#define Motor_H
#include <Arduino.h>
#include <driver/pcnt.h>
#include <soc/pcnt_struct.h>

static void IRAM_ATTR caller(void *arg);
int intr_count = 0;
pcnt_isr_handle_t user_isr_handle = NULL;
static void IRAM_ATTR caller(void *arg){
     if (PCNT.status_unit[PCNT_UNIT_0].h_lim_lat) {
        intr_count++;
    } else if (PCNT.status_unit[PCNT_UNIT_0].l_lim_lat) {
        intr_count--;
    }
    PCNT.int_clr.val = BIT(PCNT_UNIT_0);
}

class Motor
{
private:
    pcnt_config_t pcnt_config_A, pcnt_config_B;

public:
    int16_t cntRaw;
    double rad;
    double setSpd;

    void begin(int pwmA, int pwmB, int channelA, int channelB);
    void update();
    void setSpeed(double spd);// -1 ~ 1
};

void Motor::begin(int pwmA, int pwmB, int channelA, int channelB)
{
    cntRaw = 0;
    rad = 0;
    setSpd = 0;
    
    pcnt_config_A.pulse_gpio_num = channelA;//入力ピン(A)
    pcnt_config_A.ctrl_gpio_num = channelB;//制御ピン(B)
    pcnt_config_A.lctrl_mode = PCNT_MODE_KEEP;//制御ピンがlowのときの振る舞い
    pcnt_config_A.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_A.channel = PCNT_CHANNEL_0;
    pcnt_config_A.unit = PCNT_UNIT_0;
    pcnt_config_A.pos_mode = PCNT_COUNT_INC;//入力ピンが立ち上がり時の振る舞い
    pcnt_config_A.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_A.counter_h_lim = 32767;
    pcnt_config_A.counter_l_lim = -32768;

    pcnt_config_B.pulse_gpio_num = channelB;
    pcnt_config_B.ctrl_gpio_num = channelA;
    pcnt_config_B.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_B.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_B.channel = PCNT_CHANNEL_1;
    pcnt_config_B.unit = PCNT_UNIT_0;
    pcnt_config_B.pos_mode = PCNT_COUNT_INC;//入力ピンが立ち上がり時の振る舞い
    pcnt_config_B.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_B.counter_h_lim = 32767;
    pcnt_config_B.counter_l_lim = -32768;

    

    pcnt_unit_config(&pcnt_config_A);//ユニット初期化A相
    pcnt_unit_config(&pcnt_config_B);//ユニット初期化B相


    pcnt_counter_pause(PCNT_UNIT_0);//カウンタ一時停止

    pcnt_counter_clear(PCNT_UNIT_0);//カウンタ初期化

    // pcnt_set_filter_value(PCNT_UNIT_0,50);
    // pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_filter_disable(PCNT_UNIT_0);

    pcnt_counter_resume(PCNT_UNIT_0);//カウント開始


    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);
    pcnt_isr_register(caller, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT_0);

    ledcSetup(0, 50000, 10); //channel,PWMの周波数,num_bitの順
    ledcSetup(1, 50000, 10);
    ledcAttachPin(pwmA,0);
    ledcAttachPin(pwmB,1);
    return;
}

void IRAM_ATTR Motor::update()
{
    pcnt_get_counter_value(PCNT_UNIT_0, &cntRaw);
    rad = -(double)(intr_count * 16384 + (int)cntRaw) / 59392.0 * 2.0 * 3.1416f;
   
}
void IRAM_ATTR Motor::setSpeed(double spd)
{
    setSpd = spd;
    int spdInt = spd * 1024;

    if(spdInt < 0){
        spdInt = -spdInt;
        ledcWrite(0,spdInt);
        ledcWrite(1,0);
    }else{
        ledcWrite(0,0);
        ledcWrite(1,spdInt);
    }

    return;
}
#endif

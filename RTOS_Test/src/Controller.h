/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    Controller library for C59-j.
    
*/


#pragma once

#ifndef CON_H
#define CON_H
#include <Arduino.h>
#include <Motor.h>


class Controller
{
    private:
        int initPhase;
        int stopPhase;

        double Acof;// fin_angle_r = Mr / (Acof * airSpd * airSpd);
        
        double KpA;//Air Frame
        double KdA;

        double MAX_fin_angle;
        double KpM;
        double KdM;
        double F;


        double dpsiFilter, Gpsi;

        double M_PI_HALF;
    
    public:
        double MAX_V;
        double fin_dangle, old_fin_angle, Voltage, fin_angle_r, old_fin_angle_r, psi_r, fin_angle, Mr;
        
        void start(double psi);
        void update(int real_dt, double psi, double dpsi, double airSpd, Motor *maxon);
        void stop();

    Controller(){

        Gpsi = 0.6141;
        MAX_V = 10.;
        
        fin_dangle = 0;
        old_fin_angle = 0;
        
        Voltage = 0;

        old_fin_angle = 0;

        fin_angle_r = 0;
        old_fin_angle_r = 0; 

        psi_r = M_PI_HALF;

        initPhase = 1;
        stopPhase = 1;
        Acof = 0.00031999;// fin_angle_r = Mr / (Acof * airSpd * airSpd);
        
        KpA = 0.8;//Air Frame
        KdA = 0.07;

        MAX_fin_angle = 15./180.*M_PI;
        KpM = 240.0;
        KdM = 1.5;
        F = 50. * 2. * M_PI;

        M_PI_HALF = M_PI/2.;
    }
};

void Controller::start(double psi){
    if(psi > 0)
        psi_r = M_PI_HALF;
    else
        psi_r = -M_PI_HALF;

    initPhase = 1;
    stopPhase = 0;
    return;
}

void Controller::stop(){
    if(stopPhase == 0)initPhase = 1;
    stopPhase = 1;
    
    return;
}


void IRAM_ATTR Controller::update(int real_dt, double psi, double dpsi, double airSpd, Motor *maxon){
   
    double dt = (double)(real_dt) / 1000000.;
    
    old_fin_angle = fin_angle;
    fin_angle = maxon->rad;

    dpsiFilter = Gpsi * dpsiFilter + (1. - Gpsi) * dpsi;

    if(psi_r == M_PI_HALF){
        if(psi < -M_PI_HALF)psi += 2.* M_PI;
    }else if(psi_r == -M_PI_HALF){
        if(psi >  M_PI_HALF)psi -= 2.* M_PI;
    }

    //Controller
    
    Mr = KpA * (psi_r - psi) - (KdA * dpsiFilter);
    fin_angle_r = Mr / (Acof * airSpd * airSpd);
    if(stopPhase==1){
        fin_angle_r = 0;
    }

    //Limiter -13 degree ~ 13 degree
    if (fin_angle_r > MAX_fin_angle){
        fin_angle_r = MAX_fin_angle;
    }else if (fin_angle_r < -MAX_fin_angle){
        fin_angle_r = -MAX_fin_angle;
    }

    fin_dangle = (1. - dt * F) * fin_dangle + F * (fin_angle - old_fin_angle); //疑似微分で角速度 [rad/s]を計算

    
    if (initPhase == 1){
        Voltage = (fin_angle_r - fin_angle) * KpM;
        initPhase = 0;
    }else{
        Voltage = (fin_angle_r - fin_angle) * KpM +((fin_angle_r - old_fin_angle_r)/dt -  fin_dangle) * KdM; //指令電圧を計算 PD制御の場合
        // Voltage = (fin_angle_r-fin_angle)*KpM - fin_dangle * KdM;
    }

    //friction compensation
    if(Voltage > 0.05){
        if(Voltage < 0.65){
            Voltage = 0.65;
        }
    }else if(Voltage < -0.05){
        if(Voltage > -0.65){
            Voltage = -0.65;
        }
    }


    // Voltage limitation
    if(Voltage > MAX_V)Voltage = MAX_V;
    if(Voltage <  -MAX_V)Voltage = -MAX_V;

    old_fin_angle_r = fin_angle_r;

    maxon->setSpeed(Voltage / 10.0);
}
#endif
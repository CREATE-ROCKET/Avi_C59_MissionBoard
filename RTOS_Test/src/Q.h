/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    Q library.
    
*/


#pragma once

#ifndef Q_H
#define Q_H
#include <Arduino.h>

class Q
{
    private:
        double dt;
    
    public:
        double v[4];
        double dpsi;
        void init();
        void update(double *acc);
        void norm();
        void calc_Euler(double *euler);

    Q(){
        dt = 0.002;
        v[0] = 1.0;v[1] = 0;v[2] = 0;v[3] = 0;
    }
};
void IRAM_ATTR Q::init(){
    dt = 0.001;
    v[0] = 1.0;v[1] = 0;v[2] = 0;v[3] = 0;
}
void IRAM_ATTR Q::update(double *acc){


    double rotateArray[4][4] = { {0,        -acc[0],    -acc[1],    -acc[2] },
                                {acc[0],   0,          acc[2],     -acc[1] },
                                {acc[1],   -acc[2],    0,          acc[0]  },
                                {acc[2],   acc[1],     -acc[0],    0       }};


    double qnew[4];
    for(int i=0;i < 4;i++)qnew[i] = 0;

    for(int i = 0;i < 4;i++){
        qnew[i] = v[i];
        for(int k = 0;k < 4;k++){
            qnew[i] += (v[k]*rotateArray[i][k])/2.0f*dt;
        }
    }

    for(int i=0;i < 4;i++)v[i] = qnew[i];

    norm();

    return;
}
void IRAM_ATTR Q::norm(){
    double sca = v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3];
    sca = sqrt(sca);
    v[0] /= sca; v[1] /= sca; v[2] /= sca; v[3] /= sca;
    return;
}

void IRAM_ATTR Q::calc_Euler(double *euler){
	euler[0] = asin(2*(v[0]*v[1]-v[2]*v[3]));
	euler[1] = atan2(2*(v[1]*v[3]+v[0]*v[2]),v[0]*v[0]-v[1]*v[1]-v[2]*v[2]+v[3]*v[3]);
	euler[2] = atan2(2*(v[1]*v[2]+v[0]*v[3]),v[0]*v[0]-v[1]*v[1]+v[2]*v[2]-v[3]*v[3]);
}

#endif
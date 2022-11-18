/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    This program is StateMachine for C59J
    
*/


#pragma once


/*---------------- set time ----------------*/
#define BURNING_TIME 1250 // - 100 flame (launch detection)
#define GLIDING_TIME 3900// 3900
#define FALLING_TIME 10000
/*------------------------------------------*/

#define SD_USE (1)
#define MPU_USE (1)
#define PIT_USE (1)
#define FLASH_USE (1)

#ifndef MT_H
#define MT_H
#include <ProcessInterface.h>
#include <Arduino.h>
#include <SDIOLogWrapper.hpp>
#include <MPU9250.h>
#include <SPIflash.h>
#include <Q.h>
#include <PITOIC.h>
#include <Motor.h>
#include <Controller.h>
#include <iostream>



#define SDQUEUE 512 // 27 parameter 10 charas plus buffer

#define LEDPIN 5

#define FLASHCS 33
#define LPSCS 25
#define MPUCS 26
#define PITOCS 21

#define CLK 18
#define MISO 19
#define MOSI 23

#define F100KHZ 100000
#define F1MHZ 1000000
#define F10MHZ 10000000
#define F40MHZ 30000000
#define F50MHZ 50000000

void interpritor(char *input, int *output);
void interpritorf(char *input, double *output);

class  MainThread : public ProcessInterface
{
public:
    int state;
    double dt;
    double dpsi;
    double euler[3];
    
    uint32_t addr;
    int now,pre_now,real_dt;

    //input data
    double tempreture, pressure, launcherDegree, windDerection, windSpeed, batteryValutage;
    
    /*
    Debug  : for development
    Phase 0: Sleep
        Data        :False
        SPI Log     :False
        SD Log      :False
        
        Q update    :False
        Actuator    :Off

        process     :waiting for command

    Phase 1: Waiting for Launch
        Data        :True
        SPI Log     :False
        SD Log      :True
        
        Q update    :False
        Actuator    : Power on (not control)

        process     : launch detection

    Pahse 2: Firing rocket engine
        Data        :True
        SPI Log     :True
        SD Log      :True
        
        Q update    :True
        Actuator    : Power on (not control)

        process     : pose estimate

    Phase 3: Gliding
        Data        :True
        SPI Log     :True
        SD Log      :True
        
        Q update    :True
        Actuator    :Control

        process     : pose estimate and control

    Phase 4: Falling with a parachute open
        Data        :True
        SPI Log     :True
        SD Log      :True
        
        Q update    :True
        Actuator    :Power on (not control)

        process     : pose estimate and control

    Phase 5: Fall on Ground
        Data        :False
        SPI Log     :False
        SD Log      :False
        
        Q update    :False
        Actuator    :Off

        process     : none
    */
    int16_t mpuDataInt[6];
    int16_t mpuMagDataInt[6];
    int pitotDataInt;
    int16_t mpuGyroCalib[3];

    char SDbuf[SDQUEUE];
    uint8_t Flashbuf[256];

    SPICreate spi;
    MPU mpu;Flash flash;
    PITOIC pitoic;
    
    Motor maxon;
    Controller ctrl;



    void IRAM_ATTR processDebug();
    void IRAM_ATTR processPhase0();
    void IRAM_ATTR processPhase1();
    void IRAM_ATTR processPhase2();
    void IRAM_ATTR processPhase3();
    void IRAM_ATTR processPhase4();

    
    void checkDevice();
    void IRAM_ATTR logSPIflash();
    void IRAM_ATTR logSD();
    void IRAM_ATTR logSDbin();
    int IRAM_ATTR launchDetection();//0 not launch : 1 launch
    void IRAM_ATTR GetData();
    void CalibGyro();
    uint8_t calc_Euler_AngularVelocity();
    int check30deg();
    void flash2SD();
    void SDbinConv();

    int debugData;

    int inTask;
    double mpuDataAcc[3];
    double mpuDataGyro[3];
    double mpuDataMag[3];
    double pitotData;
    double estPose[4];
    Q q;

    void begin();
    void tickProcess();
};

void MainThread::begin(){
    pinMode(PITOCS,OUTPUT);
    pinMode(MPUCS,OUTPUT);
    pinMode(FLASHCS,OUTPUT);
    digitalWrite(PITOCS, HIGH);
    digitalWrite(MPUCS, HIGH);
    digitalWrite(FLASHCS, HIGH);
    delay(3000);

    // Serial.begin(115200);
    Serial.begin(115200,134217756U,16,17);
    // Serial2.begin(115200);
    pinMode(LEDPIN,OUTPUT);
    digitalWrite(LEDPIN, HIGH);

    addr = 0;

    state = 0;
    dt = 0.002;
    real_dt = 0;

    tempreture = 17.;
    pressure = 1013.;
    launcherDegree = 80.;
    windDerection = 0.;
    windSpeed = 0.;
    batteryValutage = 10.;

    now = micros();
    pre_now = micros();

    //SPI setup
    spi = SPICreate();
    spi.begin(VSPI,CLK,MISO,MOSI,F40MHZ);

    #if FLASH_USE
    flash = Flash();
    flash.begin(&spi,FLASHCS,F10MHZ);
    #endif
    
    
    #if PIT_USE
    pitoic.begin(&spi, PITOCS, 400000);
    #endif

    delay(100);

    #if MPU_USE
    mpu = MPU();
    mpu.begin(&spi, MPUCS, F10MHZ);
    #endif

    CalibGyro();

    maxon.begin(32,27,22,34);


}

void IRAM_ATTR MainThread::processDebug(){// debug process
       static int logIte = 0;

    static double motor_angle = 0.; 
    static double dt = 0.001;
    static int pwm_ratio = 0;
    static double MAX_V = 9.;
    static char uart_TX_buf[14];

    static double t = 0.;
    static double f = 3.;
    static double u = 0.; 

    static int finish = 0;

    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        Serial.println("Cancel logging and switch Phase0");
        maxon.setSpeed(0);
        maxon.update();
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }



    if(logIte > 1000){
        maxon.update();
        motor_angle = maxon.rad;
        t += (double)real_dt / 1000000.0;
        u = sin(2.*M_PI*f*t);

        if(t > 1./f){
            t = 0.;
            if(f < 1.) f+=0.1; 
            else if(f < 10.) f+=1.;
            else if(f < 100.) f+=10.;
        }
        if(f >= 100){ //100Hz�܂�
            if(finish==0){
                maxon.setSpeed(0);
                Serial.println("finish");
                finish = 1;
            }
            return;
        }
        // Serial.println(f);
        maxon.setSpeed(u);
    }
    

    logSD();
    logIte++;

}

void IRAM_ATTR MainThread::processPhase0(){// console
    static int logIte = 0;
    
    if(Serial.available()){
        
        char command = Serial.read();
        if(command == 'g'){
            Serial.println("Begin Gyro Calibration");
            CalibGyro();
            Serial.println("Finish calibration with below value.[X/Y/Z]");


            Serial.print(mpuGyroCalib[0]);Serial.print(",\t");
            Serial.print(mpuGyroCalib[1]);Serial.print(",\t");
            Serial.print(mpuGyroCalib[2]);Serial.print("\n");
        }
        else if(command == 'q'){
            Serial.println("Set Q to initial value");
            q.init();
        }
        else if(command == 'f'){
            Serial.println("Set Fin 0 position. \nMotor Surplly will be cut off within 20 sec.");
            maxon.begin(32,27,22,34);
            maxon.setSpeed(0);
            delay(17000);
            Serial.println("Leave the jig from the wing !!");
            delay(3000);
            maxon.begin(32,27,22,34);
            maxon.setSpeed(0);
            Serial.println("Power supply resumed");
        }

        else if(command == 'd'){
            Serial.println("Cut off Power supply");
            
            while(true){
                maxon.setSpeed(0);
                delay(1000);
                Serial.println("Press d");
                if(Serial.available() && Serial.read() == 'd'){
                    break;
                }
            }

            Serial.println("Power supply resumed");
        }

        else if(command == 'D'){//debug
            Serial.println("state -1");
            
#if SD_USE
            SDIOLogWrapper::makeQueue(SDQUEUE*4);
            SDIOLogWrapper::setSaveFileName("/data.csv");
            SDIOLogWrapper::setSaveInterval(100);
            Serial.println(SDIOLogWrapper::initSD());
            SDIOLogWrapper::openFile();
            SDIOLogWrapper::writeTaskCreate(PRO_CPU_NUM);
            
#endif
            state = -1;
            
            maxon.begin(32,27,22,34);
            maxon.setSpeed(0);
            delay(1000);
            return;
        }
        else if(command == 't'){
            Serial.println("Transfer Flash Data to SD card.");
            flash2SD();
            Serial.println("finish");
        }
        else if(command == 'b'){
            
            while(true){
                Serial.println("Do you continue? y/n");
                if(Serial.available())break;
                delay(1000);
            }

            char c = Serial.read();
            if(c == 'y'){
                Serial.println("Begin Bulk Erase");
                flash.erase();
            }else{
                Serial.println("Erase cancered.");
            }
            
            
        }
        else if(command == 'i'){
            Serial.println("Input Parameter.");
            Serial.println("[temp], [pressure(hPa)], [lauchDegree], [batteryValutage]");
            double readData[10];
            char buf[1000];
            int bufite = 0;
            while(true){
                if(Serial.available()){
                    char c = Serial.read();
                    buf[bufite] = c;
                    Serial.print(c);
                    bufite++;
                    if(c=='\n')break;
                }
                delay(1);
            }
            interpritorf(buf,readData);
            tempreture      = readData[0];
            pressure        = readData[1];
            launcherDegree  = readData[2];
            batteryValutage = readData[3];


            Serial.print("temp                  \t\t: ");Serial.println(readData[0]);
            Serial.print("pressure(hPa)         \t\t: ");Serial.println(readData[1]);
            Serial.print("launcherDegree(degree)\t\t: ");Serial.println(readData[2]);
            Serial.print("batteryValutage(degree)\t\t: ");Serial.println(readData[3]);


            ctrl.MAX_V = batteryValutage;
            q.v[0] = cos((90. - launcherDegree)/180./2.*M_PI);
            q.v[1] = 0.;
            q.v[2] = sin((90. - launcherDegree)/180./2.*M_PI);
            q.v[3] = 0;
            
            euler[1] = (90. - launcherDegree)/180./2.*M_PI;
        }
        else if(command == 's'){
            
            Serial.println("Switch to Log mode");
            Serial.println("Set state 1");
            delay(100);
            Serial.println("Set state 1");
            delay(100);
            Serial.println("Set state 1");
            delay(100);
            Serial.println("Set state 1");
            delay(100);
            Serial.println("Set state 1");

#if SD_USE
            SDIOLogWrapper::makeQueue(SDQUEUE*4);
            SDIOLogWrapper::setSaveFileName("/data.csv");
            SDIOLogWrapper::setSaveInterval(100);
            Serial.println(SDIOLogWrapper::initSD());
            SDIOLogWrapper::openFile();
            SDIOLogWrapper::writeTaskCreate(PRO_CPU_NUM);
            
#endif
            state = 1;
            logIte = 0;
            return;
        }
        else if(command == '\n'){
        }
        else{
            Serial.print("[E] Invalid command detect!\n[E] ");
            Serial.println(command);
        }
    }

    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, pitoic.WindSpeedFiliter, &maxon);

    if(logIte % 500 == 0){
        Serial.println("Phase 0");
        // Serial2.println("Phase 0");
    }

    if(logIte % 1000 == 0)
        digitalWrite(LEDPIN,HIGH);
    else if(logIte % 1000 == 10)
        digitalWrite(LEDPIN,LOW);

    logIte++;
}
void IRAM_ATTR MainThread::processPhase1(){// wait launch
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        addr = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::deleteQueue();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }
    // Serial.print(euler[2]);Serial.print(",");Serial.println(now);

    GetData();

    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, pitoic.WindSpeedFiliter, &maxon);
    // logSPIflash();
    // logSDbin();
    logSD();


    if(launchDetection()){
        logIte = 0;
        state = 2;
        Serial.println("Set state 2");
        return;
    }

    //LED
    if(logIte % 300 == 0)
        digitalWrite(LEDPIN,HIGH);
    else if(logIte % 300 == 150)
        digitalWrite(LEDPIN,LOW);

    logIte++;
}
void IRAM_ATTR MainThread::processPhase2(){// burning
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        addr = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::deleteQueue();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }
    
    Serial.print(euler[2],4);Serial.print(",");
    Serial.println(micros());
    GetData(); // around 50 ~ 100us but mag read 400us

    q.update(mpuDataGyro,real_dt);// < 100us
    q.calc_Euler(euler);
    calc_Euler_AngularVelocity();
    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, pitoic.WindSpeedFiliter, &maxon);

    
    logSPIflash();// < 800 us
    // logSD(); // 300 ~ 800 us
    
    

    if(logIte > BURNING_TIME){
        logIte = 0;
        state = 3;
        ctrl.start(euler[2]);
        Serial.println("Set state 3");
        return;
    }

    //LED
    if(logIte % 100 == 0)
        digitalWrite(LEDPIN,HIGH);
    else if(logIte % 100 == 50)
        digitalWrite(LEDPIN,LOW);
   
    logIte++;
}
void IRAM_ATTR MainThread::processPhase3(){// control
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        addr = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::deleteQueue();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }

    Serial.print(euler[2],4);Serial.print(",");
    Serial.println(micros());

    
    GetData();//200us ~ 400us

    //300 ~ 600 us
    q.update(mpuDataGyro,real_dt);  
    q.calc_Euler(euler);
    calc_Euler_AngularVelocity();
    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, pitoic.WindSpeedFiliter, &maxon);

    if(check30deg()==-1)ctrl.stop();
    
    
    logSPIflash();//10 ~ 200 us
    // logSD();
    


    if(logIte > GLIDING_TIME){
        logIte = 0;
        state = 4;
        ctrl.stop();
        Serial.println("Set state 4");
        return;
    }

    //LED
    digitalWrite(LEDPIN,HIGH);
    logIte++;
}
void IRAM_ATTR MainThread::processPhase4(){// falling
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        addr = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::deleteQueue();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }

    GetData();
    
    q.update(mpuDataGyro,real_dt);// < 100us
    q.calc_Euler(euler);
    calc_Euler_AngularVelocity();
    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, pitoic.WindSpeedFiliter, &maxon);


    logSPIflash();
    // logSD();


    if(logIte > FALLING_TIME){
        logIte = 0;
        state = 0;
        
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::deleteQueue();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        Serial.println("Finish");
        while(true){
            Serial.println("Finish");
            delay(1000);
        }
        return;
    }

    //LED
    if(logIte % 100 == 0)
        digitalWrite(LEDPIN,HIGH);
    else if(logIte % 100 == 50)
        digitalWrite(LEDPIN,LOW);
    logIte++;
}

void IRAM_ATTR MainThread::tickProcess(){

    pre_now = now;
    now = ESP.getCycleCount() / 240;
    maxon.update();
    real_dt = now - pre_now;
    if(real_dt < 0)real_dt = 2000;//overflow


    inTask = 1;
    switch (state)
    {
    case -1:
        processDebug();
        break;
    case 0:
        processPhase0();
        break;
    case 1:
        processPhase1();
        break;
    case 2:
        processPhase2();
        break;
    case 3:
        processPhase3();
        break;
    case 4:
        processPhase4();
        break;
    default:
        Serial.println("invalid state!!");
        break;
    }

    inTask = 0;
    return;
    
}

void IRAM_ATTR MainThread::GetData(){
    static int lastFetchMag = 0;// Last time to fetch mag data from mpu
    

    #if MPU_USE
    mpu.Get(mpuDataInt);

    //Scale Adjusting Acc 16G Gyro 2500dps to 1G(double) 1 rad(double)
    mpuDataAcc[0] = mpuDataInt[0] * 0.00048828125f;
    mpuDataAcc[1] = mpuDataInt[1] * 0.00048828125f;
    mpuDataAcc[2] = mpuDataInt[2] * 0.00048828125f;

    mpuDataGyro[0] = (mpuDataInt[3] - mpuGyroCalib[0]) * 0.06103515625f * 3.141592f / 180.0f;
    mpuDataGyro[1] = (mpuDataInt[4] - mpuGyroCalib[1]) * 0.06103515625f * 3.141592f / 180.0f;
    mpuDataGyro[2] = (mpuDataInt[5] - mpuGyroCalib[2]) * 0.06103515625f * 3.141592f / 180.0f;



    // to uT
    if(lastFetchMag%5 == 0 && mpu.GetMag(mpuMagDataInt) == 0){
        mpuDataMag[0] = mpuMagDataInt[0]*0.6f;
        mpuDataMag[1] = mpuMagDataInt[1]*0.6f;
        mpuDataMag[2] = mpuMagDataInt[2]*0.6f;
    }
    lastFetchMag++;
    #endif

    #if PIT_USE
    uint8_t pitorx[2];
    pitoic.Get(pitorx);
    pitotDataInt = pitorx[1] | pitorx[0] << 8;
    #endif

}
int IRAM_ATTR MainThread::launchDetection(){
    static int Ite = 0;
    static double aveAcc = 0;
    static int overTime = 0;

    double AccScal = mpuDataAcc[0]*mpuDataAcc[0] + mpuDataAcc[1]*mpuDataAcc[1] + mpuDataAcc[2]*mpuDataAcc[2];
    aveAcc += sqrt(AccScal)/20.0f;
    Ite++;

    if(Ite==20){
        Ite = 0;
        
        if(aveAcc > 1.2){
            Ite = 0;
            aveAcc = 0;
            overTime++;
        }
        else{
            overTime = 0;
            aveAcc = 0;
        }     
    }

    if(overTime==5){
        overTime = 0;
        return 1;
    }
    return 0;
}
void IRAM_ATTR MainThread::logSPIflash(){
#if FLASH_USE
    static int logCnt = 0;

    int slice = 85*logCnt;
    Flashbuf[slice + 0] = state;

    //pito data
    Flashbuf[slice + 1] = pitotDataInt >> 8;
    Flashbuf[slice + 2] = pitotDataInt;

    //mpu data 3 ~ 14
    for(int i = 0;i < 6;i++){
        Flashbuf[slice + 3 + 2*i] = mpuDataInt[i] >> 8;
        Flashbuf[slice + 4 + 2*i] = mpuDataInt[i];
    }

    //Q data 15 ~ 30
    for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			Flashbuf[slice + 15 + 4*i + j] = ( (int32_t)(q.v[i]*1800000000.) >> (8*(3-j)) )&0xFF;
		}
	}
    

    //Euler 31 ~ 42
    for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			Flashbuf[slice + 31 + 4*i + j] = ( (int)(euler[i]*600000000.) >> (8*(3-j)) )&0xFF;
		}
	}

    // controller config 43 ~ 42
    int buf;


    buf = dpsi * 30000000.;//43 ~ 46
    for(int i = 0;i < 4;i++){
        Flashbuf[slice + 43 + i] = ( buf >> (8*(3-i)) )&0xFF;
    }

    buf = maxon.setSpd * 10000.;//47 ~ 48
    Flashbuf[slice + 47] = buf >> 8;
    Flashbuf[slice + 48] = buf;


    buf = maxon.rad * 6000000000.;//49 ~ 52
    for(int i = 0;i < 4;i++){
        Flashbuf[slice + 49 + i] = ( buf >> (8*(3-i)) )&0xFF;
    }

    buf = ctrl.Mr * 100000000.;//53 ~ 56
    for(int i = 0;i < 4;i++){
        Flashbuf[slice + 53 + i] = ( buf >> (8*(3-i)) )&0xFF;
    }

    buf = ctrl.fin_angle_r * 6000000000.;//57 ~ 60
    for(int i = 0;i < 4;i++){
        Flashbuf[slice + 57 + i] = ( buf >> (8*(3-i)) )&0xFF;
    }

    buf = ctrl.fin_dangle * 30000000.;//61 ~ 64
    for(int i = 0;i < 4;i++){
        Flashbuf[slice + 61 + i] = ( buf >> (8*(3-i)) )&0xFF;
    }


    //mag data 65 ~ 70
    for(int i = 0;i < 3;i++){
        Flashbuf[slice + 65 + 2*i] = mpuMagDataInt[i] >> 8;
        Flashbuf[slice + 66 + 2*i] = mpuMagDataInt[i];
    }


    //time stamp
    Flashbuf[slice + 79] = real_dt >> 8;
    Flashbuf[slice + 80] = real_dt;

    Flashbuf[slice + 81] = now >> 24;
    Flashbuf[slice + 82] = now >> 16;
    Flashbuf[slice + 83] = now >> 8;
    Flashbuf[slice + 84] = now;

    if(logCnt == 2){
        flash.write(addr,Flashbuf);
        logCnt=-1;
        addr += 0x100;
        for(int i = 0;i<256;i++)Flashbuf[i] = 0;
    }
    logCnt++;
#endif
}

void IRAM_ATTR MainThread::logSD(){
#if SD_USE
    //500 ~ 600 us

    char *bfChar = new char[SDQUEUE];
    int buf;
    buf = micros();
    bfChar[0] = '\0';

    snprintf(bfChar,SDQUEUE,
            "%d,\
             %d,\
             %8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,\   
             %8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,\
             %d,\
             %d\
             \n",
            state,
            pitotDataInt,
            pitoic.WindSpeedFiliter,
            mpuDataAcc[0],  mpuDataAcc[1],  mpuDataAcc[2],
            mpuDataGyro[0], mpuDataGyro[1], mpuDataGyro[2],
            mpuDataMag[0],  mpuDataMag[1],  mpuDataMag[2],
            euler[0],       euler[1],       euler[2],       
            q.v[0],         q.v[1],         q.v[2],         q.v[3],
            dpsi,           maxon.setSpd,   maxon.rad,      ctrl.Mr,         ctrl.fin_angle_r,
            ctrl.fin_dangle,
            real_dt,
            now
            );
    
    debugData = micros() - buf;
    // Serial.print(bfChar);
    if(SDIOLogWrapper::appendQueue(bfChar) == 1){
        Serial.print("SD Overflow queue");
    }
    
#endif
}

union IntAndFloat {
    int ival;
    float fval;
};
void IRAM_ATTR MainThread::logSDbin(){
#if SD_USE
   
    char *bfChar = new char[104];
    
    
    union IntAndFloat target;
    
    for(int i = 0;i < 4;i++)bfChar[0 + i] = state >> (3-i)*8;

    for(int i = 0;i < 4;i++)bfChar[4 + i] = pitotDataInt >> (3-i)*8;


    target.fval = pitoic.WindSpeedFiliter;
    for(int i = 0;i < 4;i++)bfChar[8 + i] = target.ival >> (3-i)*8;

    target.fval = mpuDataAcc[0];
    for(int i = 0;i < 4;i++)bfChar[12 + i] = target.ival >> (3-i)*8;
    target.fval = mpuDataAcc[1];
    for(int i = 0;i < 4;i++)bfChar[16 + i] = target.ival >> (3-i)*8;
    target.fval = mpuDataAcc[2];
    for(int i = 0;i < 4;i++)bfChar[20 + i] = target.ival >> (3-i)*8;

    target.fval = mpuDataGyro[0];
    for(int i = 0;i < 4;i++)bfChar[24 + i] = target.ival >> (3-i)*8;
    target.fval = mpuDataGyro[1];
    for(int i = 0;i < 4;i++)bfChar[28 + i] = target.ival >> (3-i)*8;
    target.fval = mpuDataGyro[2];
    for(int i = 0;i < 4;i++)bfChar[32 + i] = target.ival >> (3-i)*8;

    target.fval = mpuDataMag[0];
    for(int i = 0;i < 4;i++)bfChar[36 + i] = target.ival >> (3-i)*8;
    target.fval = mpuDataMag[1];
    for(int i = 0;i < 4;i++)bfChar[40 + i] = target.ival >> (3-i)*8;
    target.fval = mpuDataMag[2];
    for(int i = 0;i < 4;i++)bfChar[44 + i] = target.ival >> (3-i)*8;

    target.fval = euler[0];
    for(int i = 0;i < 4;i++)bfChar[48 + i] = target.ival >> (3-i)*8;
    target.fval = euler[1];
    for(int i = 0;i < 4;i++)bfChar[52 + i] = target.ival >> (3-i)*8;
    target.fval = euler[2];
    for(int i = 0;i < 4;i++)bfChar[56 + i] = target.ival >> (3-i)*8;

    target.fval = q.v[0];
    for(int i = 0;i < 4;i++)bfChar[60 + i] = target.ival >> (3-i)*8;
    target.fval = q.v[1];
    for(int i = 0;i < 4;i++)bfChar[64 + i] = target.ival >> (3-i)*8;
    target.fval = q.v[2];
    for(int i = 0;i < 4;i++)bfChar[68 + i] = target.ival >> (3-i)*8;
    target.fval = q.v[3];
    for(int i = 0;i < 4;i++)bfChar[72 + i] = target.ival >> (3-i)*8;

    target.fval = dpsi;
    for(int i = 0;i < 4;i++)bfChar[76 + i] = target.ival >> (3-i)*8;

    target.fval = maxon.setSpd;
    for(int i = 0;i < 4;i++)bfChar[84 + i] = target.ival >> (3-i)*8;

    target.fval = maxon.rad;
    for(int i = 0;i < 4;i++)bfChar[88 + i] = target.ival >> (3-i)*8;

    target.fval = ctrl.Mr;
    for(int i = 0;i < 4;i++)bfChar[92 + i] = target.ival >> (3-i)*8;

    target.fval = ctrl.fin_angle_r;
    for(int i = 0;i < 4;i++)bfChar[96 + i] = target.ival >> (3-i)*8;

    target.fval = ctrl.fin_dangle;
    for(int i = 0;i < 4;i++)bfChar[100 + i] = target.ival >> (3-i)*8;

    for(int i = 0;i < 4;i++)bfChar[100 + i] = real_dt >> (3-i)*8;
    
    for(int i = 0;i < 4;i++)bfChar[100 + i] = now;
    
    if(SDIOLogWrapper::appendQueue(bfChar) == 1){
        Serial.print("SD Overflow queue");
    }
    
#endif
}

void MainThread::CalibGyro(){
    mpuGyroCalib[0] = 0;mpuGyroCalib[1] = 0;mpuGyroCalib[2] = 0;
    int16_t calib[3];
    calib[0] = 0;calib[1] = 0;calib[2] = 0;
    for(int i = 0;i < 1000;i++){
        GetData();
        calib[0] += mpuDataInt[3];
        calib[1] += mpuDataInt[4];
        calib[2] += mpuDataInt[5];
        delay(1);
    }
    mpuGyroCalib[0] = 0.001 * calib[0];
    mpuGyroCalib[1] = 0.001 * calib[1];
    mpuGyroCalib[2] = 0.001 * calib[2];
    return;
}
uint8_t MainThread::calc_Euler_AngularVelocity(){
	uint8_t Singularity_check = 0;
    
	dpsi = sin(euler[2]) * tan(euler[0]) * mpuDataGyro[0] + cos(euler[2]) * tan(euler[0]) * mpuDataGyro[1] + mpuDataGyro[2];

	if ((euler[0] > M_PI/2.-0.1) || (euler[0] < -M_PI/2.+0.1)){ //特異点近傍
		Singularity_check = 1;
	}else{
		Singularity_check = 0;
	}
	return Singularity_check;
}

int IRAM_ATTR MainThread::check30deg(){

    // // Top Secret
    // double vec[3] = {0., 0., 1.};
    // double launcherVec[3] = {sin((90. - launcherDegree)/180. * M_PI), 0., cos((90. - launcherDegree)/180. * M_PI)};
    // double res[3];
    // q.vecRotate(vec, res);

    // double dot = res[0] * launcherVec[0] + res[1] * launcherVec[1] + res[2] * launcherVec[2];
    // // Serial.println(acos(dot) / 3.14 * 180.);
    // if(acos(dot) / 3.14 * 180. > 50.)return -1;
    return 0;
}

void MainThread::flash2SD(){
    // uint8_t buf[256];
    // flash.read(0, buf);
    // for(int i = 0;i < 256;i++)Serial.println(buf[i]);

    Serial.println("transfer begin");

    SDIOLogWrapper::makeQueue(SDQUEUE*4);
    SDIOLogWrapper::setSaveFileName("/flash.csv");
    SDIOLogWrapper::setSaveInterval(100);

    Serial.println(SDIOLogWrapper::initSD());
    SDIOLogWrapper::openFile();
    SDIOLogWrapper::writeTaskCreate(PRO_CPU_NUM);

    uint8_t dataFlash[256];
    MainThread bufthread = MainThread();


    for(int i = 0;i < 100000; i++){
        flash.read(0x100 * i, dataFlash);
        if((dataFlash[1] == 0 && dataFlash[3] == 0 && dataFlash[5] == 0)||(dataFlash[1] == 255 && dataFlash[3] == 255 && dataFlash[5] == 255)){
            SDIOLogWrapper::writeTaskDelete();
            SDIOLogWrapper::deleteQueue();
            SDIOLogWrapper::closeFile();
            SDIOLogWrapper::deinitSD();
            return;
        }

        for(int k = 0;k < 3;k++){
            int slice = k * 85;

            int16_t buf;
            bufthread.state = dataFlash[slice + 0];

            bufthread.pitotDataInt = dataFlash[slice + 1] << 8 |  dataFlash[slice + 2];

            //mpu data 3 ~ 14
            for(int i_mpu = 0;i_mpu < 6;i_mpu++){
                bufthread.mpuDataInt[i_mpu] = dataFlash[slice + 3 + 2*i_mpu] << 8 | dataFlash[slice + 4 + 2*i_mpu];
            }

            //Q data 15 ~ 30
            for(int i_q = 0; i_q < 4; i_q++){
                bufthread.q.v[i_q] = 0;
                for(int j_q = 0; j_q < 4; j_q++){
                    bufthread.q.v[i_q] += dataFlash[slice + 15 + 4*i_q + j_q] << (8*(3-j_q));
                }
                bufthread.q.v[i_q] /= 1800000000.;
            }

            //Euler 31 ~ 42
            for(int i_e = 0; i_e < 3; i_e++){
                bufthread.euler[i_e] = 0;
                for(int j_e = 0; j_e < 4; j_e++){
                    bufthread.euler[i_e] += dataFlash[slice + 31 + 4*i_e + j_e] << (8*(3-j_e));
                }
                bufthread.euler[i_e] /= 600000000.;
            }

            // controller config 43 ~ 42
            bufthread.dpsi = 0;//43 ~ 46
            for(int j_b = 0; j_b < 4; j_b++){
                bufthread.dpsi += dataFlash[slice + 43 + j_b] << (8*(3-j_b));
            }
            bufthread.dpsi /= 30000000.;
            
            //47 ~ 48
            int16_t spdbuf = (dataFlash[slice + 47] << 8 | dataFlash[slice + 48]);
            bufthread.maxon.setSpd  = (double)spdbuf/10000.;
            
            bufthread.maxon.rad = 0;//49 ~ 52
            for(int j_b = 0; j_b < 4; j_b++){
                bufthread.maxon.rad += dataFlash[slice + 49 + j_b] << (8*(3-j_b));
            }
            bufthread.maxon.rad /= 6000000000.;

            bufthread.ctrl.Mr = 0;//53 ~ 56
            for(int j_b = 0; j_b < 4; j_b++){
                bufthread.ctrl.Mr += dataFlash[slice + 53 + j_b] << (8*(3-j_b));
            }
            bufthread.ctrl.Mr /= 100000000.;

            bufthread.ctrl.fin_angle_r = 0;//57 ~ 60
            for(int j_b = 0; j_b < 4; j_b++){
                bufthread.ctrl.fin_angle_r += dataFlash[slice + 57 + j_b] << (8*(3-j_b));
            }
            bufthread.ctrl.fin_angle_r /= 6000000000.;

            bufthread.ctrl.fin_dangle = 0;//61 ~ 64
            for(int j_b = 0; j_b < 4; j_b++){
                bufthread.ctrl.fin_dangle += dataFlash[slice + 61 + j_b] << (8*(3-j_b));
            }
            bufthread.ctrl.fin_dangle /= 30000000.;


            
            bufthread.real_dt   = dataFlash[slice + 79] << 8 | dataFlash[slice + 80];
            bufthread.now = dataFlash[slice + 81] << 24 | dataFlash[slice + 82] << 16 | dataFlash[slice + 83] << 8 | dataFlash[slice + 84];
        
            //mag data 65 ~ 70
            for(int i_mag = 0;i_mag < 3;i_mag++){
                bufthread.mpuMagDataInt[i_mag] = dataFlash[slice + 65 + 2*i_mag] << 8 | dataFlash[slice + 66 + 2*i_mag];
            }
            //Log SD
            char *bfChar = new char[SDQUEUE];
        
            bfChar[0] = '\0';
            snprintf(bfChar,SDQUEUE,
                   "%d,\
                    %d,\
                    %d,%d,%d,\
                    %d,%d,%d,\
                    %d,%d,%d,\
                    %8.5lf,%8.5lf,%8.5lf,\
                    %8.5lf,%8.5lf,%8.5lf,%8.5lf,\
                    %8.5lf,%8.5lf,%8.5lf,\
                    %8.5lf,%8.5lf,%8.5lf,\
                    %d,%d\
                    \n",
                    bufthread.state,
                    bufthread.pitotDataInt,
                    bufthread.mpuDataInt[0],    bufthread.mpuDataInt[1],    bufthread.mpuDataInt[2],
                    bufthread.mpuDataInt[3],    bufthread.mpuDataInt[4],    bufthread.mpuDataInt[5],
                    bufthread.mpuMagDataInt[0], bufthread.mpuMagDataInt[1], bufthread.mpuMagDataInt[2],
                    bufthread.euler[0],         bufthread.euler[1],         bufthread.euler[2],       
                    bufthread.q.v[0],           bufthread.q.v[1],           bufthread.q.v[2],           bufthread.q.v[3],
                    bufthread.dpsi,             bufthread.maxon.setSpd,     bufthread.maxon.rad,        
                    bufthread.ctrl.Mr,          bufthread.ctrl.fin_angle_r, bufthread.ctrl.fin_dangle,
                    bufthread.real_dt,          bufthread.now
                    );
            
            // Serial.print(bfChar);
            if(SDIOLogWrapper::appendQueue(bfChar) == 1){
                Serial.print("SD Overflow queue");
            }
            delay(1);
        }

        if(i%100 == 0)Serial.println(i);
    }

    SDIOLogWrapper::writeTaskDelete();
    SDIOLogWrapper::deleteQueue();
    SDIOLogWrapper::closeFile();
    SDIOLogWrapper::deinitSD();

    Serial.println("transfer end");
}

void interpritor(char *input, int *output){

    int iterator = 0;
    int iteratorNum = 0;
    int iteratorMinus = 0;
    for(int i = 0; i < 1000;i++){
        if(input[i] == '\n'){
            if(iteratorMinus==1)iteratorNum*=-1;
            output[iterator] = iteratorNum;
            return;
        }
        else if(input[i] == ','){
            if(iteratorMinus==1)iteratorNum*=-1;
            output[iterator] = iteratorNum;
            iterator++;
            iteratorMinus = 0;
            iteratorNum=0;
        }
        else if(input[i] == '-'){iteratorMinus = 1;}
        else if(48 <= input[i] && input[i] <= 57){
            int charaNum = input[i] - 48;
            iteratorNum *= 10;
            iteratorNum += charaNum;
        }
    }
}

void interpritorf(char *input, double *output){

    int iterator = 0;
    double iteratorNum = 0;
    int iteratorMinus = 0;
    int iteratorPoint = 0;//小数点位置の計算
    for(int i = 0; i < 1000;i++){
        if(input[i] == '\n'){
            if(iteratorMinus==1)iteratorNum*=-1.;
            output[iterator] = iteratorNum;
            return;
        }
        else if(input[i] == ','){
            if(iteratorMinus==1)iteratorNum*=-1.;
            output[iterator] = iteratorNum;
            iterator++;
            iteratorMinus = 0;
            iteratorPoint = 0;
            iteratorNum=0.;
        }
        else if(input[i] == '.'){
            if(iteratorPoint==0)iteratorPoint = 1;
        }
        else if(input[i] == '-'){iteratorMinus = 1;}
        else if(48 <= input[i] && input[i] <= 57){
            int charaNum = input[i] - 48;
            if(iteratorPoint==0){
                iteratorNum *= 10.;
                iteratorNum += charaNum;
            }else{
                iteratorNum += (double)pow(10.,-iteratorPoint)*charaNum;
                iteratorPoint++;
            }
            
        }
    }
}
#endif

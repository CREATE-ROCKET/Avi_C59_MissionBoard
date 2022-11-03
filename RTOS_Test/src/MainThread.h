/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    This program is StateMachine for C59J
    
*/


#pragma once


/*---------------- set time ----------------*/
#define BURNING_TIME 1250 // - 100 flame (launch detection)
#define GLIDING_TIME 3900
#define FALLING_TIME 10000
/*------------------------------------------*/

#define SD_USE (1)
#define MPU_USE (1)
#define PIT_USE (0)
#define FLASH_USE (0)

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


#define SDQUEUE 512

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



class  MainThread : public ProcessInterface
{
private:
    int state;
    double dt;
    double dpsi;
    double euler[3];

    int now,pre_now,real_dt;
    
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
    int IRAM_ATTR launchDetection();//0 not launch : 1 launch
    void IRAM_ATTR GetData();
    void CalibGyro();
    uint8_t calc_Euler_AngularVelocity();
    int check30deg();


public:
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

    Serial.begin(115200);
    // Serial2.begin(115200);
    pinMode(LEDPIN,OUTPUT);
    digitalWrite(LEDPIN, HIGH);

    state = 0;
    dt = 0.002;
    real_dt = 0;

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
    mpu.begin(&spi, MPUCS, F1MHZ);
    #endif

    CalibGyro();

    maxon.begin(32,27,22,34);

    //SD process start
#if SD_USE
    SDIOLogWrapper::makeQueue(SDQUEUE*4);
    SDIOLogWrapper::setSaveFileName("/aiueo.csv");
    SDIOLogWrapper::setSaveInterval(100);

    Serial.println("start");
    Serial.println(SDIOLogWrapper::initSD());
    SDIOLogWrapper::openFile();
    SDIOLogWrapper::writeTaskCreate(PRO_CPU_NUM);
#endif
    
}

void IRAM_ATTR MainThread::processDebug(){
    
}

void IRAM_ATTR MainThread::processPhase0(){
    static int logIte = 0;
    
    if(Serial.available()){
        
        char command = Serial.read();
        if(command == 'g'){
            Serial.println("Begin Gyro Calibration");
            CalibGyro();
            Serial.println("Finish calibration");
        }
        else if(command == 'q'){
            Serial.println("Set Q to initial value");
            q.init();
        }
        else if(command == 'f'){
            Serial.println("Set Fin 0 position");
        }
        else if(command == 'b'){
            Serial.println("Begin Bulk Erase");
            flash.erase();
        }
        else if(command == 's'){
            
            Serial.println("Switch to Log mode");
            Serial.println("Set state 1");

#if SD_USE
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
    ctrl.update(real_dt, euler[2], dpsi, 80., &maxon);

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
void IRAM_ATTR MainThread::processPhase1(){
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }

    GetData();

    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, 80., &maxon);

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
void IRAM_ATTR MainThread::processPhase2(){
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        return;
    }

    GetData(); // around 50 ~ 100us but mag read 400us

    q.update(mpuDataGyro,real_dt);// < 100us
    q.calc_Euler(euler);
    calc_Euler_AngularVelocity();
    maxon.update();
    ctrl.update(real_dt, euler[2], dpsi, 80., &maxon);

    
    logSPIflash();// < 800 us
    logSD(); // 300 ~ 800 us
    
    

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
void IRAM_ATTR MainThread::processPhase3(){
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
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
    ctrl.update(real_dt, euler[2], dpsi, 80., &maxon);

    if(check30deg()==-1)ctrl.stop();


    logSPIflash();
    logSD();


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
void IRAM_ATTR MainThread::processPhase4(){
    static int logIte = 0;
    if(Serial.available() && Serial.read() == 'c'){
        logIte = 0;
        state = 0;
        Serial.println("Cancel logging and switch Phase0");
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
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
    ctrl.update(real_dt, euler[2], dpsi, 80., &maxon);


    logSPIflash();
    logSD();


    if(logIte > FALLING_TIME){
        logIte = 0;
        state = 0;
        
#if SD_USE
        SDIOLogWrapper::writeTaskDelete();
        SDIOLogWrapper::closeFile();
        SDIOLogWrapper::deinitSD();
#endif
        Serial.println("Set state 0");
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
    if((now - lastFetchMag) > 10000 && mpu.GetMag(mpuMagDataInt) == 0){
        lastFetchMag = now;
        mpuDataMag[0] = mpuMagDataInt[0]*0.6f;
        mpuDataMag[1] = mpuMagDataInt[1]*0.6f;
        mpuDataMag[2] = mpuMagDataInt[2]*0.6f;
    }
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
        
        if(aveAcc > 1.5){
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
    static uint32_t addr = 0;
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

    //Q data 15 ~ 22
    uint16_t Qint[4];
    for(int i = 0;i < 4;i++){
        Qint[i] = (uint16_t)(q.v[i] * 1000);
        
        Flashbuf[slice + 15 + 2*i] = Qint[i] >> 8;
        Flashbuf[slice + 16 + 2*i] = Qint[i];
    }

    //time stamp
    Flashbuf[slice + 81] = now >> 24;
    Flashbuf[slice + 82] = now >> 16;
    Flashbuf[slice + 83] = now >> 8;
    Flashbuf[slice + 84] = now;

    if(logCnt == 2){
        logCnt=-1;
        addr += 0x100;
        flash.write(addr,Flashbuf);
        for(int i = 0;i<256;i++)Flashbuf[i] = 0;
    }
    logCnt++;
#endif
}

void IRAM_ATTR MainThread::logSD(){
#if SD_USE
    //500 ~ 600 us
    
    char *bfChar = new char[SDQUEUE];
    
    bfChar[0] = '\0';
    snprintf(bfChar,SDQUEUE,
            "%d,\
             %d,\
             %8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,\   
             %8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,%8.5lf,%8.5lf,\
             %d,\
             %d\
             \n",
            state,
            pitotDataInt,
            pitoic.WindSpeedFiliter,
            mpuDataAcc[0],  mpuDataAcc[1],  mpuDataAcc[2],
            mpuDataGyro[0], mpuDataGyro[1], mpuDataGyro[2],
            // mpuDataMag[0],  mpuDataMag[1],  mpuDataMag[2],
            euler[0],       euler[1],       euler[2],       
            q.v[0],         q.v[1],         q.v[2],         q.v[3],
            dpsi,           maxon.setSpd,   maxon.rad,      ctrl.Mr,         ctrl.fin_angle_r,
            real_dt,
            now
            );
    
    // Serial.print(bfChar);
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

int MainThread::check30deg(){
    double vec[3] = {0., 0., 1.};
    double launcherVec[3] = {0., 0., 1.,};
    double res[3];
    q.vecRotate(vec, res);

    double dot = res[0] * launcherVec[0] + res[1] * launcherVec[1] + res[2] * launcherVec[2];
    if(acos(dot) / 3.14 * 180. > 30.)return -1;
    return 0;
}
#endif

/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    This program is StateMachine for C59J
    
*/


#pragma once


/*---------------- set time ----------------*/
#define BURNING_TIME 2000
#define GLIDING_TIME 4000
#define FALLING_TIME 10000
/*------------------------------------------*/

#ifndef MT_H
#define MT_H
#include <ProcessInterface.h>
#include <Arduino.h>

#include <SDIOLogWrapper.h>
#include <MPU9250.h>
#include <SPIflash.h>
#include <Q.h>

#define LEDPIN 5

#define FLASHCS 33
#define LPSCS 25
#define MPUCS 26
#define PITCS 21

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
    int pitotDataInt;
    int16_t mpuGyroCalib[3];

    char SDbuf[128];
    SPICreate spi;
    MPU mpu;Flash flash;SDIOLogWrapper SDIO;

    void processDebug();
    void processPhase0();
    void processPhase1();
    void processPhase2();
    void processPhase3();
    void processPhase4();

    
    void checkDevice();
    void logSPIflash();
    void logSD();
    int launchDetection();//0 not launch : 1 launch
    void GetData();
    void CalibGyro();
    uint8_t calc_Euler_AngularVelocity();


public:
    double mpuDataAcc[3];
    double mpuDataGyro[3];
    double pitotData;
    double estPose[4];
    Q q;

    void begin();
    void tickProcess();
};

void MainThread::begin(){
    Serial.begin(115200);
    // Serial1.begin(115200,16,17);
    Serial2.begin(115200);
    pinMode(LEDPIN,OUTPUT);
    state = -1;
    dt = 0.002;
    

    //SPI setup
    spi = SPICreate();
    spi.begin(VSPI,CLK,MISO,MOSI,F40MHZ);

    //flash setup
    flash = Flash();
    flash.begin(&spi,FLASHCS,F40MHZ);

    //accerolometer setup
    mpu = MPU();
    mpu.begin(&spi,MPUCS,F10MHZ);
    
    CalibGyro();
    

    //SD process start
    // SDIO.makeQueue(64, 128);
    // SDIO.initSD();
    // SDIO.openFile();
    // SDIO.writeTaskCreate(APP_CPU_NUM);
}
void IRAM_ATTR MainThread::processDebug(){
    static int logIte = 0;
    static double z = 0;

    double euler[3];
    GetData();
    q.update(mpuDataGyro);
    q.calc_Euler(euler);

    z += mpuDataGyro[2] * dt;

    if(logIte % 50 == 0){
        // Serial.print((int)(mpuDataGyro[0]*1000));Serial.print("\t");
        // Serial.print((int)(mpuDataGyro[1]*1000));Serial.print("\t");
        // Serial.println((int)(mpuDataGyro[2]*1000));

        // Serial.print((int)(euler[0]*1000));Serial.print("\t");
        // Serial.print((int)(euler[1]*1000));Serial.print("\t");
        // Serial.println((int)(euler[2]*1000));

        // Serial.print((int)(q.v[0]*1000));Serial.print("\t");
        // Serial.print((int)(q.v[1]*1000));Serial.print("\t");
        // Serial.print((int)(q.v[2]*1000));Serial.print("\t");
        // Serial.println((int)(q.v[3]*1000));

        logSD();
    }

    logIte++;
}

void IRAM_ATTR MainThread::processPhase0(){
    static int logIte = 0;
    static int inTask = 0;
    
    
    if(Serial.available() && (inTask == 0)){
        
        char command = Serial.read();
        inTask = 1;
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
            state = 1;
            logIte = 0;
        }
        else if(command == '\n'){
        }
        else{
            Serial.println("[E] Invalid command detect!\n[E] ");
            Serial.print(command);
            Serial.print("\n");
        }
        inTask = 0;
    }

    if(logIte % 500 == 0){
        Serial.println("Phase 0");
        Serial2.println("Phase 0");
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
        return;
    }

    GetData();
    
    if(launchDetection()){
        logIte = 0;
        state = 2;
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
    // if(Serial.available() && Serial.read() == 'c'){
    //     logIte = 0;
    //     state = 0;
    //     Serial.println("Cancel logging and switch Phase0");
    //     return;
    // }

    GetData();
    q.update(mpuDataGyro);

    int startTime = micros();
    logSPIflash();
    int endTime = micros();

    Serial.println(endTime - startTime);



    if(logIte > BURNING_TIME){
        logIte = 0;
        state = 3;
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
    // if(Serial.available() && Serial.read() == 'c'){
    //     logIte = 0;
    //     state = 0;
    //     Serial.println("Cancel logging and switch Phase0");
    //     return;
    // }

    GetData();
    q.update(mpuDataGyro);

    logSPIflash();


    if(logIte > GLIDING_TIME){
        logIte = 0;
        state = 4;
        return;
    }

    //LED
    digitalWrite(LEDPIN,HIGH);
    logIte++;
}
void IRAM_ATTR MainThread::processPhase4(){
    static int logIte = 0;
    // if(Serial.available() && Serial.read() == 'c'){
    //     logIte = 0;
    //     state = 0;
    //     Serial.println("Cancel logging and switch Phase0");
    //     return;
    // }

    GetData();
    q.update(mpuDataGyro);


    logSPIflash();


    if(logIte > FALLING_TIME){
        logIte = 0;
        state = 0;
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

    int startTask = micros();
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

    int endTask = micros();
    if(endTask-startTask> 1000){
        // Serial.println(endTask-startTask);
    }
    
}
void IRAM_ATTR MainThread::GetData(){
    mpu.Get(mpuDataInt);

    //Scale Adjusting Acc 16G Gyro 2500dps to 1G(double) 1 rad(double)
    mpuDataAcc[0] = mpuDataInt[0] * 0.00048828125f;
    mpuDataAcc[1] = mpuDataInt[1] * 0.00048828125f;
    mpuDataAcc[2] = mpuDataInt[2] * 0.00048828125f;

    mpuDataGyro[0] = (mpuDataInt[3] - mpuGyroCalib[0]) * 0.06103515625f * 3.141592 / 180.0f;
    mpuDataGyro[1] = (mpuDataInt[4] - mpuGyroCalib[1]) * 0.06103515625f * 3.141592 / 180.0f;
    mpuDataGyro[2] = (mpuDataInt[5] - mpuGyroCalib[2]) * 0.06103515625f * 3.141592 / 180.0f;
}
int IRAM_ATTR MainThread::launchDetection(){
    static int Ite = 0;
    static double aveAcc = 0;

    double AccScal = mpuDataAcc[0]*mpuDataAcc[0] + mpuDataAcc[1]*mpuDataAcc[1] + mpuDataAcc[2]*mpuDataAcc[2];
    aveAcc += sqrt(AccScal)/20.0f;
    Ite++;

    if(Ite==20){
        Ite = 0;
        
        if(aveAcc > 2.0){
            Ite = 0;
            aveAcc = 0;
            return 1;
        }
        else{
            aveAcc = 0;
        }     
    }
    return 0;
}
void IRAM_ATTR MainThread::logSPIflash(){
    int d[64];

    d[0] = micros();//time stamp
    //Acc
    d[1] = (int)(mpuDataAcc[0] * 1000);
    d[2] = (int)(mpuDataAcc[1] * 1000);
    d[3] = (int)(mpuDataAcc[2] * 1000);
    //Gyro
    d[4] = (int)(mpuDataGyro[0] * 1000);
    d[5] = (int)(mpuDataGyro[1] * 1000);
    d[6] = (int)(mpuDataGyro[2] * 1000);
    //Q
    d[7] = (int)(q.v[0] * 1000);
    d[8] = (int)(q.v[1] * 1000);
    d[9] = (int)(q.v[2] * 1000);
    d[10] = (int)(q.v[3] * 1000);

    delay(1);
    
    flash.putInt(d,11);
}

void IRAM_ATTR MainThread::logSD(){
    int d[64];

    sprintf(SDbuf,
            "%8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,\
             %8.5lf,%8.5lf,%8.5lf,%8.5lf\
             \n",
            mpuDataAcc[0], mpuDataAcc[1], mpuDataAcc[2],
            mpuDataGyro[0], mpuDataGyro[1], mpuDataGyro[2],
            q.v[0], q.v[1], q.v[2], q.v[3] 
            );
    
    Serial.print(SDbuf);

    // d[0] = micros();//time stamp
    // //Acc
    // d[1] = (int)(mpuDataAcc[0] * 1000);
    // d[2] = (int)(mpuDataAcc[1] * 1000);
    // d[3] = (int)(mpuDataAcc[2] * 1000);
    // //Gyro
    // d[4] = (int)(mpuDataGyro[0] * 1000);
    // d[5] = (int)(mpuDataGyro[1] * 1000);
    // d[6] = (int)(mpuDataGyro[2] * 1000);
    // //Q
    // d[7] = (int)(q.v[0] * 1000);
    // d[8] = (int)(q.v[1] * 1000);
    // d[9] = (int)(q.v[2] * 1000);
    // d[10] = (int)(q.v[3] * 1000);

    delay(1);
    
    flash.putInt(d,11);
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
	double omega_x,omega_y,omega_z;
	uint8_t Singularity_check = 0;
    double euler[3];

    q.calc_Euler(euler);//phi theta psi

	dpsi = sin(euler[2]) * tan(euler[0]) * mpuDataGyro[0] + cos(euler[2]) * tan(euler[0]) * mpuDataGyro[1] + mpuDataGyro[2];

	if ((euler[0] > M_PI/2.-0.1) || (euler[0] < -M_PI/2.+0.1)){ //特異点近傍
		Singularity_check = 1;
	}else{
		Singularity_check = 0;
	}
	return Singularity_check;
}

#endif

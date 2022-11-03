/*
 Author : Yuta Takayasu
 Date : 2022/5/14

    Library for operating barometer with ESP32.
    It depends on SPICREATE.h and SPICREATE.h
*/


#pragma once

#ifndef MPU_H
#define MPU_H
#include <SPICREATE.h>
#include <Arduino.h>

#define MPU_Data_Adress 0x3B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_WhoAmI_Adress 0x75
#define MPU_ACC_CONFIG 0x1C
#define MPU_WakeUp_Adress 0x6B
#define MPU_16G 0b00011000
#define MPU_8G 0b00010000
#define MPU_4G 0b00001000
#define MPU_2G 0b00000000
#define MPU_2000dps 0b00011000
#define MPU_1000dps 0b00010000
#define MPU_500dps 0b00001000
#define MPU_250dps 0b00000000

#define MPU_2500deg 0x18

class MPU
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *MPUSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 1000000);
    uint8_t WhoAmI();
    void CheckReg();
    void IRAM_ATTR Get(int16_t *rx);
    int IRAM_ATTR GetMag(int16_t *rx);
};

void MPU::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{
    pinMode(cs,OUTPUT);
    digitalWrite(cs,HIGH);
    delay(10);
    digitalWrite(cs,LOW);
    CS = cs;
    MPUSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    //if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = SPI_MODE3;
    if_cfg.queue_size = 1;

    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = MPUSPI->addDevice(&if_cfg, cs);

    
    CheckReg();

    //Init
    delay(100);

    MPUSPI->setReg(MPU_WakeUp_Adress,0x80,deviceHandle);//リセット
    delay(1);
    
    MPUSPI->setReg(0x6A,0x80,deviceHandle);//I2Cの利用を禁止
    delay(1);

    MPUSPI->setReg(0x23,0x00, deviceHandle);//mpuの設定　FIFOにしない
    delay(1);

	MPUSPI->setReg(0x6A,0x30, deviceHandle);//mpuの内部I2C有効化
    delay(1);

	MPUSPI->setReg(0x24,0x5D, deviceHandle);//mpuがI2Cでデータ読み込み中に割り込みさせない＋センサとのやり取りでSCL400KHzにする（地磁気センサの最高速mode2の最大SCL） ＋ストップビットを入れる（地磁気センサ曰く「命令終了時にはストップコンディションを入力してください。」とのこと）
	delay(1);

    MPUSPI->setReg(0x25,0x0C, deviceHandle);//I2Cスレーブ0のアドレスと地磁気センサのスレーブアドレスの対応
	delay(1);

    MPUSPI->setReg(0x26,0x0A, deviceHandle);//mpuがI2Cで書き込む対象を決めるアドレスと地磁気センサ制御アドレスの対応
	delay(1);

    MPUSPI->setReg(0x63,0x16, deviceHandle);//mpuがI2Cで書き込む内容を決めるアドレスと地磁気センサ制御内容（16ビットでやり取り＋地磁気センサ最高速mode2データ100Hz）
	delay(1);

    MPUSPI->setReg(0x27,0x97, deviceHandle);//mpuがI2Cでの初期設定書き込みやデータのやり取りをmpu内のスレーブ0で行うこととする＋データは偶数ビットあるとする＋データ量7バイトでやり取り
    delay(1);

    MPUSPI->setReg(MPU_ACC_CONFIG, MPU_16G, deviceHandle);//set range 16G
    delay(1);

    MPUSPI->setReg(0x19, 1, deviceHandle);
    delay(1);

    MPUSPI->setReg(0x1A, 1, deviceHandle);
    delay(1);

    MPUSPI->setReg(MPU_GYRO_CONFIG, MPU_2000dps, deviceHandle);//set range 2000 deg/s
    delay(1);


    return;
}
uint8_t MPU::WhoAmI()
{
    return MPUSPI->readByte(0x80 | 0x75, deviceHandle);
}


void MPU::CheckReg(){
    while(WhoAmI()!=113){
        Serial.print("Wrong Who Am I MPU ");
        Serial.println(WhoAmI());
        delay(1);
        // MPUSPI->setReg(MPU_WakeUp_Adress,0x80,deviceHandle);//リセット
        delay(1);
        MPUSPI->setReg(0x6A,0x80,deviceHandle);//I2Cの利用を禁止
        
        delay(1000);
    }
    return;
}
void IRAM_ATTR MPU::Get(int16_t *rx)
{
    uint8_t rx_buf[14];
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (14) * 8;
    comm.cmd = MPU_Data_Adress | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    MPUSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);

    rx[0] = (rx_buf[0] << 8 | rx_buf[1]);
    rx[1] = (rx_buf[2] << 8 | rx_buf[3]);
    rx[2] = (rx_buf[4] << 8 | rx_buf[5]);
    rx[3] = (rx_buf[8] << 8 | rx_buf[9]);
    rx[4] = (rx_buf[10] << 8 | rx_buf[11]);
    rx[5] = (rx_buf[12] << 8 | rx_buf[13]);
    return;
}

int IRAM_ATTR MPU::GetMag(int16_t *rx){
    MPUSPI->setReg(0x25, 0x0c|0x80,  deviceHandle);//スレーブアドレスを読み取り用へ
    MPUSPI->setReg(0x26, 0x03,       deviceHandle);//I2Cを用いて地磁気センサとmpuのやり取りするアドレスの対応
    MPUSPI->setReg(0x27, 0x97,       deviceHandle);//地磁気センサからmpuへのデータの読み出し
    uint8_t s;
    s = MPUSPI->readByte(0x80 | 0x3A, deviceHandle);
    if(s&0x01){//データが準備できているか
        uint8_t rx_buf[7];
        spi_transaction_t comm = {};
        comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
        comm.length = (7) * 8;
        comm.cmd = 0x49 | 0x80;

        comm.tx_buffer = NULL;
        comm.rx_buffer = rx_buf;
        comm.user = (void *)CS;

        spi_transaction_ext_t spi_transaction = {};
        spi_transaction.base = comm;
        spi_transaction.command_bits = 8;
        MPUSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);


    	// MPU_Read_Bytes(0x49,Receive_Data,7);//EXT_SENS_DATA_00からI2Cのデータが入る　最後のデータはST2（データ状態）で読み込まないと更新されない
    	rx[0]=(rx_buf[1]<<8 | rx_buf[0]);
		rx[1]=(rx_buf[3]<<8 | rx_buf[2]);
		rx[2]=(rx_buf[5]<<8 | rx_buf[4]);
		return 0;
    }else{
    	return 1;
    }
}
#endif

/*
 Author : Yuta Takayasu
 Date : 2022/5/14

    Library for operating barometer with ESP32.
    It depends on SPICREATE.h and SPICREATE.h
*/


#pragma once

#ifndef LPS_H
#define LPS_H
#include <SPICREATE.h>
#include <Arduino.h>

#define LPS_Data_Adress_0 0x28
#define LPS_Data_Adress_1 0x29
#define LPS_Data_Adress_2 0x2A

#define LPS_WakeUp_Adress 0x10
#define LPS_WakeUp_Value 0x40
#define LPS_WhoAmI_Adress 0x0F

class LPS
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *LPSSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoAmI();
    void Get(int *rx);
    // (uint32_t)rx[2] << 16 | (uint32_t)rx[1] << 8 | (uint32_t)rx[0] means pressure
};

void LPS::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{
    CS = cs;
    LPSSPI = targetSPI;
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

    deviceHandle = LPSSPI->addDevice(&if_cfg, cs);
    Serial.println(deviceHandle);
    LPSSPI->setReg(LPS_WakeUp_Adress, LPS_WakeUp_Value, deviceHandle);
    delay(1);
    return;
}
uint8_t LPS::WhoAmI()
{
    LPSSPI->setReg(LPS_WakeUp_Adress, LPS_WakeUp_Value, deviceHandle);
    return LPSSPI->readByte(LPS_WhoAmI_Adress | 0x80, deviceHandle);
    //registor 0x0F and you'll get 0d177 or 0xb1 or 0b10110001
}

void LPS::Get(int *rx)
{
    uint8_t rx_buf[3];
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (3) * 8;
    comm.cmd = LPS_Data_Adress_0 | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;

    LPSSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);

    // uint8_t rxsub[3];
    // rxsub[0] = LPSSPI->readByte(LPS_Data_Adress_0 | 0x80, deviceHandle);
    // rxsub[1] = LPSSPI->readByte(LPS_Data_Adress_1 | 0x80, deviceHandle);
    // rxsub[2] = LPSSPI->readByte(LPS_Data_Adress_2 | 0x80, deviceHandle);
    // (uint32_t)RecieveData[2] << 16 | (uint16_t)RecieveData[1] << 8 | RecieveData[0];
    *rx = (int)(rx_buf[0] | (uint16_t)rx_buf[1] << 8 | rx_buf[2] << 16);//(rx_buf[2] | rx_buf[1] << 8 | rx_buf[0] << 16);
    //*rx = 0;
    //*rx = (int)(rx_buf[3] | rx_buf[4] << 8);
    
    return;
}

#endif

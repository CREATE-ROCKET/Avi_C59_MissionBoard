#pragma once

#ifndef SPIFlash_H
#define SPIFlash_H
#include <SPICREATE.h>
#include <Arduino.h>

using namespace arduino::esp32::spi::dma;

#define CMD_RDID 0x9f
#define CMD_READ 0x03
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_P4E 0x20
#define CMD_P8E 0x40
#define CMD_BE 0x60
#define CMD_PP 0x02
#define CMD_RDSR 0x05

class Flash
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *flashSPI;

public:
    uint32_t currentAdd;
    int pageUsage;
    uint8_t dataBuf[256];

    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    void erase();

    void putByte(uint8_t *data,int size);
    void putInt(int *data,int size);

    void write(uint32_t addr, uint8_t *tx);
    void read(uint32_t addr, uint8_t *rx);
};

void Flash::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{

    currentAdd = 0;
    pageUsage = 0;
    for(int i = 0;i < 256;i++)dataBuf[i] = 0;

    CS = cs;
    flashSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    //if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    //if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;
    if_cfg.command_bits = 0;
    if_cfg.address_bits = 0;
    if_cfg.mode = SPI_MODE3;
    if_cfg.queue_size = 1;
    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = flashSPI->addDevice(&if_cfg, cs);
    uint8_t readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);

    while (readStatus != 0)
    {
        readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);
        delay(100);
    }
    delay(100);
    return;
}
void Flash::erase()
{
    Serial.println("start erase");
    if (flashSPI == NULL)
    {
        return;
    }

    flashSPI->sendCmd(CMD_WREN, deviceHandle);
    flashSPI->sendCmd(CMD_BE, deviceHandle);
    uint8_t readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);
    while (readStatus != 0)
    {
        readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);
        Serial.print(",");
        delay(500);
    }
    Serial.println("Bulk Erased");
    return;
}



void  IRAM_ATTR Flash::putByte(uint8_t *data,int size){
    if((pageUsage/256) > 65535){
        Serial.print("[E] SPI flash overflow!");
        return;
    }

    if((pageUsage + size) > 256){
        for(int i = 0; (i+pageUsage) < 256; i++){
            dataBuf[i + pageUsage] = data[i];
        }

        write(currentAdd,dataBuf);

        for(int i = 0;i < 256;i++)dataBuf[i] = 0;

        for(int i = 0; i < (pageUsage + size - 256); i++){
            dataBuf[i] = data[i + (256 - pageUsage)];
        }

        pageUsage = pageUsage + size - 256;
        currentAdd += 256;

    }else{
        for(int i = 0; i < size; i++)dataBuf[i + pageUsage] = data[i];
        pageUsage += size;
    }
}

void IRAM_ATTR Flash::putInt(int *data,int size){
    uint8_t byteData[256];

    for(int i = 0;i < size;i++){
        byteData[i * 4 + 0] = data[i] << 24;
        byteData[i * 4 + 1] = data[i] << 16;
        byteData[i * 4 + 2] = data[i] << 8;
        byteData[i * 4 + 3] = data[i];
    }

    putByte(byteData,size*4);

}

void IRAM_ATTR Flash::write(uint32_t addr, uint8_t *tx)
{
    flashSPI->sendCmd(CMD_WREN, deviceHandle);
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (256) * 8;
    comm.cmd = CMD_PP;
    comm.addr = addr;
    comm.tx_buffer = tx;
    comm.user = (void *)&CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    spi_transaction.address_bits = 24;

    flashSPI->transmit((spi_transaction_t *)&spi_transaction, deviceHandle);
    return;
}
void Flash::read(uint32_t addr, uint8_t *rx)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (256) * 8;
    comm.cmd = CMD_READ;
    comm.addr = addr;
    comm.tx_buffer = NULL;
    comm.rx_buffer = rx;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    spi_transaction.address_bits = 24;
    flashSPI->transmit((spi_transaction_t *)&spi_transaction, deviceHandle);
}

#endif

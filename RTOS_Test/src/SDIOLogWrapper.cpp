#include "SDIOLogWrapper.h"

bool SDIOLogWrapper::isSDOpend = 0;
File SDIOLogWrapper::logFile;
QueueHandle_t SDIOLogWrapper::xQueue;
TaskHandle_t SDIOLogWrapper::xWriteSDHandle;
int SDIOLogWrapper::logCounterMax = 1024;
int SDIOLogWrapper::xQueueSize;
const char SDIOLogWrapper::fileName[32] = "/logger.csv";

int SDIOLogWrapper::initSD()
{
    if (isSDOpend)
    {
        return 0;
    }
    if (!SD_MMC.begin())
    {
        return 1;
    }
    isSDOpend = 1;
    return 0;
}

void SDIOLogWrapper::deinitSD()
{
    SD_MMC.end();
    isSDOpend = 0;
}

void SDIOLogWrapper::openFile()
{
    printf("file:%s", fileName);
    logFile = SD_MMC.open(fileName, FILE_APPEND);
}

void SDIOLogWrapper::writeFile(const char mData[])
{
    logFile.println(mData);
}

void SDIOLogWrapper::closeFile()
{
    logFile.close();
}

int SDIOLogWrapper::makeQueue(int uxQueueLength, int uxQueueSize)
{
    xQueueSize = uxQueueSize;
    xQueue = xQueueCreate(uxQueueLength, xQueueSize * sizeof(char));

    if (xQueue == NULL)
    {
        return 1;
    }
    return 0;
}

void SDIOLogWrapper::deleteQueue()
{
    vQueueDelete(xQueue);
}

IRAM_ATTR int SDIOLogWrapper::countWaitingQueue()
{
    return uxQueueMessagesWaiting(xQueue);
}

IRAM_ATTR int SDIOLogWrapper::appendQueue(char xData[])
{
    if (uxQueueSpacesAvailable(xQueue) < 1)
    {
        return 1;
    }
    else
    {
        xQueueSend(xQueue, xData, 0);
    }
    return 0;
}

IRAM_ATTR void SDIOLogWrapper::writeSDfromQueue(void *parameters)
{
    for (;;)
    {
        char data[xQueueSize];
        int logCounter = 0;
        if (xQueueReceive(xQueue, &data, 0) == pdTRUE)
        {
            logFile.print(data);
            logCounter++;
            if (logCounter == logCounterMax)
            {
                logCounter = 0;
                logFile.close();
                logFile = SD_MMC.open(
                    fileName, FILE_APPEND);
            }
        }
        else
        {
            delay(1);
        }
    }
}

void SDIOLogWrapper::setSaveInterval(int interval)
{
    logCounterMax = interval;
}

void SDIOLogWrapper::writeTaskCreate(int TaskExecuteCore)
{
    xTaskCreateUniversal(writeSDfromQueue, "Queue2SD", 8192, NULL, 1, &xWriteSDHandle, TaskExecuteCore);
}

void SDIOLogWrapper::writeTaskDelete()
{
    vTaskDelete(xWriteSDHandle);
}
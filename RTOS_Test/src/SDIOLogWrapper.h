#include <FS.h>
#include <SD_MMC.h>

#ifndef SDIOLOGWRAPPER
#define SDIOLOGWRAPPER

class SDIOLogWrapper
{
private:
    static bool isSDOpend;
    static File logFile;
    static TaskHandle_t xWriteSDHandle;
    static const char fileName[32];
    static int xQueueSize;
    static QueueHandle_t xQueue;
    static int logCounterMax;

public:
    static int initSD();
    static void deinitSD();

    static void openFile();
    static void writeFile(const char mData[]);
    static void closeFile();

    static int makeQueue(int uxQueueLength, int uxQueueSize);
    static int appendQueue(char xData[]);
    static int countWaitingQueue();
    static void deleteQueue();

    static void writeSDfromQueue(void *parameters);

    static void setSaveInterval(int interval);
    static void writeTaskCreate(int TaskExecuteCore);
    static void writeTaskDelete();
};
#endif
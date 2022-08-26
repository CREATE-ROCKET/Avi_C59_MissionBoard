#include <Arduino.h>
#include <SPICREATE.h>
#include <MainThread.h>


#define loggingPeriod 2

TimerHandle_t thand_test;
xTaskHandle xlogHandle;

MainThread mainThread;

IRAM_ATTR void logging(void *parameters)
{
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    char bfChar[128] = "very tekitou na data\n";

    // if (SDIO.appendQueue(bfChar) == 1)
    // {
    //   vTaskDelete(&xlogHandle);
    //   Serial.println("queue filled!");
    //   ESP.restart();
    // }
    mainThread.tickProcess();
    vTaskDelayUntil(&xLastWakeTime, loggingPeriod / portTICK_PERIOD_MS);
  }
}

void setup()
{
  mainThread = MainThread();


  mainThread.begin();


  xTaskCreateUniversal(logging, "logging", 8192, NULL, 1, &xlogHandle, PRO_CPU_NUM);
}

void loop()
{
}
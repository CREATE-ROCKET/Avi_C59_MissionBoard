#include <Arduino.h>
#include <SPICREATE.h>
#include <MainThread.h>



#define loggingPeriod 2

TimerHandle_t thand_test;
xTaskHandle xlogHandle;

MainThread mainThread;

int tickflag = 0;

IRAM_ATTR void logging(void *parameters)
{
  
  for (;;)
  {
    portTickType xLastWakeTime = xTaskGetTickCount();
    if(mainThread.inTask ==0){
      mainThread.tickProcess();
    }
    vTaskDelayUntil(&xLastWakeTime, loggingPeriod / portTICK_PERIOD_MS);
  }
}

IRAM_ATTR void tick()
{
  tickflag = 1;
}


hw_timer_t * timer = NULL; 
void setup()
{
  // setCpuFrequencyMhz(240);
  disableCore0WDT();

  mainThread = MainThread();
  mainThread.begin();


  // xTaskCreateUniversal(logging, "logging", 1024 * 16, NULL, 1, &xlogHandle, PRO_CPU_NUM);
  timer = timerBegin(0, 80, true); //timer=1us
  timerAttachInterrupt(timer, &tick, true);
  timerAlarmWrite(timer, 2000, true); 
  timerAlarmEnable(timer);
}


void loop()
{
  if(tickflag){
    if(mainThread.inTask == 0){
        mainThread.tickProcess();
    }
    tickflag = 0;
  }
  
}
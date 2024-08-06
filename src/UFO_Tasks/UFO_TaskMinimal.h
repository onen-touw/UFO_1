#pragma once
#include "UFO_Config.h"


enum UFO_TASKS_ID : uint8_t
{
    NO_TASK,
    TASK_WIFI_HANDL,
    TASK_IMU,
    TASK_BME,
    TASK_CONTROL,
    
    INITIALIZATION,
    TEST_TELEMETRY,
    MAIN,
};

struct UFO_TaskDescriptor
{
    const char *const tName;
    UFO_TASKS_ID ID;
    const uint32_t tStackSize;
    const uint8_t tPriority;
    const uint32_t tUpdatePeriodMs; // delay(200) (into task)
    void (*tFunction)(void *);
    const uint8_t tCoreID;
};


struct UFO_SendToFunckMinimal
{
    uint32_t updTime;
    UFO_TASKS_ID taskID;
    void *arg;
    const char* name;
};


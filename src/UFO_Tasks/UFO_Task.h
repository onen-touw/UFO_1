#pragma once

#include "UFO_TaskMinimal.h"
#include "UFO_TaskWiFiControl.h"
#include "UFO_TaskIMU.h"
#include "UFO_TaskBME.h"
#include "UFO_TaskControl.h"


// #include"UFO_Task_Gimbal.h"
// WARNING!!! ORDER should be same as in  UFO_TASKS_ID enum (whitout NO_TASK)
struct UFO_TaskDescriptor tasksDesc[] = {
    //  Name                ID                                      StackSize   Prior   Ms      Funck                   CoreID
    {"WiFi", UFO_TASKS_ID::TASK_WIFI_HANDL, 8192, 5, 100, UFO_Task_WiFiControl, 0},
    {"IMU", UFO_TASKS_ID::TASK_IMU, 4096, 5, 20, UFO_Task_IMU, 1},
    {"BME", UFO_TASKS_ID::TASK_BME, 2048, 5, 100, UFO_Task_BME, 1},
    // {"Control", UFO_TASKS_ID::TASK_CONTROL, 2048, 5, 100, UFO_Task_Control, 1},
    // {"Main", UFO_TASKS_ID::MAIN, 4096, 5, 100, UFO_Task_Main, 0},
};

void UFO_CreateTask(UFO_TASKS_ID id, void *dataToFunk = nullptr)
{
    uint8_t rID = static_cast<uint8_t>(id) - 1;
    // WARNING !!!delete should be in the end of task function!!!
    UFO_SendToFunckMinimal *data = new UFO_SendToFunckMinimal{tasksDesc[rID].tUpdatePeriodMs, id, dataToFunk, tasksDesc[rID].tName};

    xTaskCreatePinnedToCore(
        tasksDesc[rID].tFunction,
        tasksDesc[rID].tName,
        tasksDesc[rID].tStackSize,
        (void *)data,
        tasksDesc[rID].tPriority,
        NULL,
        tasksDesc[rID].tCoreID);
}
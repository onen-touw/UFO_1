#pragma once

#include "UFO_TaskMinimal.h"
#include "UFO_TaskIMU.h"
#include "UFO_TaskBME.h"
#include "UFO_TaskControl.h"
#include "UFO_TaskSocket.h"

// #include"UFO_Task_Gimbal.h"
// WARNING!!! ORDER should be same as in  UFO_TASKS_ID enum (whitout NO_TASK)
static struct UFO_TaskDescriptor UFO_TasksDesc[] = {
    //  Name                ID                                      StackSize   Prior   Ms      Funck                   CoreID
    {"SCK", UFO_TASKS_ID::TASK_SOCKET, 4096, 5, 10, UFO_Task_Socket, 1},
    {"IMU", UFO_TASKS_ID::TASK_IMU, 4096, 5, 20, UFO_Task_IMU, 1},
    {"BME", UFO_TASKS_ID::TASK_BME, 2048, 5, 100, UFO_Task_BME, 1},
    // {"Control", UFO_TASKS_ID::TASK_CONTROL, 2048, 5, 100, UFO_Task_Control, 1},
    // {"Main", UFO_TASKS_ID::MAIN, 4096, 5, 100, UFO_Task_Main, 0},
};

void UFO_CreateTask(UFO_TASKS_ID id, void *dataToFunk = nullptr)
{
    uint8_t rID = static_cast<uint8_t>(id) - 1;
    // WARNING !!!delete should be in the end of task function!!!
    UFO_SendToFunckMinimal *data = new UFO_SendToFunckMinimal{UFO_TasksDesc[rID].tUpdatePeriodMs, id, dataToFunk, UFO_TasksDesc[rID].tName};

    xTaskCreatePinnedToCore(
        UFO_TasksDesc[rID].tFunction,
        UFO_TasksDesc[rID].tName,
        UFO_TasksDesc[rID].tStackSize,
        (void *)data,
        UFO_TasksDesc[rID].tPriority,
        &UFO_TasksDesc[rID]._ptr,
        UFO_TasksDesc[rID].tCoreID);
}
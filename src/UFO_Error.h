
#pragma once
#include "UFO_Config.h"

enum UFO_ErrType :uint8_t
{
    UFO_ERR_NO,
    UFO_ERR_CRITICAL_ERROR,
    UFO_ERR_WARNING,
    UFO_ERR_COUNTING,
};

#define UFO_ERROR_WARNING_MSG       "Warning:: "
#define UFO_ERROR_CRITICAL_MSG      "Critical error:: "
#define UFO_ERROR_COUNTING_MSG      "Counting error:: "

struct UFO_ErrorMSG
{
    const char* _msg;
    const char* _place;
    UFO_ErrType _type = UFO_ERR_NO;
};
struct UFO_ErrorControlBlock
{
    UFO_ErrorMSG _emsg;
    UFO_ErrorMSG _emsgPrev;
    SemaphoreHandle_t _lock;
};

struct UFO_TaskPointerNode
{
    UFO_TaskPointerNode* _next;
};

class UFO_Error
{
private:

    
    /* data */
public:
    UFO_Error(/* args */) {}
    ~UFO_Error() {}
};
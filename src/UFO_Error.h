
#pragma once
#include "UFO_Config.h"

enum UFO_ErrType :uint8_t
{
    UFO_ERR_NO,
    UFO_ERR_CRITICAL_ERROR,
    UFO_ERR_WARNING,
    UFO_ERR_COUNTING,
};

#define UFO_ERROR_BUFFER_SIZE   8
#define UFO_ERROR_WARNING_MSG       "Warning:: "
#define UFO_ERROR_CRITICAL_MSG      "Critical error:: "
#define UFO_ERROR_COUNTING_MSG      "Counting error:: "

struct UFO_ErrorMSG
{
    const char* _msg;
    const char* _place;
    UFO_ErrType _type = UFO_ERR_NO;
    uint64_t _time = 0;
};
struct UFO_ErrorControlBlock
{
    UFO_ErrorMSG _emsg[UFO_ERROR_BUFFER_SIZE] = {};
    uint8_t _it = 0;
    SemaphoreHandle_t _lock;
};

class UFO_Error
{
private:

    
    /* data */
public:
    UFO_Error(/* args */) {}
    ~UFO_Error() {}
};
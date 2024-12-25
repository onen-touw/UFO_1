
#pragma once
#include "UFO_Config.h"
#include <iostream>

enum UFO_ErrType :uint8_t
{
    UFO_ERR_NO,
    UFO_ERR_CRITICAL_ERROR,
    UFO_ERR_WARNING,
    UFO_ERR_COUNTING,
};

#define UFO_ERROR_USE_TAG
#define UFO_ERROR_USE_TIMESTAMP

#define UFO_ERROR_BUFFER_SIZE       8
#define UFO_ERROR_WARNING_MSG       "Warning:: "
#define UFO_ERROR_CRITICAL_MSG      "Critical error:: "
#define UFO_ERROR_COUNTING_MSG      "Counting error:: "

struct UFO_ErrorMSG
{
#ifdef UFO_ERROR_USE_TAG
    const char* _tag;
#endif
    const char* _line;
    const char* _place;
    const char* _msg;
    UFO_ErrType _type = UFO_ERR_NO;

#ifdef UFO_ERROR_USE_TIMESTAMP
    uint64_t _time = 0;
#endif
};


struct UFO_ErrorControlBlock
{
    UFO_ErrorMSG _emsg[UFO_ERROR_BUFFER_SIZE] = {};
    uint8_t _it = 0;
    SemaphoreHandle_t _lock;
    UFO_ErrorMSG* GetLast(){}
};

/*
    FONT                    BACKGROUND
    Black           = 30, Black           = 40,
    Red             = 31, Red             = 41,
    Green           = 32, Green           = 42,
    Yellow          = 33, Yellow          = 43,
    Blue            = 34, Blue            = 44,
    Magenta         = 35, Magenta         = 45,
    Cyan            = 36, Cyan            = 46,
    White           = 37, White           = 47,
*/

#define UFO_TERMINAL_MSG_IN             "\033[1;"
#define UFO_TERMINAL_CMD_OUT             "m"
#define UFO_TERMINAL_MSG_OUT            "\033[0m"

#define UFO_TERMINAL_FONT_COLOR_RED     "31;"
#define UFO_TERMINAL_FONT_COLOR_GREEN   "32;"
#define UFO_TERMINAL_FONT_COLOR_BLUE    "34;"
#define UFO_TERMINAL_FONT_COLOR_WHITE   "37;"

#define UFO_TERMINAL_BACKGROUNF_COLOR_RED     "41"
#define UFO_TERMINAL_BACKGROUNF_COLOR_GREEN   "42"
#define UFO_TERMINAL_BACKGROUNF_COLOR_BLUE    "44"
#define UFO_TERMINAL_BACKGROUNF_COLOR_WHITE   "47"



class UFO_Error
{
private:
    UFO_ErrorControlBlock _e;
public:
    UFO_Error() {}
    ~UFO_Error() {}

    
private: 
    void __CallCritical(){
        
        std::cout                                           << 
        UFO_TERMINAL_MSG_IN                                 <<
        UFO_TERMINAL_FONT_COLOR_WHITE                       <<
        UFO_TERMINAL_BACKGROUNF_COLOR_RED                   <<
        UFO_TERMINAL_CMD_OUT                                <<
        "[CRITICAL]:"                                       <<        
#ifdef UFO_ERROR_USE_TAG
        _e._tag                                             <<
#endif
        ""
        /*   msg */
        UFO_TERMINAL_MSG_OUT                                <<
        '\n';
    }

    void __CallMsg(){
        std::cout << "";
    }
};
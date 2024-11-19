#include "UFO_Config.h"
#include "UFO_Utils.h"
#include "UFO_WiDTxCommon/UFO_TrsmStructDef.h"


enum UFO_SockType :uint8_t {
    UFO_SOCK_NO,
    UFO_SOCK_SERVER,
    UFO_SOCK_CLIENT,
};

struct UFO_SockConfigMinimal
{
    UFO_TrsmControlBlock* _TxBuf = nullptr;
    std::function<void(UFO_TrsmDataControlBlock* rcv)> _callbackFunc;
    uint16_t _port = 0;
    UFO_SockType _type = UFO_SOCK_NO;
    const char* _ip;
};

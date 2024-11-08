#pragma once

#include "UFO_Config.h"
#include "lwip/sockets.h"
#include "UFO_TaskClassBase.h"
#include "UFO_SockStructDefinition.h"


class UFO_Socket : public UFO_TaskClassBase
{
private:
    UFO_SockType _sType = UFO_SOCK_NO;

    char _addrStr[128];
    sockaddr_in _destAddr; 
    int32_t _port = -1;
    int32_t _sock = -1;

    socklen_t _socklen = 0;
    sockaddr_storage _sourceAddr; // Large enough for both IPv4 or IPv6

    int addr_family = 0;
    int ip_protocol = 0;

    bool __addrCatched = false;

    SockTxData_t* _trsm = nullptr;            //add setter
    SockDCB_t* _rcv = nullptr;
    int32_t _noResponceCount = 0;

    void* _alarm = nullptr;     // void* for a while          //add setter
    std::function<void(SockDCB_t* rcv)> _recvFunck;     //add setter

public:
    UFO_Socket(/* args */) {
       _socklen = sizeof(_sourceAddr);
       _port = 6464;        //default 
       _rcv = new SockDCB_t();
    }

    ~UFO_Socket()
    {
        if (_rcv)
        {
            delete _rcv;
        }
        this->Close();
    }

    void SetRecvFuck(std::function<void(SockDCB_t* rcv)> ff){
        _recvFunck = ff;
    }

    void SetTrsmCB(SockTxData_t* stdcb){
        _trsm = stdcb;
    }

    void SetType(UFO_SockType t){
        _sType = t;
    }

    void SetPort(int32_t port){

        Serial.println("Socket set port :");
        
        if (port < 0)
        {
            Serial.println("\n\tSocket Incorrect port");
            Serial.print("\t used default port: ");
            _port = 6464;
        }
        Serial.println(_port);
        _port = port;
    }

    void SetServerIP(const char* sip){
        _destAddr.sin_addr.s_addr = inet_addr(sip);
    }

    void SetConfig (UFO_SockConfigMinimal* conf) {
        _recvFunck = conf->_callbackFunc;
        _port = conf->_port;
        _sType = conf->_type;
        if (_sType == UFO_SOCK_CLIENT)
        {
            _destAddr.sin_addr.s_addr = inet_addr(conf->_ip);
        }
        _trsm = conf->_TxBuf;
    }

    virtual void Setup() final
    {
        esp_err_t err = ESP_OK;
        Serial.println("Socket setuping");
        Serial.println("\tChecking");

        if (_sType == UFO_SOCK_NO)
        {
            Serial.println("\t\tError: no sock type");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        if (!_trsm)
        {
            Serial.println("\t\tError: trsm cb is null");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        if (!_trsm->_lock)
        {
            Serial.println("\t\tError: trsm->lock cb is null");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        
        if (!_recvFunck)
        {
            Serial.println("\t\tError: no rcv fuck");
            // Serial.println("\t\t\tpassed...");
            while (true)
            {
                Serial.println("Critical error");
                delay(500);
            }
            return;
        }
        Serial.println("\tgood");

        Serial.println("Socket: Server creating");

        if (_sock >= 0)
        {
            // setCritical
            Serial.println("\tcreating fail");
            err = ESP_FAIL;
            // return err;
        }

        if (_port < 0)
        {
            // set critical
            Serial.println("\tcreating fail: incorrect port");
            err = ESP_FAIL;
            // return err;
        }

        if (_sType == UFO_SOCK_SERVER)
        {
            _destAddr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);
        }
        else {
            if (!_destAddr.sin_addr.s_addr)
            {
                Serial.println("\t\tError: no Server addr");
                while (true)
                {
                    Serial.println("Critical error");
                    delay(500);
                }
                return;
            }
            
        }
        Serial.println(_destAddr.sin_addr.s_addr);
        _destAddr.sin_family = AF_INET;
        _destAddr.sin_port = htons(_port);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        _sock = lwip_socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (_sock < 0)
        {
            // set critical
            Serial.println("\tsocket failed");
            err = ESP_FAIL;
            // return err;
        }
        Serial.println("\tsocket created");

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 1;         // blocking lwip_recvfrom time
        timeout.tv_usec = 0;      // 100ms (0.1s)
        lwip_setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        if (_sType == UFO_SOCK_SERVER)
        {
            int e = bind(_sock, (struct sockaddr *)&_destAddr, sizeof(_destAddr));
            if (e < 0)
            {
                Serial.print("\tsocket unable to bind:");
                Serial.println(e);
            }
            Serial.print("\tsocket bound, port:");
        }
        
        Serial.println(_port);
        // return err;
    }

    virtual void Iteration() final {
    if (xSemaphoreTake(_trsm->_lock, TickType_t(1000)) == pdTRUE)
    {
        // Serial.println("Sem");
        // debug
        if (_trsm->_ready)
        {
            // Serial.println("Sem->ready");
            // send
            int e = 0;
            if (_sType == UFO_SOCK_SERVER)
            {
                e = lwip_sendto(_sock, _trsm->_dcb._buff, _trsm->_dcb._len, 0, (struct sockaddr *)&_sourceAddr, _socklen);
            }
            else{
                e = lwip_sendto(_sock, _trsm->_dcb._buff, _trsm->_dcb._len, 0, (struct sockaddr *)&_destAddr, sizeof(_destAddr));
            }            

            // if (PC_debug) {
            // lwip_sendto(...,_pcAddr);
            // }

            if (e < 0)
            {
                Serial.println("Error occurred during sending");        //add counter
            }

            if (_trsm->_dcb._errorCount > 0)
            {
                --_trsm->_dcb._errorCount;
            }

            _trsm->_ready = false;
            _trsm->_dcb._len = 0;
            _trsm->_dcb._lastCallTick = millis(); // millis for a while
            // Serial.println("Sem->ready->quit");
        }
        else
        {
            if (millis() - _trsm->_dcb._lastCallTick > 200) // millis, 200 - for a while
            {
                ++_trsm->_dcb._errorCount;
            }
        }
        xSemaphoreGive(_trsm->_lock);
    } 
    else {
        //debug
        Serial.println("-It1notSem");
    }

    // Serial.println("rsv");

    int len = lwip_recvfrom(_sock, _rcv->_buff, UFO_SOCKET_BUFF_SIZE - 1, 0, (struct sockaddr *)&_sourceAddr, &_socklen);
    if (len > 0)
    {
    // Serial.println("rsv->funk");
        _recvFunck(_rcv);
        if (_rcv->_errorCount > 0)
        {
            --_rcv->_errorCount;
        }
    }
    else
    {
        uint32_t ms = millis();
        if (ms - _rcv->_lastCallTick > 500)
        {
            ++_rcv->_errorCount;
        }
        _rcv->_lastCallTick = ms;
    }
    if (_sourceAddr.ss_family == PF_INET)
    {
        inet_ntoa_r(((struct sockaddr_in *)&_sourceAddr)->sin_addr, _addrStr, sizeof(_addrStr) - 1);
    }

    if (_trsm->_dcb._errorCount > 10) // 10 for a while
    {
        // _alarm.doSmth
        // func*   onSendError(); 
        Serial.println("Alarm1!");
    }
    if (_rcv->_errorCount > 4)      //4 for a while
    {
        Serial.println("Alarm2!");
        // func*   onRecvError(); 
    }
}

    void Close()
{
    if (_sock != -1)
    {
        Serial.println("Shutting down socket");
        lwip_shutdown(_sock, 0);
        lwip_close(_sock);
    }
}

};


#pragma region

// static void udp_server_task(void *pvParameters)
// {
//     char rx_buffer[128];
//     char addr_str[128];
//     int addr_family = (int)pvParameters;
//     int ip_protocol = 0;
//     struct sockaddr_in6 dest_addr;

//     while (1) {

//         if (addr_family == AF_INET) {
//             struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
//             dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
//             dest_addr_ip4->sin_family = AF_INET;
//             dest_addr_ip4->sin_port = htons(PORT);
//             ip_protocol = IPPROTO_IP;
//         } else if (addr_family == AF_INET6) {
//             bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
//             dest_addr.sin6_family = AF_INET6;
//             dest_addr.sin6_port = htons(PORT);
//             ip_protocol = IPPROTO_IPV6;
//         }

//         int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//         if (sock < 0) {
//             ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
//             break;
//         }
//         ESP_LOGI(TAG, "Socket created");

//         // Set timeout
//         struct timeval timeout;
//         timeout.tv_sec = 10;
//         timeout.tv_usec = 0;
//         setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

//         int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//         if (err < 0) {
//             ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
//         }
//         ESP_LOGI(TAG, "Socket bound, port %d", PORT);

//         struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
//         socklen_t socklen = sizeof(source_addr);

// #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
//         struct iovec iov;
//         struct msghdr msg;
//         struct cmsghdr *cmsgtmp;
//         u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

//         iov.iov_base = rx_buffer;
//         iov.iov_len = sizeof(rx_buffer);
//         msg.msg_control = cmsg_buf;
//         msg.msg_controllen = sizeof(cmsg_buf);
//         msg.msg_flags = 0;
//         msg.msg_iov = &iov;
//         msg.msg_iovlen = 1;
//         msg.msg_name = (struct sockaddr *)&source_addr;
//         msg.msg_namelen = socklen;
// #endif

//         while (1) {
//             ESP_LOGI(TAG, "Waiting for data");
// #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
//             int len = recvmsg(sock, &msg, 0);
// #else
//             int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
// #endif
//             // Error occurred during receiving
//             if (len < 0) {
//                 ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
//                 break;
//             }
//             // Data received
//             else {
//                 // Get the sender's ip address as string
//                 if (source_addr.ss_family == PF_INET) {
//                     inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
// #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
//                     for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
//                         if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
//                             struct in_pktinfo *pktinfo;
//                             pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
//                             ESP_LOGI(TAG, "dest ip: %s", inet_ntoa(pktinfo->ipi_addr));
//                         }
//                     }
// #endif
//                 } else if (source_addr.ss_family == PF_INET6) {
//                     inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
//                 }

//                 rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
//                 ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
//                 ESP_LOGI(TAG, "%s", rx_buffer);

//                 int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
//                 if (err < 0) {
//                     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
//                     break;
//                 }
//             }
//         }

//         if (sock != -1) {
//             ESP_LOGE(TAG, "Shutting down socket and restarting...");
//             shutdown(sock, 0);
//             close(sock);
//         }
//     }
//     vTaskDelete(NULL);
// }

// void app_main(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

//     /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
//      * Read "Establishing Wi-Fi or Ethernet Connection" section in
//      * examples/protocols/README.md for more information about this function.
//      */
//     ESP_ERROR_CHECK(example_connect());

// #ifdef CONFIG_EXAMPLE_IPV4
//     xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
// #endif
// #ifdef CONFIG_EXAMPLE_IPV6
//     xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
// #endif

// }

#pragma endregion

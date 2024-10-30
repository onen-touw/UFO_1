#pragma once

#include "UFO_Config.h"
#include "lwip/sockets.h"
#include "UFO_TaskClassBase.h"
#include "UFO_StructDefinition.h"


class UFO_Socket : public UFO_TaskClassBase
{
private:
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
       _port = 6464;
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

    void SetTrsmCB(SockTxData_t* stdcb){
        _trsm = stdcb;
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

    virtual void Setup() final
    {
        esp_err_t err = ESP_OK;
        Serial.println("Socket setuping");
        Serial.println("\tChecking");

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
            Serial.println("\t\t\tpassed...");
            // while (true)
            // {
            //     Serial.println("Critical error");
            //     delay(500);
            // }
            // return;
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
        _destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
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
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        int e = bind(_sock, (struct sockaddr *)&_destAddr, sizeof(_destAddr));
        if (e < 0)
        {
            Serial.print("\tsocket unable to bind:");
            Serial.println(e);
        }
        Serial.print("\tsocket bound, port:");
        Serial.println(_port);
        // return err;
    }

#pragma region



// void RecievHandle(){
//      socklen_t socklen = sizeof(_sourceAddr);
//     int len = recvfrom(_sock, _rxBuff, sizeof(_rxBuff) - 1, 0, (struct sockaddr *)&_sourceAddr, &socklen);
//     // Error occurred during receiving
//     if (len < 0)
//     {
//         Serial.println("recvfrom failed");
//         return;
//     }

// // debug (only ipv4)
//     if (_sourceAddr.ss_family == PF_INET)
//     {
//         inet_ntoa_r(((struct sockaddr_in *)&_sourceAddr)->sin_addr, _addrStr, sizeof(_addrStr) - 1);
//     }

//     _rxBuff[len] = 0; // Null-terminate whatever we received and treat like a string...
//     Serial.println("recived info:");
//     Serial.print("\tfrom: ");
//     Serial.println(_addrStr);
//     Serial.print("\tinfo: ");
//     Serial.printf("%s\n", _rxBuff);

//     // _recvFunck(_rxBuff);
// }
// esp_err_t CreateClient(const char* addr, int32_t port){
//     esp_err_t err = ESP_OK;
//     Serial.println("Socket: Server creating");

//     if (_sock >= 0)
//     {
//         //setCritical
//         err = ESP_FAIL;
//         return err;
//     }

//     if (port < 0)
//     {
//         // set critical
//         err = ESP_FAIL;
//         return err;
//     }
//     _port = port;
    
//     _destAddr.sin_addr.s_addr = inet_addr(addr);
//     _destAddr.sin_family = AF_INET;
//     _destAddr.sin_port = htons(_port);
//     addr_family = AF_INET;
//     ip_protocol = IPPROTO_IP;

//     _sock = lwip_socket(addr_family, SOCK_DGRAM, ip_protocol);
//     if (_sock < 0)
//     {
//         // set critical
//         Serial.println("\tSocket failed");
//         err = ESP_FAIL;
//         return err;
//     }

//     // Set timeout
//     struct timeval timeout;
//     timeout.tv_sec = 10;
//     timeout.tv_usec = 0;
//     lwip_setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof (timeout));

//     Serial.println("\tsocket created");

// }
// esp_err_t CreateServer()
// {
//     esp_err_t err = ESP_OK;
//     Serial.println("Socket: Server creating");
//     if (_sock >= 0)
//     {
//         // setCritical
//         Serial.println("\tcreating fail");
//         err = ESP_FAIL;
//         return err;
//     }

//     if (_port < 0)
//     {
//         // set critical
//         Serial.println("\tcreating fail: incorrect port");
//         err = ESP_FAIL;
//         return err;
//     }
//     _destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
//     _destAddr.sin_family = AF_INET;
//     _destAddr.sin_port = htons(_port);
//     addr_family = AF_INET;
//     ip_protocol = IPPROTO_IP;

//     _sock = lwip_socket(addr_family, SOCK_DGRAM, ip_protocol);
//     if (_sock < 0)
//     {
//         // set critical
//         Serial.println("\tsocket failed");
//         err = ESP_FAIL;
//         return err;
//     }
//     Serial.println("\tsocket created");

//     // Set timeout
//     struct timeval timeout;
//     timeout.tv_sec = 10;
//     timeout.tv_usec = 0;
//     setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof (timeout));

//     int e = bind(_sock, (struct sockaddr *)&_destAddr, sizeof(_destAddr));
//     if (e < 0)
//     {
//         Serial.print("\tsocket unable to bind:");
//         Serial.println(e);
//     }
//     Serial.print("\tsocket bound, port:");
//     Serial.println(_port);
//     return err;
// }
// void SetPort(int32_t p){
//     _port = p;
// }
#pragma endregion

virtual void Iteration() final {
    if (xSemaphoreTake(_trsm->_lock, TickType_t(10000)) == pdTRUE)
    {
        // debug
        if (_trsm->_ready)
        {
            // send
            int e = lwip_sendto(_sock, _trsm->_dcb._buff, _trsm->_dcb._len, 0, (struct sockaddr *)&_sourceAddr, _socklen);
            if (e < 0)
            {
                Serial.println("Error occurred during sending");        //add counter
            }

            if (_noResponceCount > 0)
            {
                --_noResponceCount;
            }

            _trsm->_ready = false;
            _trsm->_dcb._len = 0;
            _trsm->_dcb._lastCallTick = millis(); // millis for a while
        }
        else
        {
            if (millis() - _trsm->_dcb._lastCallTick > 200) // millis, 200 - for a while
            {
                ++_noResponceCount;
            }
        }
        xSemaphoreGive(_trsm->_lock);
    } 
    else {
        //debug
        Serial.println("-It1notSem");
    }
    
    if (_noResponceCount > 10) // 10 for a while
    {
        // _alarm.doSmth
        Serial.println("Alarm!");
    }

    Serial.println("It1rcv");
    int len = lwip_recvfrom(_sock, _rcv->_buff, UFO_SOCKET_BUFF_SIZE - 1, 0, (struct sockaddr *)&_sourceAddr, &_socklen);     
    if (len < 0)
    {
        Serial.println("recvfrom failed");
        return;
    }
    if (_sourceAddr.ss_family == PF_INET)
    {
        inet_ntoa_r(((struct sockaddr_in *)&_sourceAddr)->sin_addr, _addrStr, sizeof(_addrStr) - 1);
    }
    Serial.printf("\tbuf: %s\n", _rcv->_buff);
    // _recvFunck(_rcv);
}

void It(){
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
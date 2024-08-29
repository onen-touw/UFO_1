#pragma once
#include "UFO_Config.h"
#include "esp_wifi.h"

static bool ___UFO_wf_constructed = false;              // only one UFO_wifi object can exist 
static EventGroupHandle_t _eGrp = nullptr;
static esp_netif_t *_netiff;

#define WIFI_CONNECTED_BIT  0b00001
#define WIFI_FAIL_BIT       0b00010


class UFO_WiFi_
{
private:
    void* pass = nullptr; // void* for a while 
    void* ssid = nullptr;// void* for a while 


    wifi_config_t _conf = {}; 
    wifi_mode_t _if =  wifi_mode_t::WIFI_MODE_AP;     //wifi interface::: null === auto (start like ap, if error -> sta) / sta === station / ap === access point 
    
    // uint16_t ___a;                                      // may be it will replace _eGrp;
    bool _driverStarted = false;
    uint8_t _reconC = 0;
    

public:
    UFO_WiFi_() {
    assert(!___UFO_wf_constructed);     

    
    // if(ssid != NULL && ssid[0] != 0){
    // 	if(password != NULL && password[0] != 0){
    // 		_conf.sta.threshold.authmode = min_security;
    // 	}
    //     if(bssid != NULL){
    //         _conf.sta.bssid_set = 1;
    //         memcpy(_conf.sta.bssid, bssid, 6);
    //     }
    // }

    // _conf.sta.ssid = "rngkong";
    //     wifi_config_t c = {
    //         .sta = {
    //             // .ssid = "ESP32wf",
    //             // .password = "12345678",
    //             // /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
    //             //  * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
    //             //  * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
    //             //  * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
    //             //  */
    //             // .threshold.authmode = WIFI_AUTH_OPEN,
    //             // .sae_pwe_h2e = ESP_WIFI_SAE_MODE,

    //             .ssid = "ESP32wf",          /**< SSID of target AP. */
    //             .password = "12345678",             /**< Password of target AP. */
    //             .scan_method = WIFI_ALL_CHANNEL_SCAN, /**< do all channel scan or fast scan */
    //             .bssid_set = 0,                 /**< whether set MAC address of target AP or not. Generally, station_config.bssid_set needs to be 0; and it needs to be 1 only when users need to check the MAC address of the AP.*/
    //             .bssid = {0},                   /**< MAC address of target AP*/
    //             .channel = 0,                   /**< channel of target AP. Set to 1~13 to scan starting from the specified channel before connecting to AP. If the channel of AP is unknown, set it to 0.*/
    //             .listen_interval = 0,           /**< Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set. Units: AP beacon intervals. Defaults to 3 if set to 0. */
    //             .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,   /**< sort the connect AP in the list by rssi or security mode */
    //             .threshold = {
    //                 .rssi = -127,
    //                 .authmode = WIFI_AUTH_WPA2_PSK,
    //             },                                          /**< When sort_method is set, only APs which have an auth mode that is more secure than the selected auth mode and a signal stronger than the minimum RSSI will be used. */
    //             .pmf_cfg = {
    //                 .capable = true,
    //                 .required = false,
    //             },      /**< Configuration for Protected Management Frame. Will be advertized in RSN Capabilities in RSN IE. */
    //         }
    //     };

        ___UFO_wf_constructed = true;
    }
    ~UFO_WiFi_() {
        ___UFO_wf_constructed = false;
    }


    esp_err_t Init(){
        esp_err_t err = ESP_OK;
        // if (_if == null) { _if = sta .. "try connect" .. _if = ap .. "start ap"}
        esp_ip4_addr_t* __add;
        if (_if != WIFI_MODE_NULL)
        {
            switch (_if)
            {
            case WIFI_MODE_AP:
                __InitAP();
                err = __DriverInit();
                if (err)
                {
                    Serial.println("ERROR:: __DriverInit");
                    return err;
                }
                
                wifi_mode_t mode;
                if (esp_wifi_get_mode(&mode) != ESP_OK)
                {
                    Serial.println("WiFi not started");
                    return ESP_FAIL;
                }

                esp_netif_ip_info_t ip;
                if (esp_netif_get_ip_info(_netiff, &ip) != ESP_OK)
                {
                    Serial.println("ERROR:: esp_netif_get_ip_info");
                    return ESP_FAIL;
                }
                Serial.printf("IP::");
                __add= &ip.ip;
                Serial.printf(IPSTR, IP2STR(__add));

                break;
            case WIFI_MODE_STA:
                __InitSTA();
                break;
            default:
                err = ESP_FAIL;
                return err;
            }
            return err;
        }

        _if = WIFI_MODE_STA; 
        __InitSTA();
        err = __DriverInit();
        uint8_t tryes = 0;
        while (tryes < 10 || !_driverStarted)
        {
            //try to connect to ap 
            // delay 20
        }

        // if (success)
        //  {
        // return err
        //} 

        //  else :
        err = __DriverDeinit();
         _if = WIFI_MODE_AP; 
        __InitAP();
        err = __DriverInit();
        return err;        
    }



private:

    void __InitSTA(){
        _conf.sta.channel = 0;
        _conf.sta.listen_interval = 0;
        _conf.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;//WIFI_ALL_CHANNEL_SCAN or WIFI_FAST_SCAN
        _conf.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;//WIFI_CONNECT_AP_BY_SIGNAL or WIFI_CONNECT_AP_BY_SECURITY
        _conf.sta.threshold.rssi = -127;
        _conf.sta.pmf_cfg.capable = true;
        _conf.sta.pmf_cfg.required = false;
        _conf.sta.bssid_set = 0;
        memset(_conf.sta.bssid, 0, 6);
        _conf.sta.threshold.authmode = WIFI_AUTH_OPEN;
        _conf.sta.ssid[0] = 0;
        _conf.sta.password[0] = 0;
        memcpy(_conf.sta.ssid, "ESP32wf", 8);
        memcpy(_conf.sta.password, "12345678", 9);
    }

    void __InitAP(){
        _conf.ap.channel = 6;
        _conf.ap.max_connection = 4;            //set to 1
        _conf.ap.beacon_interval = 100;
        // _conf.ap.ssid_hidden = 0;
        _conf.ap.authmode = WIFI_AUTH_WPA2_PSK;
        _conf.ap.ssid_len = 12;
        _conf.ap.ssid[0] = 0;
        _conf.ap.password[0] = 0;
        _conf.ap.ftm_responder = false;
        _conf.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP; // Disable by default enabled insecure TKIP and use just CCMP.
        memcpy(_conf.ap.ssid, "esp32323232", 12);
        memcpy(_conf.ap.password, "12345678", 9);

        // if (ssid != NULL && ssid[0] != 0)
        // {
        //     wifi_config->ap.ssid_len = strlen(ssid);
        //     if (password != NULL && password[0] != 0)
        //     {
        //         wifi_config->ap.authmode = authmode;
        //         _wifi_strncpy((char *)wifi_config->ap.password, password, 64);
        //     }
        // }
    }

    esp_err_t __DriverInit(){
        Serial.println("__DriverInit");
        esp_err_t err = ESP_OK;
        
        _eGrp = xEventGroupCreate();
        if (!_eGrp)
        {
            Serial.println("ERROR:: xEventGroupCreate");
            return ESP_FAIL;
        }

        err = esp_netif_init(); 
        if (err != ESP_OK)
        {
            // set critical
            Serial.println("ERROR:: esp_netif_init");
            return err;
        }
        
        // esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
        // _netiff = esp_netif_new(&cfg);
        // if (!_netiff)
        // {
        //     err = ESP_FAIL;
        //     Serial.println("ERROR:: esp_netif_new");
        //     return;
        // }
        // err = esp_netif_attach_wifi_station(_netiff);
        // err = esp_wifi_set_default_wifi_sta_handlers();


        err = esp_event_loop_create_default();
        if (err != ESP_OK)
        {
            Serial.println("ERROR:: esp_event_loop_create_default");
            // set critical
            return err;
        }
        if (_if != wifi_mode_t::WIFI_MODE_AP)
        {
            _netiff = esp_netif_create_default_wifi_sta();
        }
        else
        {
            _netiff = esp_netif_create_default_wifi_ap();
        }
          if (!_netiff)
        {
            err = ESP_FAIL;
            Serial.println("ERROR:: esp_netif_create_default_wifi");
            return err;
        }
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK)
        {
            Serial.println("ERROR:: esp_wifi_init");
            // set critical
            return err;
        }
        err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, __EventHandler, NULL, NULL);
        if (err != ESP_OK)
        {
            Serial.println("ERROR:: esp_event_handler_instance_register");
            // set critical
            return err;
        }

        if (_if != wifi_mode_t::WIFI_MODE_AP)
        {
            err = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, __EventHandler, NULL, NULL);
            if (err != ESP_OK)
            {
                Serial.println("ERROR:: esp_event_handler_instance_register");
                // set critical
                return err;
            }
        }

        err = esp_wifi_set_mode(_if); 
        if (err != ESP_OK)
        {
                Serial.println("ERROR:: esp_wifi_set_mode");
                return err;
        }

        wifi_interface_t if_f = wifi_interface_t::WIFI_IF_AP;
        if (_if != WiFiMode_t::WIFI_MODE_AP)
        {
            if_f = wifi_interface_t::WIFI_IF_STA;
        }

        err = esp_wifi_set_config(if_f, &_conf);
                if (err != ESP_OK)
        {
                Serial.println("ERROR:: esp_wifi_set_config");
                return err;
        }

        err = esp_wifi_start(); 
        if (err != ESP_OK)
        {
                Serial.println("ERROR:: esp_wifi_start");
                return err;
        }

        if (_if != wifi_mode_t::WIFI_MODE_AP)
        {

            /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
             * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
            EventBits_t bits = xEventGroupWaitBits(_eGrp,
                                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                   pdFALSE,
                                                   pdFALSE,
                                                   portMAX_DELAY);

            /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
             * happened. */
            if (bits & WIFI_FAIL_BIT)
            {
                // set reconnect error
                err = ESP_FAIL;
                return err;
            }
        }
        _driverStarted = true;
        Serial.println("\tSuccess");
        return err;
    }


    esp_err_t __DriverDeinit(){
        esp_err_t err = ESP_OK;
        err = esp_wifi_stop();
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, __EventHandler);
        esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, __EventHandler);
        esp_netif_destroy_default_wifi(((void*)_netiff));
        esp_wifi_deinit();
        esp_event_loop_delete_default();
        esp_netif_deinit();
        vEventGroupDelete(_eGrp);
        _driverStarted = false;
        return err;
    }

    static void __EventHandler(void* arg, esp_event_base_t base, int32_t id, void* edat){
        static uint8_t reconC = 0;
        if (base == WIFI_EVENT)
        {
            if (id == WIFI_EVENT_STA_DISCONNECTED)
            {
                esp_wifi_connect();
                ++reconC;
                Serial.print("Disconnected\t ReconC= ");
                Serial.println(reconC);
            }
            else if (id== WIFI_EVENT_STA_CONNECTED)
            { 
                xEventGroupSetBits(_eGrp, WIFI_FAIL_BIT);
                Serial.print("Connected");
                Serial.println(reconC);
            }
            else if  (id == WIFI_EVENT_AP_STACONNECTED)
            {
                wifi_event_ap_staconnected_t *event = reinterpret_cast<wifi_event_ap_staconnected_t *>(edat);
                Serial.printf("station " MACSTR " join, AID=%d\n", MAC2STR(event->mac), event->aid);
            }
            else if (id == WIFI_EVENT_AP_STADISCONNECTED)
            {
                wifi_event_ap_stadisconnected_t *event = reinterpret_cast<wifi_event_ap_stadisconnected_t *>(edat);
                Serial.printf("station " MACSTR " leave, AID=%d\n", MAC2STR(event->mac), event->aid);
            }
        }
        else if(base == IP_EVENT){
            if (id ==  IP_EVENT_STA_GOT_IP)
            {
                xEventGroupSetBits(_eGrp, WIFI_CONNECTED_BIT);
                reconC = 0;
                ip_event_got_ip_t* event = reinterpret_cast<ip_event_got_ip_t*>(edat);
                Serial.print("if_if\t");
                Serial.println(event->if_index);
                Serial.print("ip_address\t");
                Serial.println(event->ip_info.ip.addr);
                Serial.print("ip_gw\t");
                Serial.println(event->ip_info.gw.addr);
                Serial.print("ip_netmask\t");
                Serial.println(event->ip_info.netmask.addr);
            }
        }
    }
};




//  uint8_t mac[8];
//         if(esp_efuse_mac_get_default(mac) == ESP_OK){
//             esp_base_mac_addr_set(mac);
//         }
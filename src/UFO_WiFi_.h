#pragma once
#include "UFO_Config.h"
#include "esp_wifi.h"

static bool ___UFO_wf_constructed = false;              // only one UFO_wifi object can exist 
static EventGroupHandle_t _eGrp = nullptr;
static esp_netif_t *_netiff;

#define WIFI_CONNECTED_BIT  0b00001
#define WIFI_FAIL_BIT       0b00010
#define UFO_WIFI_DEF_BASE_IP "192.168.4.2"
#define UFO_WIFI_DEF_NETMASK "255.255.255.0"
#define UFO_WIFI_DEF_GW_ADDR "192.168.4.1"
#define UFO_WIFI_DEF_MAINDNS_SERVER UFO_WIFI_DEF_GW_ADDR
#define UFO_WIFI_DEF_BCP_DNS_SERVER UFO_WIFI_DEF_GW_ADDR

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
        ___UFO_wf_constructed = true;
    }

    ~UFO_WiFi_() {
        ___UFO_wf_constructed = false;
    }


    esp_err_t Init(){
        esp_err_t err = ESP_OK;
        // if (_if == null) { _if = sta .. "try connect" .. _if = ap .. "start ap"}
        esp_ip4_addr_t* __add;
        wifi_mode_t mode;
        esp_netif_ip_info_t ip;

        if (_if != WIFI_MODE_NULL)
        {
            switch (_if)
            {
            case WIFI_MODE_AP:
                Serial.println("Starting wifi in Ap mode");
                
                Serial.println("\tInitializing AP...");
                __InitAP();

                Serial.println("\tInitializing wifi driver...");
                err = __DriverInit();
                if (err)
                {
                    Serial.println("ERROR:: __DriverInit");
                    return err;
                }
                
                if (esp_wifi_get_mode(&mode) != ESP_OK)
                {
                    Serial.println("WiFi not started");
                    return ESP_FAIL;
                }

                if (esp_netif_get_ip_info(_netiff, &ip) != ESP_OK)
                {
                    Serial.println("ERROR:: esp_netif_get_ip_info");
                    return ESP_FAIL;
                }

                Serial.println("\tSuccses!!!");
                Serial.printf("IP::");
                __add= &ip.ip;
                Serial.printf(IPSTR, IP2STR(__add));
                Serial.println();

                break;
            case WIFI_MODE_STA:
                Serial.println("Starting wifi in STA mode:");

                Serial.println("\tInitializing STA...");
                __InitSTA();

                Serial.println("\tInitializing wifi driver...");
                err = __DriverInit();
                if (err)
                {
                    Serial.println("ERROR:: __DriverInit");
                    return err;
                }

                if (esp_wifi_get_mode(&mode) != ESP_OK)
                {
                    Serial.println("WiFi not started");
                    return ESP_FAIL;
                }

                if (esp_netif_get_ip_info(_netiff, &ip) != ESP_OK)
                {
                    Serial.println("ERROR:: esp_netif_get_ip_info");
                    return ESP_FAIL;
                }
                Serial.println("\tSuccses!!!");
                Serial.printf("IP::");
                __add= &ip.ip;
                Serial.printf(IPSTR, IP2STR(__add));
                Serial.println();
                break;

            default:
                err = ESP_FAIL;
                return err;
            }

            Serial.print("Initing err code:\t");
            Serial.println(err);
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

    esp_err_t SetApIP(const char* ipstr){
        esp_err_t err = ESP_OK;
        esp_netif_dhcp_status_t status = ESP_NETIF_DHCP_INIT;
	    esp_netif_ip_info_t info;

        if(!strlen(ipstr)){
            info.ip.addr = ipaddr_addr(UFO_WIFI_DEF_BASE_IP);
            info.gw.addr = ipaddr_addr(UFO_WIFI_DEF_GW_ADDR);
        }
        else {
            info.ip.addr = ipaddr_addr(ipstr);
            info.gw.addr = ipaddr_addr(ipstr);
        }


        info.netmask.addr = ipaddr_addr(UFO_WIFI_DEF_NETMASK);


        err = esp_netif_dhcpc_get_status(_netiff, &status);
        if(err){
        	log_e("DHCPC Get Status Failed! 0x%04x", err);
        	return err;
        }
		err = esp_netif_dhcpc_stop(_netiff);
		if(err && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED){
			log_e("DHCPC Stop Failed! 0x%04x", err);
			return err;
		}
        err = esp_netif_set_ip_info(_netiff, &info);
        if(err){
        	log_e("Netif Set IP Failed! 0x%04x", err);
        	return err;
        }
    	if(info.ip.addr == 0){
    		err = esp_netif_dhcpc_start(_netiff);
    		if(err){
            	Serial.printf("DHCPC Start Failed! 0x%04x", err);
            	return err;
            }
    	}
        return err;
    }

private:

    void __InitSTA(){
        const char *ss = "KULON";
        const char *ps = "KULON";
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
        memcpy(_conf.sta.ssid, ss, strlen(ss));
        memcpy(_conf.sta.password, ps, strlen(ps));;
    }

    void __InitAP(){
        const char* ss = "ESP_UFO32";
        const char* ps = "12345678";
        _conf.ap.channel = 0;
        _conf.ap.max_connection = 1;            //set to 1
        _conf.ap.beacon_interval = 100;
        _conf.ap.ssid_hidden = 0;
        _conf.ap.authmode = WIFI_AUTH_WPA2_PSK;
        _conf.ap.ssid_len = strlen(ss);
        _conf.ap.ssid[0] = 0;
        _conf.ap.password[0] = 0;
        _conf.ap.ftm_responder = false;
        _conf.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP; // Disable by default enabled insecure TKIP and use just CCMP.
        memcpy(_conf.ap.ssid, ss, strlen(ss));
        memcpy(_conf.ap.password,  ps, strlen(ps));

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
            err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, __EventHandler, NULL, NULL);
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
        if (_if != wifi_mode_t::WIFI_MODE_AP)
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
        Serial.println("\twaiting bits...");
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
        Serial.println("\t\tSuccess");
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
            else if (id == WIFI_EVENT_STA_START)
            { 
                esp_wifi_connect();
            }
            else if (id == WIFI_EVENT_STA_CONNECTED)
            { 
                // xEventGroupSetBits(_eGrp, WIFI_CONNECTED_BIT);
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
                // Serial.print("if_if\t");
                // Serial.println(event->if_index);
                // Serial.print("ip_address\t");
                // Serial.println(event->ip_info.ip.addr);
                // Serial.print("ip_gw\t");
                // Serial.println(event->ip_info.gw.addr);
                // Serial.print("ip_netmask\t");
                // Serial.println(event->ip_info.netmask.addr);
            }
        }
    }
};




//  uint8_t mac[8];
//         if(esp_efuse_mac_get_default(mac) == ESP_OK){
//             esp_base_mac_addr_set(mac);
//         }
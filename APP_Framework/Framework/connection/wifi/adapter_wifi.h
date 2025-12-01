#ifndef ADAPTER_WIFI_H
#define ADAPTER_WIFI_H

#define CONFIG_WIFI_RESET            (0)
#define CONFIG_WIFI_RESTORE          (1)
#define CONFIG_WIFI_BAUDRATE         (2)


#define SOCKET_PROTOCOL_TCP  (6)
#define SOCKET_PROTOCOL_UDP  (17)

struct WifiParam
{
    uint8_t wifi_ssid[128];
    uint8_t wifi_pwd[128];
};


#endif
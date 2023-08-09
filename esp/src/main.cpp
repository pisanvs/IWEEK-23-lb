#ifdef ESP8266
extern "C" {
#include "ets_sys.h"
#include "gpio.h"
#include "os_type.h"
#include "osapi.h"
#include "user_config.h"
#include "user_interface.h"
}
#endif

#include <Arduino.h>

int ap_channel = 1;

#define ID 2

// I have no clue how any of this works, but i assume its some sort of enum and
// pointer hackery
struct RxControl {
    signed rssi : 8;
    unsigned rate : 4;
    unsigned is_group : 1;
    unsigned : 1;
    unsigned sig_mode : 2;
    unsigned legacy_length : 12;
    unsigned damatch0 : 1;
    unsigned damatch1 : 1;
    unsigned bssidmatch0 : 1;
    unsigned bssidmatch1 : 1;
    unsigned MCS : 7;
    unsigned CWB : 1;
    unsigned HT_length : 16;
    unsigned Smoothing : 1;
    unsigned Not_Sounding : 1;
    unsigned : 1;
    unsigned Aggregation : 1;
    unsigned STBC : 2;
    unsigned FEC_CODING : 1;
    unsigned SGI : 1;
    unsigned rxend_state : 8;
    unsigned ampdu_cnt : 8;
    unsigned channel : 4;
    unsigned : 12;
};

struct LenSeq {
    uint16_t length;
    uint16_t seq;
    uint8_t address3[6];
};

struct sniffer_buf {
    struct RxControl rx_ctrl;
    uint8_t buf[36];
    uint16_t cnt;
    struct LenSeq lenseq[1];
};

struct sniffer_buf2 {
    struct RxControl rx_ctrl;
    uint8_t buf[112];
    uint16_t cnt;
    uint16_t len;
};
unsigned long time_ = 0;

const char *whitelist[] = {
    "001122334455", "de639120c2a5", "a864f13cda25", "405ef649cd07"};  // Whitelisted device MAC addresses
size_t numWhitelistedDevices = sizeof(whitelist) / sizeof(whitelist[0]);

bool isMacInWhitelist(String mac) {
    for (size_t i = 0; i < numWhitelistedDevices; i++) {
        if (mac.equalsIgnoreCase(whitelist[i])) {
            return true;
        }
    }
    return false;
}

// Callback function
void ICACHE_FLASH_ATTR promisc_cb(uint8 *buf, uint16 len) {
    int i = 0;
    if (len == 12) return;
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2 *)buf;
    String macAddress = "";
    for (i = 0; i < 6; i++) {
        macAddress += String(sniffer->buf[i + 10], HEX);
    }
    int rssi = sniffer->rx_ctrl.rssi;

    // Check if the MAC address is in the whitelist
    if (isMacInWhitelist(macAddress)) {
        Serial.printf("%d, %s, RSSI: %2d\n", ID, macAddress.c_str(), rssi);
    }
}

void setup() {
    Serial.begin(115200);
    wifi_set_opmode(0x1);
    wifi_set_channel(ap_channel);
    wifi_promiscuous_enable(0);
    wifi_set_promiscuous_rx_cb(promisc_cb);
    wifi_promiscuous_enable(1);
}

void loop() {
    if (Serial.available()) {
        Serial.write(Serial.read());
    }
}
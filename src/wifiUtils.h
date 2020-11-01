#include "secrets/secrets.h"

// Wifi & NTP

#include <WiFi.h>
#include <NTPClient.h>

// Blynk

#include <WiFiClient.h>

bool iniNTP();
static void InitWifi();
static bool isWifiOK();

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


bool iniNTP(){

    timeClient.begin();
    // To machine advisor, should be sent UCT+0 to be able to sync different machines world wide.
    timeClient.setUpdateInterval(1000*60*100); // update time every 100 minutes

    bool actualitzada;
    while(true) {
        actualitzada = timeClient.update();
        if (!actualitzada) {
            timeClient.forceUpdate();
            Serial.println("t");
        } else break;
    }
    Serial.println("Connected to NTP. Time = " + timeClient.getFormattedTime());
    
    return(actualitzada);
}


static void InitWifi() {
    Serial.println(" > WiFi");
    Serial.println("Connecting...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP().toString());
}

static bool isWifiOK(){
    return (WiFi.status() == WL_CONNECTED);
}
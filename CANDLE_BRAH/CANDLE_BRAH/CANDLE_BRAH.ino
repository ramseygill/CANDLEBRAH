/*==============================================================================
    
   Title:CANDLEBRAH
   
   Description:Arduino based art robot that lights and extinguishes a candle 
   on command. (voice, clap, button) Allows your robot overlord to wield fire. 
   What could possibly go wrong with an IoT connected source 
   of pyrotechnic ignition?
   
   Author:Ramsey Gill
   
   Website:https://hackaday.io/project/161041-candle-brah
   
   Git Repo:https://github.com/ramseygill/CANDLEBRAH

    MIT License
    
    Copyright (c) 2018 Ramsey Gill
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
  
 */===============================================================================


#include <Arduino.h>
#ifdef ESP32
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"
#include "credentials.h"

// NODE MCU ESP-12E built in LED's on GPIO 2 and 16
#define SERIAL_BAUDRATE                 115200
#define LED                             2
#define BUTTON                          
#define SRV_SHOULDER                    4
#define SRV_ELBOW                       5
#define SRV_VALVE                       14
#define FLAME_SENSOR                    12                          

fauxmoESP fauxmo;

// -----------------------------------------------------------------------------
// Wifi Setup Function
// -----------------------------------------------------------------------------

void wifiSetup() {

    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

    // Connect
    Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Connected!
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------

void setup() {

    // Init serial port and clean garbage
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println();
    Serial.println();

    // Wifi
    wifiSetup();

    // LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    // You can enable or disable the library at any moment
    // Disabling it will prevent the devices from being discovered and switched
    fauxmo.enable(true);

    // Add virtual devices
    fauxmo.addDevice("switch one");
	//fauxmo.addDevice("switch two"); // You can add more devices
	//fauxmo.addDevice("switch three");

    // fauxmoESP 2.0.0 has changed the callback signature to add the device_id,
    // this way it's easier to match devices to action without having to compare strings.
    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state) {
        Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
        digitalWrite(LED, !state);
    });

    // Callback to retrieve current state (for GetBinaryState queries)
    fauxmo.onGetState([](unsigned char device_id, const char * device_name) {
        return !digitalRead(LED);
    });

}

// -----------------------------------------------------------------------------
// MAIN LOOP
// -----------------------------------------------------------------------------

void loop() {

    // Since fauxmoESP 2.0 the library uses the "compatibility" mode by
    // default, this means that it uses WiFiUdp class instead of AsyncUDP.
    // The later requires the Arduino Core for ESP8266 staging version
    // whilst the former works fine with current stable 2.3.0 version.
    // But, since it's not "async" anymore we have to manually poll for UDP
    // packets
    fauxmo.handle();

    static unsigned long last = millis();
    if (millis() - last > 5000) {
        last = millis();
        Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
    }

}
// -----------------------------------------------------------------------------
// FUNCTIONS
// -----------------------------------------------------------------------------

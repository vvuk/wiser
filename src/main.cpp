/*********************************************************************************
 * ESP-Serial-Bridge
 * 
 * Simple WiFi Serial Bridge for Espressif microcontrollers
 * 
 * Forked from https://github.com/yuri-rage/ESP-Serial-Bridge
 * Forked from https://github.com/AlphaLima/ESP32-Serial-Bridge
 * 
 * Added compatibility for ESP8266, WiFi reconnect on failure, and mDNS discovery.
 * 
 * Note: ESP8266 is limited to 115200 baud and may be somewhat unreliable in
 *       this application.
 * 
 *   -- Yuri - Aug 2021
 * 
 * Disclaimer: Don't use for life support systems or any other situation
 * where system failure may affect user or environmental safety.
*********************************************************************************/

#ifdef ESP32
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SoftwareSerial.h>  // use SoftwareSerial for ESP8266
#endif
#include <WiFiClient.h>

#include "config.h"

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 
#endif

#ifdef ESP32
#define WhateverSerial HardwareSerial
#define WhateverSerialConfig HardwareSerialConfig
#else
#define WhateverSerial SoftwareSerial
#define WhateverSerialConfig SoftwareSerialConfig
#endif

#define MAX_CLIENTS_PER_PORT 1

struct COM_PORT {
  WhateverSerial serial;
  WiFiServer server;
  WiFiClient clients[MAX_CLIENTS_PER_PORT];

  uint32_t baud;
  WhateverSerialConfig config;

  int rxPin;
  int txPin;

  void beginSerial() {
    serial.begin(baud, config, rxPin, txPin);
  }

  void begin() {
    beginSerial();
    server.begin();
    server.setNoDelay(true);
  }
};

COM_PORT COM[] = {
  { {}, SERIAL0_TCP_PORT, {}, SERIAL0_BAUD,
    WhateverSerialConfig(SERIAL0_PARAM), D3, D4 }, //SERIAL0_TXPIN, SERIAL0_RXPIN },
  //{ {}, SERIAL1_TCP_PORT, {}, SERIAL1_BAUD,
  //  WhateverSerialConfig(SERIAL1_PARAM), SERIAL1_TXPIN, SERIAL1_RXPIN },
};

WiFiServer controlServer(CONTROL_PORT);
WiFiClient controlClient;
char clientCmdBuf[BUFFER_SIZE + 1];
int clientCmdBufPos = 0;

char sBuffer[BUFFER_SIZE];

#define DPRINTF(...) if (debug) Serial.printf(__VA_ARGS__)

#ifdef MODE_STA
void reconnectWiFi()
{
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    DPRINTF(".");
  }

  DPRINTF("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
}

#ifdef ESP32
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
#elif defined(ESP8266)
WiFiEventHandler stationDisconnectedHandler;
void WiFiStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
#endif
  DPRINTF("WiFi disconnected: %s, trying to reconnect...\n",
#ifdef ESP32
    info.disconnected.reason
#else
    "(unknown)"
#endif
  );

  reconnectWiFi();
}
#endif

#define NUM_COMS int(sizeof(COM) / sizeof(COM[0]))

void setup() {

  delay(500);
  Serial.begin(38400);

  delay(500);

  DPRINTF("\n\nWiFi serial bridge %s ", VERSION);
  
  #ifdef MODE_AP 
  DPRINTF("Open ESP Access Point mode");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP
  #endif

  #ifdef MODE_STA
  DPRINTF("Open ESP Station Mode\n");
  WiFi.mode(WIFI_STA);
  #ifdef ESP32
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  #elif defined(ESP8266)
  stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&WiFiStationDisconnected);
  #endif

  reconnectWiFi();

  if (MDNS.begin(mdns_name)) {
    DPRINTF("Started mDNS, discoverable as: %s\n", mdns_name);
    for (int i = 0; i < NUM_COMS; i++) {
      MDNS.addService("telnet", "tcp", COM[i].server.port());
    }
  } else {
     DPRINTF("Error starting mDNS");
  }
  #endif

#ifdef OTA_HANDLER  
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif   

  controlServer.begin();

  for (int num = 0; num < NUM_COMS; num++) {
    if (debug) {
      Serial.print("Starting serial comms & TCP Server ");
      Serial.println(num + 1);
    }
    COM[num].begin();
  }

#ifdef BATTERY_SAVER
#ifdef ESP32
  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
#elif defined(ESP8266)
  WiFi.setOutputPower(15);
#endif
#endif
}

void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif

  // Handle control port first
  if (controlServer.hasClient()) {
    if (controlClient.connected()) {
      controlClient.stop();
    }
    controlClient = controlServer.available();
  }

  if (controlClient.connected()) {
    while (int av = controlClient.available()) {
      if (av + clientCmdBufPos < BUFFER_SIZE) {
        controlClient.readBytes(clientCmdBuf + clientCmdBufPos, av);
        clientCmdBufPos += av;
      } else {
        controlClient.write("READ BUFFER OVERFLOW\n");
        clientCmdBufPos = 0;
      }

      clientCmdBuf[clientCmdBufPos] = 0;
      while (char *nl = strchr(clientCmdBuf, '\n')) {
        *nl = 0;

        // handle command in clientCmdBuf
        if (strstr(clientCmdBuf, "baud ") == clientCmdBuf) {
          int port = atoi(clientCmdBuf + 5) - 1;
          int baud = atoi(clientCmdBuf + 7);

          if (port < 0 || port >= NUM_COMS) {
            controlClient.write("Invalid port\n");
          } else if (baud < 0 || baud > 115200) {
            controlClient.write("Invalid baud rate\n");
          } else {
            COM[port].baud = baud;
            COM[port].beginSerial();
            controlClient.write("OK\n");
          }
        } else if (strcmp(clientCmdBuf, "status") == 0) {
          for (int i = 0; NUM_COMS > i; i++) {
            controlClient.printf("Port %d: %d baud\n", i + 1, COM[i].baud);
          }
        } else {
          controlClient.write("Commands:\n");
          controlClient.write("baud <port> <baud>\n");
          controlClient.write("status\n");
          controlClient.write("Unknown command\n");
        }

        // next command
        memcpy(clientCmdBuf, nl + 1, BUFFER_SIZE - (nl - clientCmdBuf));
        clientCmdBufPos -= (nl - clientCmdBuf + 1);
        clientCmdBuf[clientCmdBufPos] = 0;
      }
    }
  }

  // Check for new connections and accept them
  for (int num = 0; num < NUM_COMS; num++)
  {
    auto& com = COM[num];
    if (com.server.hasClient()) {
      int dc = 0;
      for (int i = 0; i < MAX_CLIENTS_PER_PORT; i++) {
        if (!com.clients[i].connected()) {
          dc = i;
          break;
        }
      }

      dc = dc % MAX_CLIENTS_PER_PORT;
      if (com.clients[dc].connected()) {
        com.clients[dc].stop();
      }

      com.clients[dc] = com.server.available();
      DPRINTF("Client for port %d connected: %s\n", num+1, com.clients[dc].remoteIP().toString().c_str());
    }
  }
 
  for (int num = 0; num < NUM_COMS; num++)
  {
    auto& com = COM[num];

    // handle incoming from TCP, and send to UART
    int bufpos = 0;

    // TODO -- should broadcast to other TCP connections, otherwise
    // things will look weird?
    for (int i = 0; i < MAX_CLIENTS_PER_PORT; i++) {
      if (!com.clients[i].connected())
        continue;

      bufpos = 0;
      // read from client and write to serial port
      while (com.clients[i].available()) {
        sBuffer[bufpos++] = com.clients[i].read();
        if (bufpos == BUFFER_SIZE) {
          com.serial.write(sBuffer, bufpos);
          bufpos = 0;
        }
      }
      if (bufpos > 0) {
        com.serial.write(sBuffer, bufpos);
      }
    }

    // read from serial port and write to all clients
    bufpos = 0;
    while (com.serial.available()) {
      sBuffer[bufpos++] = com.serial.read();
      if (bufpos == BUFFER_SIZE) {
        for (int i = 0; i < MAX_CLIENTS_PER_PORT; i++) {
          if (com.clients[i].connected()) {
            com.clients[i].write(sBuffer, bufpos);
          }
        }
      }
    }
    if (bufpos > 0) {
      for (int i = 0; i < MAX_CLIENTS_PER_PORT; i++) {
        if (com.clients[i].connected()) {
          com.clients[i].write(sBuffer, bufpos);
        }
      }
    }
  }
}
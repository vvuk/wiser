/*********************************************************************************
 * ESP-Serial-Bridge
 * 
 * Simple WiFi Serial Bridge for Espressif microcontrollers
 * 
 * Forked from https://github.com/yuri-rage/ESP-Serial-Bridge
 * Forked from https://github.com/AlphaLima/ESP32-Serial-Bridge
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
#include <SoftwareSerial.h>
#endif
#include <WiFiClient.h>

#include "config.h"

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 
#endif

// Control-Q
#define ESCAPE_CHAR 0x11
#define MAX_CLIENTS_PER_PORT 2

struct COM_PORT {
  typedef void (*ConfigureSerialFn)(uint32_t baud, SerialConfig cfg);

  //HardwareSerial serial;
  Stream& serial;
  ConfigureSerialFn serialConfigure;

  WiFiServer server;
  WiFiClient clients[MAX_CLIENTS_PER_PORT];

  uint32_t baud;
  SerialConfig config;

  int enablePin; // -1 if not used, otherwise pulls pin high on start

  COM_PORT(Stream& srm, ConfigureSerialFn cfg,
    int serverPort, uint32_t brate = 9600, int en = -1, SerialConfig scfg = SERIAL_8N1)
    : serial(srm), serialConfigure(cfg)
    , server(serverPort), baud(brate), enablePin(en), config(scfg)
  {
    if (en != -1) {
      pinMode(en, OUTPUT);
      digitalWrite(en, LOW);
    }
  }

  void beginSerial() {
    if (enablePin != -1)
      digitalWrite(enablePin, HIGH);
    serialConfigure(baud, config);
  }

  void begin() {
    beginSerial();
    server.setNoDelay(true);
    server.begin();
  }
};

//SoftwareSerial serial1(D4, D3);
HardwareSerial serial1(1);

COM_PORT COM[] = {
  {
    //serial1, [](uint32_t baud, SerialConfig cfg) { serial1.begin(baud, (SoftwareSerialConfig)cfg, -1, -1); },
    serial1, [](uint32_t baud, SerialConfig cfg) { serial1.begin(baud, cfg); },
    SERIAL0_TCP_PORT, 9600, D8, SERIAL_8N1
  },
};

#define NUM_COMS int(sizeof(COM) / sizeof(COM[0]))

WiFiServer controlServer(CONTROL_PORT);
WiFiClient controlClient;
int controlCom = -1; // if the controlClient should be connected to a com port
char clientCmdBuf[BUFFER_SIZE + 1];
int clientCmdBufPos = 0;

char sBuffer[BUFFER_SIZE];

#define DPRINTF(...) if (debug && controlClient.connected()) controlClient.printf(__VA_ARGS__)

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

void setup()
{
  WiFiClient::setDefaultNoDelay(true);
  WiFiClient::setDefaultSync(true);

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
      DPRINTF("OTA Start updating %s\n", type);
    });
    ArduinoOTA.onEnd([]() {
      DPRINTF("OTA End\n"); 
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     DPRINTF("OTA Progress: %u%%\r", (progress / (total / 100))); 
    });
    ArduinoOTA.onError([](ota_error_t error) {
      DPRINTF("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) DPRINTF("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) DPRINTF("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) DPRINTF("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) DPRINTF("Receive Failed");
      else if (error == OTA_END_ERROR) DPRINTF("End Failed");
      DPRINTF("\n");
    });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif   

  controlServer.begin();

  for (int num = 0; num < NUM_COMS; num++) {
    DPRINTF("Starting serial comms & TCP Server %d\n", num + 1);
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
    // always disconnect from a com port
    controlCom = -1;
  }

  if (controlClient.connected() && controlCom == -1) {
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

        DPRINTF("Command: '%s'\n", clientCmdBuf);

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
            controlClient.printf("OK %d %d\n", port + 1, baud);
          }
        } else if (strstr(clientCmdBuf, "com ") == clientCmdBuf) {
          int port = atoi(clientCmdBuf + 4) - 1;
          if (port < 0 || port >= NUM_COMS) {
            controlClient.write("Invalid port\n");
          } else {
            controlClient.printf(">> Connecting to port %d\n", port + 1);
            controlCom = port;
            break;
          }
        } else if (strstr(clientCmdBuf, "status") == clientCmdBuf) {
          for (int i = 0; NUM_COMS > i; i++) {
            controlClient.printf("Port %d: %d baud\n", i + 1, COM[i].baud);
          }
        } else {
          controlClient.write("Commands:\n");
          controlClient.write("baud <port> <baud> -- set COM<port> baud rate (8n1 assumed)\n");
          controlClient.write("com <port> -- connect to COM<port>\n");
          controlClient.write("status\n");
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

    auto fromTcpToSerial = [&](WiFiClient& client, Stream& serial, bool handleEscape) {
      // read from client and write to serial port
      while (int nread = client.read(sBuffer, BUFFER_SIZE)) {
        sBuffer[nread] = 0;
        if (handleEscape) {
          if (char *esc = strchr(sBuffer, ESCAPE_CHAR)) {
            *esc = 0;
            nread = esc - sBuffer;
            controlCom = -1;
            controlClient.write(">>> Back to control console\n");
          }
        }
        DPRINTF("tcp->ser r: %d bytes '%s'\n", nread, sBuffer);
        serial.write(sBuffer, nread);
      }
    };

    if (controlCom == num) {
      fromTcpToSerial(controlClient, com.serial, true);
    }

    for (int i = 0; i < MAX_CLIENTS_PER_PORT; i++) {
      if (!com.clients[i].connected())
        continue;

      fromTcpToSerial(com.clients[i], com.serial, false);
    }

    // read from serial port and write to all clients
    while (int nread = com.serial.read(sBuffer, BUFFER_SIZE)) {
      sBuffer[nread] = 0;
      DPRINTF("ser r: %d bytes '%s'\n", nread, sBuffer);

      if (controlCom == num) {
        controlClient.write(sBuffer, nread);
      }

      for (int i = 0; i < MAX_CLIENTS_PER_PORT; i++) {
        if (com.clients[i].connected()) {
          DPRINTF("tcp w[%d]: %d bytes\n", i, nread);
          com.clients[i].write(sBuffer, nread);
        }
      }
    }
  }
}

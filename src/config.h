/*********************************************************************************
 * config.h
 * 
 * Added compatibility for ESP8266 -- Yuri - Aug 2021
 * 
*********************************************************************************/

#define OTA_HANDLER     // uncomment to enable OTA programming
#define MODE_STA        // or MODE_AP for access point
#define PROTOCOL_TCP    // TCP only, PROTOCOL_UDP is not presently implemented
//#define BLUETOOTH     // uncomment to create a bluetooth serial bridge (ESP32 only)
//#define BATTERY_SAVER // uncomment to reduce wifi power

const char *ssid = "MajorNet";          // SSID to broadcast (or join)
const char *pw = "barkbark11275";        // wifi password
const char *mdns_name = "esp32";    // hostname for STA mode mDNS

IPAddress apIP(192, 168, 200, 1);       // static IP for AP mode
IPAddress apNetmask(255, 255, 255, 0);

const bool debug = true; // set false to suppress system serial messages

#ifdef ESP32
#define VERSION "1.11-ESP32"
#elif defined(ESP8266)
#define VERSION "1.11-ESP8266"
#endif

#define CONTROL_PORT 8880

/**************************  COM Port 0 *******************************/
#define SERIAL0_BAUD 57600          // Baudrate UART0
#define SERIAL0_PARAM SERIAL_8N1  // Data/Parity/Stop UART0 (use SWSERIAL_* for ESP8266)
#define SERIAL0_TXPIN 1           // transmit Pin UART0
#define SERIAL0_RXPIN 21          // receive Pin UART0
#define SERIAL0_TCP_PORT 8881     // Wifi Port UART0
/*************************  COM Port 1 *******************************/
#define SERIAL1_BAUD 57600            // Baudrate UART1
#define SERIAL1_PARAM SERIAL_8N1    // Data/Parity/Stop UART1 (use SWSERIAL_* for ESP8266)
#define SERIAL1_TXPIN 17            // transmit Pin UART1
#define SERIAL1_RXPIN 16            // receive Pin UART1
#define SERIAL1_TCP_PORT 8882       // Wifi Port UART1
/*************************  COM Port 2 *******************************/
#define SERIAL2_BAUD 57600            // Baudrate UART2         (ESP32 only)
#define SERIAL2_PARAM SERIAL_8N1    // Data/Parity/Stop UART2 (ESP32 only)
#define SERIAL2_TXPIN 4             // transmit Pin UART2     (ESP32 only)
#define SERIAL2_RXPIN 15            // receive Pin UART2      (ESP32 only)
#define SERIAL2_TCP_PORT 8883       // Wifi Port UART2        (ESP32 only)

#define BUFFER_SIZE 1024
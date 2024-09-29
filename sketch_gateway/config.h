
// Generic config

// Version
const char* VERSION = "0.014c";
const char* NAME = "Gateway"; 

// address of this device
byte localAddress = 0xAA;           

// setting for MQTT liveness message 
#if TEST_ENABLED
#define intervalMillis 10000  // TEST, interval for liveness
#else
#define intervalMillis 300000  // interval for liveness
#endif
//const long intervalMillis = 10000;  // TEST, interval for liveness
//const long intervalMillis = 300000;  // interval for liveness


// LoRa define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//LoRa 866E6 for Europe
#define BAND 866E6

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// time factor
#define mS_TO_S_FACTOR 1000     // factor for milli seconds to seconds
#define uS_TO_S_FACTOR 1000000  // factor for micro seconds to seconds 



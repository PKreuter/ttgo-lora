


//Version
const char* VERSION = "0.014a";
const char* NAME = "Gateway"; 

//const long intervalMillis = 10000;  // TEST, interval for liveness
const long intervalMillis = 300000;  // interval for liveness


//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//866E6 for Europe
#define BAND 866E6

//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// time factor
#define mS_TO_S_FACTOR 1000
#define uS_TO_S_FACTOR 1000000  // conversion factor for micro seconds to seconds 







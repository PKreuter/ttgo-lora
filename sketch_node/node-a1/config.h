


//Version
const char* VERSION = "0.043";

const char* NAME = "Test-Node"; 
byte localAddress = 0xA2;          // address of LoRa device

//const char* NAME = "Briefkasten"; 
//byte localAddress = 0xA1;          // address of LoRa device

const int SLEEP = 10;         // seconds to Sleep
const int DEEP_SLEEP = 900;   // seconds to Sleep, Default 900



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
#define uM_TO_S_FACTOR 1000     // micro seconds to milli seconds


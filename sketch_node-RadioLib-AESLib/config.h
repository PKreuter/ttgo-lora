






//Version
const PROGMEM char* VERSION = "0.045";


//LoRa Destination Adress / send to
byte destination = 0xAA; 


// Speed Serial Monitor
#define BAUD 115200

const int SLEEP = 60;         // seconds to Sleep
const int DEEP_SLEEP = 900;   // seconds to Sleep, Default 900

// both false = USSensor
bool enableButton = false;
bool enableIRSensor = true;


// SPI
#define SCK 5
#define MISO 19
#define MOSI 27

// SX1278
#define SPI_LORA_SS 18
#define SPI_LORA_DIO0 26
#define SPI_LORA_RST 23
#define SPI_LORA_DIO0 3



/*
Der Spreading Factor (SF) beschreibt dabei wieviele Chirps, also Daten Carrier pro Sekunde übertragen werden. 
Dadurch ist die Bitrate, pro Symbol abgestrahlte Energie und die Reichweite definiert. 
Dabei wurde bei LoRa die „Adaptive Data Rate“ eingeführt: 
je nach Netzwerk Konfiguration wird über den Sender der Spreading Faktor zwischen SF7 und SF12 eingestellt.
SF7 = range 2km, time on air 61ms, 5470bps
**/
#define loRaSpreadingFactor 6

//866E6 for Europe
#define BAND 866E6

//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// time factor
#define mS_TO_S_FACTOR 1000
#define uS_TO_S_FACTOR 1000000  // conversion factor for micro seconds to seconds 
#define uM_TO_S_FACTOR 1000     // micro seconds to milli seconds


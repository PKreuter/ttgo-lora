
// Generic config

// Version
PROGMEM const char* VERSION = "x.001";
PROGMEM const char* NAME = "Gateway"; 

// address of this device
PROGMEM const byte localAddress = 0xBB;           

// Speed Serial Monitor
#define BAUD 115200

// setting for MQTT liveness message 
//PROGMEM const long intervalMillis = 10000;  // TEST, interval for liveness
PROGMEM const long intervalMillis = 300000;  // interval for liveness

#define LED_PIN 8

// Pins used by Wire
#define I2C_SDA 6
#define I2C_SCL 7


// SPI, Default for ESP32-C6 MOSI: 19, MISO: 20, SCK: 21, SS: 18
#define MOSI 19  // SDI
#define MISO 20  // SDO, DIO                        
#define SCK  21
#define SS   10

// Pins used by OLED 128x128
#define SPI_OLED_2_SS  18  // CS
#define SPI_OLED_2_RST 1   // RST

// OLED 128x128 und SX1262 vertragen sich nicht

// Pins used by LoRa SX1262 
#define SPI_LORA_SS   4
#define SPI_LORA_DIO1 11
#define SPI_LORA_RST  5
#define SPI_LORA_BUSY 3
// SX1262 has the following connections:
// NSS pin:   4
// DIO1 pin:  11
// NRST pin:  5
// BUSY pin:  3
// SX1262 radio = new Module(4, 11, 5, 3);
// SX1262 radio = new Module(4, 11, 5, 3);
//SX1262 radio = new Module(SPI_LORA_SS, SPI_LORA_DIO1, SPI_LORA_RST, SPI_LORA_BUSY);


//LoRa band, 866E6 for Europe
#define BAND 866E6


// OLED 128x64
#define SCREEN_1_WIDTH 128
#define SCREEN_1_HEIGHT 64

// OLED 128x128
#define SCREEN_2_WIDTH  128
#define SCREEN_2_HEIGHT 128 


// time factor
#define mS_TO_S_FACTOR 1000     // factor for milli seconds to seconds
#define uS_TO_S_FACTOR 1000000  // factor for micro seconds to seconds 








//Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//define OLED instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Instanziierung

//Initialize OLED Module
void initOLED() {
  Serial.println("Initializing OLED Display");
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3C for 128x32
    Serial.println(F("*SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  Serial.println(" OLED Display OK!");
  display.clearDisplay();
  display.display(); // zeigt den Grafikpuffer auf dem OLED-Display
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("OLED Display OK!");
  display.display();

}

void showVersion(){
  display.setCursor(0,0);
  display.print("Version: ");
  display.println(VERSION);
  display.setCursor(0,10);
  display.print("LoRa Node: 0x");
  display.println(localAddress, HEX);
  display.display();
}


//Libraries for LoRa
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

//Initialize LoRa Module
void initLoRa() {
  Serial.println("Initializing LoRa Node");
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  LoRa.setSpreadingFactor(loRaSpreadingFactor);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  //LoRa.dumpRegisters(Serial);
  LoRa.enableCrc();
  Serial.println(" LoRa Initializing OK!");
  display.setCursor(0,30);
  display.print("LoRa Initializing OK!");
  display.display();
}



//Libraries for DHT22/11
#include <DHT.h>
#include <DHT_U.h>
//Digital pin connected to the DHT sensor
#define DHTPIN 25
#define DHTTYPE DHT22 
//define DHT instance
DHT_Unified dht(DHTPIN, DHTTYPE);

//Initialize DHT Sensor
void initDHT() {
  // Initialize DHT device.
  Serial.println(F("Initializing DHT Sensor"));
  dht.begin();
  Serial.println(F(" DHT Sensor is OK!"));
}


//Libraries for BMP280
//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/**
BMP280 Pin Description Communication via IC2
  VCC: Connected to the positive supply of 3.3V
  GND: Common Ground pin.
  SCL: Serial Clock pin 
  SDA: Serial Data pin 
**/
Adafruit_BMP280 bmp;     // Communication via I2C
#define BMP280 0x77      // SCB & SDO +5V

//Initialize BMP280 Sensor
void initBMP() {
  Serial.println(F("Initializing BMP280 Sensor"));
  if (!bmp.begin()) {  
    Serial.println(" Could not find a valid BMP280 !");
    while (1);
  }
  else {
    Serial.println(F(" BMP280 is OK!"));
  }
  bmp.takeForcedMeasurement();

    /* Default settings from datasheet. */

  //bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  //startAltitude = bmp.readAltitude(1013.25);

}
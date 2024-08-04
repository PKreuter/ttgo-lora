/**

This is the LoRa Node Code 

LILYGO / TTGO LORA - Model T3V1.6.1

1) Node send a JSON String to LoRA Gateway
{"data":[
  {"name":"Test-Node","node":"0xA2","type":"lora","message":3,"temperature":0,"humidity":0,"pressure":0,"sensor_state":"low","diepostistda":"true","sensor_value":0,"vbattery":4.527970791}
]}


2) LoRa Gateway receive LoRa message
3) LoRa Gateway add ts and send to MQTT over WiFi
{"ts":1722536448,
 "data":[
  {"name":"Test-Node","node":"0xA2","type":"lora","message":763,"temperature":0,"humidity":0,"pressure":0,"sensor_state":"low","diepostistda":"true","sensor_value":70,"vbattery":4.519106388}
]}




CHANGES
- IR Sensor DFRobot SEN0523 change to Analog-Input


RULES
- Default with DEEP_SLEEP_MODE
- Using RTC Memory to Store Data During Sleep
- Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 

External Componets
- IR Sensor DFRobot SEN0523
- DHT22


Pull Widerstand 10K nach 5V for sleepModePin

*/ 

#include "config.h" 

//Libraries for LoRa
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

//Libraries for BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Libraries for DHT22/11
#include <DHT.h>
#include <DHT_U.h>

//Libraries to create Json Documents
#include <ArduinoJson.h>


const int SLEEP = 60;         // seconds to Sleep
const int DEEP_SLEEP = 900;   // seconds to Sleep, Default 900


//Digital pin connected to the DHT sensor
#define DHTPIN 25
#define DHTTYPE DHT22 

/**
BMP280 Pin Description
VCC: Connected to the positive supply of 3.3V
GND: Common Ground pin.
SCL: Serial Clock pin 
SDA: Serial Data pin 
ID of 0x56-0x58 represents a BMP 280,
*/
Adafruit_BMP280 bmp;     // Communication via I2C
#define BMP280 0x77
#define SEALEVELPRESSURE_HPA (1013.25)

// digital IO of the sleep mode enable pin, 
//  high=sleep enabled, default
//  low=sleep disabled 
const int sleep900ModePin = 13;
const int sleep60ModePin = 14;

// variable for reading the sleepmode
int sleep900ModeState = LOW; 
int sleep60ModeState = LOW;  

// analog IO of the IR Sensor DFRobot SEN0523
const int sensorPin = 4;
// variable for storing the pin value and states
//  Value of 4095 => IR Strecke OK, State = False
//  Value of < 1000 => IR Strecke unterbroachen, State = TRUE
int sensorValue = 0;
int sensorState = LOW;

// analog IO of the VBAT   
const uint8_t vbatPin = 35; 


////define OLED instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Instanziierung

// OLED line 0..6, writePixel (x, y, color)
int disPos_y0 = 0;
int disPos_y1 = 11;
int disPos_y2 = 20;
int disPos_y3 = 29;
int disPos_y4 = 38;
int disPos_y5 = 47;
int disPos_y6 = 56;


//global variables for temperature and Humidity
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;

// battery voltage from ESP32 ADC read
float VBAT;  

//initilize packet counter
int readingID = 0;
String LoRaMessage = "";


// Using RTC Memory to Store Data During Sleep
// Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 
RTC_DATA_ATTR unsigned int msgCounter = 0;


byte destination = 0xAA;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 50;            // interval between sends

//define DHT instance
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

JsonDocument doc;
char jsonSerial[500];




void setup() {

  //initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("***LoRa Node - Version " +String(VERSION));

  // Battery Pin as an input 
  pinMode(vbatPin, INPUT);
  // initialize the IR pin as an input, die-post
  pinMode(sensorPin, INPUT);
  // initialize the pushbutton pin as an input, display on/off
  pinMode(sleep900ModePin, INPUT);
  pinMode(sleep60ModePin, INPUT);

  // initialize Sensors/OLED/LoRa
  initDHT();
  //initBMP();
  initOLED();
  showVersion();
  initLoRa();

  // sleep to hold messages on display
  delay(2 * mS_TO_S_FACTOR);

  getSleepModeState();
  Serial.print("Deep sleep mode enabled: ");
  if (sleep900ModeState == LOW) {
    Serial.println("true");
  }
  else {
    Serial.println("false");
  }

}


void showVersion(){
  display.setCursor(0,0);
  display.print("Version: ");
  display.println(VERSION);
  display.setCursor(0,10);
  display.print("LoRa Node: 0x");
  display.println(localAddress, HEX);
  display.display();
  delay(2 * mS_TO_S_FACTOR); 
}


//Initialize OLED Module
void initOLED() {
  Serial.println("Initializing OLED Display");
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  Serial.println("OLED Display OK!");

  display.clearDisplay();
  display.display(); // zeigt den Grafikpuffer auf dem OLED-Display
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("OLED Display OK!");
  display.display();

}


//Initialize LoRa Module
void initLoRa() {
  Serial.println("Initializing LoRa Node");
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.enableCrc();

  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,30);
  display.print("LoRa Initializing OK!");
  display.display();
}

//Initialize BMP280 Sensor
void initBMP() {
  Serial.println(F("Initializing BMP280 Sensor"));
 
 /**
  if (!bmp.begin(0x76,0x58)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
*/

  
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 !");
  //  while (1);
  }
  else {
    Serial.println(F("BMP280 is OK!"));
  }
}

//Initialize DHT Sensor
void initDHT() {
  // Initialize DHT device.
  Serial.println(F("Initializing DHT Sensor"));
  dht.begin();
  Serial.println(F("DHT Sensor is OK!"));
}

//function for fetching DHT readings
void getDHTreadings() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Temperature=event.temperature;
    Serial.print(Temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Humidity = event.relative_humidity;
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}


//function for fetching BMP280 readings
void getBMPreadings() {
  Pressure=bmp.readPressure()/133;
  Serial.print(F("Pressure: "));
  Serial.print(Pressure);
  Serial.println(F(" mmHg"));
}


// 
void getSensorValue() {
  sensorValue = analogRead(sensorPin);
  Serial.print("Sensor value: ");
  Serial.print(sensorValue);
  if (sensorValue < 3000) {
    sensorState = LOW;
    Serial.print(" - Sensor state: ");
  } else {
    sensorState = HIGH;
  }
  Serial.println(sensorState);

}

void getSleepModeState() {
  // HIGH = Deep Sleep Mode enabled
  // LOW = Deep Sleep Mode disabled
  sleep900ModeState = digitalRead(sleep900ModePin);
  sleep60ModeState = digitalRead(sleep60ModePin);
}

//function for fetching All readings at once
void getReadings() {
  getDHTreadings();
  //getBMPreadings();
  getSensorValue();
}

//Display Readings on OLED
void displayReadings() {
  display.clearDisplay();
  display.setCursor(0,disPos_y0);
  display.print("RUNNING... Status: OK");
  display.setCursor(0,disPos_y1);
  display.setTextSize(1);
  display.print("Temperature : ");
  display.setCursor(80,disPos_y1);
  display.print(Temperature);
  display.drawCircle(116,disPos_y1,1, WHITE);
  display.setCursor(121,disPos_y1);
  display.print("C");
  display.setCursor(0,disPos_y2);
  display.print("Humidity(%) : ");
  display.setCursor(80,disPos_y2);
  display.print(Humidity);
  display.setCursor(116,disPos_y2);
  display.print("Rh");
  display.setCursor(0,disPos_y3);
  display.print("Pressure : ");
  display.setCursor(62,disPos_y3);
  display.print(Pressure);
  display.setCursor(103,disPos_y3);
  display.print("mmHg");
  display.setCursor(0,disPos_y4);
  display.print("Battery : ");
  display.setCursor(60,disPos_y4);
  display.print(VBAT);
  display.setCursor(98,disPos_y4);
  display.print("Volts");
  display.display();

  if (sensorState == LOW) {
    display.setCursor(0,disPos_y5);
    display.print("Die Post : TRUE / ");
    display.setCursor(103,disPos_y5);
    display.print(sensorValue);
  } else {
    display.setCursor(0,disPos_y5);
    display.print("Die Post : FALSE / ");
    display.setCursor(103,disPos_y5);
    display.print(sensorValue);    
  }
  display.setCursor(0,disPos_y6);
  display.print("LoRa send :");
  display.setCursor(80,disPos_y6);
  display.print(msgCounter);
  display.display();  
}

void getVbat() {
  // Battery Voltage
  VBAT = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;
  /*
  The ADC value is a 12-bit number, so the maximum value is 4095 (counting from 0).
  To convert the ADC integer value to a real voltage you’ll need to divide it by the maximum value of 4095,
  then double it (note above that Adafruit halves the voltage), then multiply that by the reference voltage of the ESP32 which 
  is 3.3V and then vinally, multiply that again by the ADC Reference Voltage of 1100mV.
  */
  Serial.print("Battery: "); 
  Serial.print(VBAT); 
  Serial.println(" Volts");
}


//Send data to receiver node using LoRa
void sendReadings() {
  msgCounter++;

  // localAddress as 0xAA
  char nodeAddress[10];
  sprintf(nodeAddress, "0x%02X", localAddress);

  JsonDocument doc;
  char jsonSerial[500];
  JsonArray data = doc["data"].to<JsonArray>();
  JsonObject doc1 = data.createNestedObject();
  //doc1["uptime"] = x_uptime;    // millis
  doc1["name"] = NAME;
  doc1["node"] = nodeAddress;
  doc1["type"] = "lora";
  doc1["message"] = msgCounter;
  doc1["temperature"] = Temperature;
  doc1["humidity"] = Humidity;
  doc1["pressure"] = Pressure;

  if (sensorState == LOW) {
    doc1["sensor_state"] = "low";
    doc1["diepostistda"] = "true";
  } else {
    doc1["sensor_state"] = "high";
    doc1["diepostistda"] = "false";
  }
  doc1["sensor_value"] = sensorValue;
  doc1["vbattery"] = VBAT;
  serializeJson(doc, jsonSerial);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(byte(msgCounter));         // add message ID
  LoRa.write(LoRaMessage.length());     // add payload length
  LoRa.print(jsonSerial);
  LoRa.endPacket();

  Serial.printf("Packet Sent: %d\n", msgCounter);
  Serial.println(jsonSerial);
   
  displayReadings();
}



void loop() {

  getReadings();
  getVbat();
  sendReadings();

  // sleep to hold messages on display
  delay(5 * mS_TO_S_FACTOR); 
  
  // check sleep mode state
  getSleepModeState();

  // check for sleep mode
    /**
    Nomalbetrieb ~40 mA
    Deep Sleep Mode ~3 mA
    Deep Sleep Mode mit Display=on ~10mA
    **/
  if (sleep900ModeState == LOW) {
    Serial.printf("Go into deep sleep mode for %i seconds\n", DEEP_SLEEP);
    display.dim(true);
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  } 
  else if (sleep60ModeState == LOW) {
    Serial.printf("Go into sleep mode for %i seconds\n", SLEEP);
    display.dim(true);
    esp_sleep_enable_timer_wakeup(SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start(); 
  } 
  else {
    Serial.println("Go to next ...");
  }

}


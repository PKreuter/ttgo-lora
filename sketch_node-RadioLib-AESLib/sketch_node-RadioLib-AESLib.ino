/**

This is the LoRa Node Code 
- LILYGO / TTGO LORA - Model T3V1.6.1
- Send Data as JSON
- Use AES to encrypt payload

!!! Zum Download US Sensor entfernen !!!

// you should not use global defined variables of type String. The variable-type String eats up all RAM over time

RULES
- Default with DEEP_SLEEP_MODE
- Using RTC Memory to Store Data During Sleep
- Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 

External Components
- JOY-IT SEN-US01
- JOY_IT BMP280 (not used)
- DHT22

Arduion IDE
- Select Board : TTGO LoRa32-OLED
-
Chip is ESP32-PICO-D4 (revision v1.1)
Features: WiFi, BT, Dual Core, 240MHz, Embedded Flash, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz

PINs, 
  0   Boot
  2   LOW=Chip in download mode.
      Sensor Trigger
  4   Sensor US Echo / PressButton / IR
  5   LoRa SCK
  12  ?? darf nigh HIGH sein sonst (RTCWDT_RTC_RESET)
  13  Sleep PIN digital => PULLUP 10K
  14  LED Pin / MOS-FET to enabled Power for Sensors
  15  DHT Sensor
  18  LoRa CS / SS
  19  LoRa MISO
  21  SDA  Display
  22  SLC  Display
  23  LoRa RST
  25  LED green
  26  LoRa DIO0
  27  LoRa MOSI
  34  Wakeup digital
  35  interval VBAT analog
  36  Sleep PIN digital => PULLUP 10K
  39  Debug => Default LOW, ON = 10K to VCC 


  Wenn VBAT dauernd 7.27 V anzeigt siimmt was mit den Widerstand nicht
  PIN: 13, 36 via 10K to VCC

  
 
  LED
  - red     VBUS
  - blue    Battery
  - green   IO25 

  Interfaces
  - LoRa SPI
  - TF Card SPI
  - OLED IC2

*/ 

//#include "config-node-a1.h"
#include "config-node-a2.h"

// Sensitive configs
#include "secrets.h"   

// 
#include "config.h" 
#include "log.h"
#include "init.h"
#include "sensor.h"

//https://github.com/adafruit/Adafruit_SleepyDog/tree/master
#include <Adafruit_SleepyDog.h>

//Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "oled.h"

//Libraries to create Json Documents
#include <ArduinoJson.h>

/** Define Sleep Mode 
 both HIGH = Mode 1 Deep
 others = Mode 2 'sleep'
 both LOW = Disabled as 'wait_for'
**/
const int sleepModePin1 = 13;  
const int sleepModePin2 = 36; //12; //39;   //2
// variable for store the sleepmode
int sleepMode = 1;   // 0=no sleep, 1=deep, 2=short
int wait_for = 15;   // every n seconds

// digital IO as Output
const int pwrPin =  14;  // Power Sensor

//const char* wakeUpPin = GPIO_NUM_34;
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
//#define GPIO_NUM_34 34

// battery voltage from ESP32 ADC read
// analog IO of the VBAT   
const uint8_t vbatPin = 35; 
float VBAT;

// Send LoRa packets
const int ledPin =  25;

// OLED line 0..6, writePixel (x, y, color)
int displayRow1 = 0;
int displayRow2 = 11;
int displayRow3 = 19;
int displayRow4 = 28;
int displayRow5 = 38;
int displayRow6 = 47;
int displayRow7 = 56;


// Buffer to write message to display
char text[25] = {0};


//global variables for temperature and Humidity
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;
uint32_t delayMS;  // ???


// Using RTC Memory to Store Data During Sleep, so that it will not be deleted during the deep sleep
// Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 
RTC_DATA_ATTR unsigned int msgCounter = 0;
RTC_DATA_ATTR int bootCount = 0; 


// encrypt / decrypt lora messages
#include "aes.h"
#include <base64.h>


void bufferSize(char* text, int &length)
{
  int i = strlen(text);
  int buf = round(i / BLOCK_SIZE) * BLOCK_SIZE;
  length = (buf <= i) ? buf + BLOCK_SIZE : length = buf;
}


// RadioLib
#include <RadioLib.h>
SX1276 radio = new Module(SPI_LORA_SS, SPI_LORA_DIO0, SPI_LORA_RST, 3);

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

//Libraries for LoRa
#include <SPI.h>



//Initialize LoRa Module SX1278
void initRadioSX() {

  debugOutput("[SX1278] Initializing ... ", 4);
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    debugOutput("[SX1278]   success!", 4);
  } else {
    debugOutput("[SX1278]   failed, code " +String(state), 2);
    while (true) { delay(10); }
  }

  // set carrier frequency to 433.5 MHz
  if (radio.setFrequency(866.6) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    debugOutput("[SX1278] Selected frequency is invalid for this module!", 2);
    while (true) { delay(10); }
  }

  // set LoRa sync word to 0x14
  // NOTE: value 0x34 is reserved for LoRaWAN networks and should not be used
  if (radio.setSyncWord(0x14) != RADIOLIB_ERR_NONE) {
    debugOutput("[SX1278] Unable to set sync word!", 2);
    while (true) { delay(10); }
  }

  // set output power to 10 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission
  //       duty cycle MUST NOT exceed 1%
  if (radio.setOutputPower(10) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    debugOutput("[SX1278] Selected output power is invalid for this module!", 2);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag, RISING);

  // update display
  display.setCursor(0,30);
  if (state == RADIOLIB_ERR_NONE) {
    display.print("LoRa Initializing OK!");
  } else {
    display.print("LoRa Initializing NOK!");
  }
  display.display();

}



void showVersion()
{
  sprintf(text, "Version %s", String(VERSION));
  oledWriteMsg(0, text);
  sprintf(text, "LoRa Node 0x%s", String(localAddress, HEX));
  oledWriteMsg(10, text);
}

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;
void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}

void setup()
{
  //initialize Serial Monitor
  Serial.begin(BAUD);

  // Init Logger
  isDebugEnabled();

  debugOutput("***LoRa Node - Version " +String(VERSION), 5);

  // Slowing down the ESP32 to 1/4 of its speed saves more energy
	setCpuFrequencyMhz(60);

  // Battery Pin as an analog input 
  pinMode(vbatPin, INPUT);
 
  // initialize Sleep Pin, digital input
  pinMode(sleepModePin1, INPUT_PULLUP);
  pinMode(sleepModePin2, INPUT_PULLUP);

  // initialize the sensor 
  if ( enableButton == true ) {
    pinMode(sensorPin, INPUT_PULLDOWN);
  }
  else if ( enableIRSensor == true ) {
    pinMode(sensorPin, INPUT_PULLDOWN);
  }
  else {
    initSensorUS();
  }

  // initialize the power pin as an output
  pinMode(pwrPin, OUTPUT);
  digitalWrite(pwrPin, HIGH); // set power on

  // initialize green LED to indicate send packets
  pinMode(ledPin, OUTPUT);

  // initialize Sensors/OLED/LoRa
  initDHT();
  if ( enableBMP == true ) {
    initBMP();
  }
  initOLED();
  showVersion();
  initRadioSX();

  // AES
  aes_init();
  aesLib.set_paddingmode(paddingMode::CMS);

  //Increments boot number and prints it every reboot
  bootCount++;
  debugOutput("Boot number: " + String(bootCount), 5);

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,1); //1 = High, 0 = Low
  
  // sleep to hold messages on display
  delay(1 * mS_TO_S_FACTOR);

}



void print_sleep_info()
{
  if (sleepMode == 1) {
    oledWriteMsg(45, "SLEEP Mode 1 - DEEP");
    sprintf(text, "Update every %ss", String(DEEP_SLEEP));
    oledWriteMsg(55, text);
  }
  else if (sleepMode == 2) {
    oledWriteMsg(45, "SLEEP Mode 2");
    sprintf(text, "Update every %ss", String(SLEEP));
    oledWriteMsg(55, text);
  }
  else {
    oledWriteMsg(45, "SLEEP Mode 0");
    sprintf(text, "Update every %ss", String(wait_for));
    oledWriteMsg(55, text);
  }
}


//function for fetching DHT readings
void getDHTreadings()
{
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    debugOutput("*Error reading temperature!", 3);
  }
  else {
    Temperature = event.temperature;
    debugOutput("Temperature: " +String(Temperature)+ "°C", 4);
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    debugOutput("*Error reading humidity!", 3);
  }
  else {
    Humidity = event.relative_humidity;
    debugOutput("Humidity: " +String(Humidity)+ "%", 4);
  }
}


//function for fetching BMP280 readings
void getBMPreadings()
{
  //float SLP = bmp.seaLevelForAltitude(SEALEVELPRESSURE_HPA, Pressure);
  //float LLP = bmp.readAltitude(SLP);  //SEALEVELPRESSURE_HPA
  //Pressure = LLP;
  Pressure = bmp.readPressure()/100;
  debugOutput("Pressure: " +String(Pressure)+ " hPa", 4);

  Temperature = bmp.readTemperature();
  debugOutput("Temperature: " +String(Temperature)+ " *C", 4);
}



void getSleepModeState()
{

  int sleepModeValue1 = 0;
  int sleepModeValue2 = 0;
  sleepModeValue1 = digitalRead(sleepModePin1);
  sleepModeValue2 = digitalRead(sleepModePin2);

  debugOutput("Sleep PIN: " +String(sleepModePin1)+ " - Value: " +String(sleepModeValue1), 5);
  debugOutput("Sleep PIN: " +String(sleepModePin2)+ " - Value: " +String(sleepModeValue2), 5);

  if (sleepModeValue1 == HIGH and sleepModeValue2 == HIGH ) {
    sleepMode = 1;
    debugOutput("Sleep enabled, Mode " +String(sleepMode), 5);
  } 
  else if (sleepModeValue1 == LOW and sleepModeValue2 == LOW) {
    sleepMode = 0;
     debugOutput("Sleep disabled ", 5);
    } 
    else { 
      sleepMode = 2;
      debugOutput("Sleep enabled, Mode " +String(sleepMode), 5);
    }

}


/*
Method to print the reason by which ESP32 has been awaken from sleep
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : debugOutput("Wakeup caused by external signal using RTC_IO", 5); break;
    case ESP_SLEEP_WAKEUP_EXT1 : debugOutput("Wakeup caused by external signal using RTC_CNTL", 5); break;
    case ESP_SLEEP_WAKEUP_TIMER : debugOutput("Wakeup caused by timer", 5); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : debugOutput("Wakeup caused by touchpad", 5); break;
    case ESP_SLEEP_WAKEUP_ULP : debugOutput("Wakeup caused by ULP program", 5); break;
    default : debugOutput("Wakeup was not caused by deep sleep: " +String(wakeup_reason), 5); break;
  }
}



//Display Readings on OLED
void displayReadings()
{
  display.clearDisplay();
  oledWriteMsg(displayRow1, "RUNNING... Status: OK");
  sprintf(text, "Temperature : %sC", String(Temperature));
  oledWriteMsg(displayRow2, text);
  sprintf(text, "Humidity(%) : %sRh", String(Humidity));
  oledWriteMsg(displayRow3, text);
  sprintf(text, "Pressure : %s mmHg", String(Pressure));
  oledWriteMsg(displayRow4, text);
  sprintf(text, "Battery : %s Volts", String(VBAT));
  oledWriteMsg(displayRow5, text);

  // common rule
  if (sensorState == LOW && sensorValue == 0) {
    sprintf(text, "Post : ERROR / %s", String(sensorValue));
    oledWriteMsg(displayRow6, text);
  }
  else if (sensorState == HIGH) {
    sprintf(text, "Post : TRUE / %s", String(sensorValue));
    oledWriteMsg(displayRow6, text);    
  } else {
    sprintf(text, "Post : FALSE / %s", String(sensorValue));
    oledWriteMsg(displayRow6, text); 
  }


  sprintf(text, "LoRa send :%s", String(msgCounter));
  oledWriteMsg(displayRow7, text); 
}

void getVbat()
{
  // Battery Voltage
  VBAT = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;
  /*
  The ADC value is a 12-bit number, so the maximum value is 4095 (counting from 0).
  To convert the ADC integer value to a real voltage you’ll need to divide it by the maximum value of 4095,
  then double it (note above that Adafruit halves the voltage), then multiply that by the reference voltage of the ESP32 which 
  is 3.3V and then vinally, multiply that again by the ADC Reference Voltage of 1100mV.
  */
  debugOutput("Battery: " +String(VBAT)+ " Volts", 5); 
}


//Send data to receiver node using LoRa
void sendReadings()
{
  msgCounter++;

  // localAddress as 0xAA
  char nodeAddress[10];
  sprintf(nodeAddress, "0x%02X", localAddress);

  JsonDocument doc;
  char jsonSerial[230];
  // create nestsed object: {"data":[{"node":"0xA2"
  //JsonArray data = doc["data"].to<JsonArray>();
  //JsonObject doc1 = data.createNestedObject();
 
  // create an object
 // JsonObject object = doc.to<JsonObject>();
  JsonObject doc1 = doc.to<JsonObject>();

  doc1["node"] = nodeAddress;
  doc1["msg_num"] = msgCounter;
  doc1["temperature"] = Temperature;
  doc1["humidity"] = Humidity;
  //doc1["pressure"] = Pressure;

  doc1["sensor_value"] = sensorValue;
  if (sensorState == LOW && sensorValue == 0) {
    doc1["sensor_state"] = "false";
  }
  else {
    doc1["sensor_state"] = "true";
  }

  doc1["vbattery"] = VBAT;
  doc1["wakeup_reason"] = esp_sleep_get_wakeup_cause();

  // serialize
  serializeJson(doc, jsonSerial);
  debugOutput("Jsontext: " +String(jsonSerial), 5);

  // Encrypt
  byte enc_iv[BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String encrypted = encrypt_impl(jsonSerial, enc_iv);

  // send ERROR message wenn Packet > 256
  if(encrypted.length() > 240) {
    debugOutput("LoRa Packet to big", 2);
    JsonDocument doc;
    char jsonSerial[230];
    JsonArray data = doc["data"].to<JsonArray>();
    JsonObject doc1 = data.createNestedObject();
    doc1["node"] = nodeAddress;
    doc1["msg_num"] = msgCounter;
    doc1["message"] = "ERROR LoRa Packet to big";
    serializeJson(doc, encrypted);
  }

  debugOutput("Ciphertext: " +String(encrypted), 5);
  debugOutput("Ciphertext: " +String(encrypted.length()), 5);

  //Send LoRa packet to receiver
  debugOutput("LoRa, begin to send packet to Gateway", 4);
  digitalWrite(ledPin, HIGH); 

  transmissionState = radio.startTransmit(encrypted);

  /**
  LoRa.beginPacket();
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(byte(msgCounter));         // add message ID
  LoRa.write(encrypted.length());        // add payload length
  LoRa.print(encrypted);                 // add payload
  LoRa.endPacket();
  //LoRa.endPacket(true); // true = async / non-blocking mode
  **/

  
  digitalWrite(ledPin, LOW); 

  debugOutput("LoRa, send packet done, message number: " +String(msgCounter), 4);

  displayReadings();

}


// prepare for sleep
void do_ready_for_sleep()
{
  display.dim(true);
  display.clearDisplay();
  radio.sleep();

  digitalWrite(pwrPin, LOW);    // disable Power US   
}


//function for fetching All readings at once
void getReadings()
{
  getDHTreadings();
  if ( enableBMP == true ) {
    getBMPreadings();
  }
  /** Sensor enable one of them **/
  if ( enableButton == true ) {
    getButtonState();     // as digital IO based on Press-Button
  }
  else if ( enableIRSensor == true ) {
    getSensorIRState();   // as analog IO
  }
  else {
    getSensorUSValue();   // as analog IO
  }
  getVbat();
  if (sleepMode > 0) {
    digitalWrite(pwrPin, LOW);    // disable Power US 
  } 
}




void loop()
{

  debugOutput("---", 4);

  getReadings();
  sendReadings();

  // sleep to hold messages on display
  delay(2 * mS_TO_S_FACTOR); 
  // check sleep mode state
  getSleepModeState();

  // check for sleep mode
    /**
    Nomalbetrieb ~40 mA
    Deep Sleep Mode mit Display=on ~10mA
    Deep Sleep Mode ~5 mA
    Zuseatzlich mit LoRa.sleep() ~2mA
    Wenn Sensor mit Power dann switch-off sensor = ~2mA
    **/
  if (sleepMode == 1) {
    debugOutput("Go into deep sleep mode for " +String(DEEP_SLEEP)+ " seconds", 4);
    display.clearDisplay();
    print_sleep_info();
    delay(1 * mS_TO_S_FACTOR); 
    do_ready_for_sleep();
    esp_sleep_enable_timer_wakeup((DEEP_SLEEP -4) * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  } 
  else if (sleepMode == 2) {
    debugOutput("Go into deep sleep mode for " +String(SLEEP)+ " seconds", 4);
    display.clearDisplay();
    print_sleep_info();;
    delay(1 * mS_TO_S_FACTOR); 
    do_ready_for_sleep();
    esp_sleep_enable_timer_wakeup((SLEEP -4) * uS_TO_S_FACTOR);
    esp_deep_sleep_start(); 
  } 
  else {
    delay((5) * mS_TO_S_FACTOR);  // show display for +5 secondsdelay
    display.clearDisplay();
    print_sleep_info();;
    delay((wait_for -9) * mS_TO_S_FACTOR);  // not sleep but wait some seconds
  }

}


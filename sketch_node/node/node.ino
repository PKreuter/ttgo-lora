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
- US
- IR Sensor DFRobot SEN0523 change to Analog-Input
- BMP280


RULES
- Default with DEEP_SLEEP_MODE
- Using RTC Memory to Store Data During Sleep
- Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 

External Components
- JOY-IT SEN-US01
- JOY_IT BMP280
- DHT22


PINs, welche wahrscheinlich reserviert sind
  0   Boot / Sensor US Trigger
  2   LOW=Chip in download mode.
  4   Sensor US Echo
  5   LoRa SCK
  12  ??
  13  
  14  LED Pin / MOS-FET to enabled Power for Sensors
  15  DHT Sensor   // PIN 15 ev nur digital
  18  LoRa CS / SS
  19  LoRa MISO
  23  LoRa RST
  25  
  26  LoRa DIO0
  27  LoRa MOSI
  34  Wakeup digital
  35  interval VBAT analog
  36  Sleep PIN digital
  39  Sleep PIN digital

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

#include "config.h" 
#include "sensor.h"
#include "init.h"

//https://github.com/adafruit/Adafruit_SleepyDog/tree/master
#include <Adafruit_SleepyDog.h>

//Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Libraries to create Json Documents
#include <ArduinoJson.h>

/** Define Sleep Mode 
 both HIGH = Mode 1 Deep
 others = Mode 2 'sleep'
 both LOW = Disabled as 'wait_for'
**/
const int sleepModePin1 = 36;   //2
const int sleepModePin2 = 39;   //2
// variable for store the sleepmode
int sleepMode = 1;   // 0=no sleep, 1=deep, 2=short
int wait_for = 15;   // every n seconds

// digital IO as Output
const int pwrPin =  14;  // Power Sensor
//const int pwrPin =  25;  // Power Sensor

//const char* wakeUpPin = GPIO_NUM_34;
#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

// analog IO of the VBAT   
const uint8_t vbatPin = 35; 


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


// Using RTC Memory to Store Data During Sleep, so that it will not be deleted during the deep sleep
// Place RTC_DATA_ATTR in front of any variable that you want to store in RTC memory. 
RTC_DATA_ATTR unsigned int msgCounter = 0;
RTC_DATA_ATTR int bootCount = 0; 



long lastSendTime = 0;        // last send time
int interval = 50;            // interval between sends


uint32_t delayMS;

JsonDocument doc;
char jsonSerial[500];


void setup() {

  //initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("***LoRa Node - Version " +String(VERSION));

  // Battery Pin as an analog input 
  pinMode(vbatPin, INPUT);
 
  // initialize Sleep Pin as an analog input
  pinMode(sleepModePin1, INPUT);
  pinMode(sleepModePin2, INPUT);

  // initialize the sensor 
  if ( enableButton == true ) {
    pinMode(sensorPin, INPUT);
  }
  else {
    initSensorUS();
  }

  // initialize the power pin as an output
  pinMode(pwrPin, OUTPUT);
  digitalWrite(pwrPin, HIGH); // set power on
 
  // initialize Sensors/OLED/LoRa
  initDHT();
  if ( enableBMP == true ) {
    initBMP();
  }
  initOLED();
  showVersion();
  initLoRa();

  //Increments boot number and prints it every reboot
  bootCount++;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,1); //1 = High, 0 = Low

  // get and print sleep mode to OLED
  //getSleepModeState();
  //print_sleep_info();
  
  // sleep to hold messages on display
  delay(1 * mS_TO_S_FACTOR);

}



void print_sleep_info() {
  if (sleepMode == 1) {
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,45);
    display.print("SLEEP Mode 1 DEEP");
    display.setCursor(0,55);
    display.print("Update every ");
    display.print(DEEP_SLEEP);
    display.print("s");    
    display.display();
  }
  else if (sleepMode == 2) {
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,45);
    display.print("SLEEP Mode 2");
    display.setCursor(0,55);
    display.print("Update every ");
    display.print(SLEEP);
    display.print("s");    
    display.display();
  }
  else {
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,45);
    display.print("SLEEP Mode 0");
    display.setCursor(0,55);
    display.print("Update every ");
    display.print(wait_for);
    display.print("s");
    display.display();
  }

}


//function for fetching DHT readings
void getDHTreadings() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("*Error reading temperature!"));
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
    Serial.println(F("*Error reading humidity!"));
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
  Pressure = bmp.readPressure()/100;
  //float SLP = bmp.seaLevelForAltitude(SEALEVELPRESSURE_HPA, Pressure);
  //float LLP = bmp.readAltitude(SLP);  //SEALEVELPRESSURE_HPA
  //Pressure = LLP;
  Serial.print(F("Pressure: "));
  Serial.print(Pressure);
  Serial.println(F(" hPa"));
  Temperature = bmp.readTemperature();
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

}


void getSleepModeState() {

  int sleepModeValue1 = 0;
  int sleepModeValue2 = 0;
  /**
  sleepModeValue = analogRead(sleepModePin);
  
  Serial.print("Sleep PIN: ");
  Serial.print(sleepModePin);
  Serial.print(" - Value: ");
  Serial.print(sleepModeValue);
  Serial.print(" - Sleep: ");
  if (sleepModeValue < 300) {
    sleepMode = 1;
    Serial.print("enabled");
  } 
  else if (sleepModeValue == 4095) {
    sleepMode = 0;
    Serial.print("disabled");
  } 
  else { 
    sleepMode = 2;
    Serial.print("enabled");
  }
  Serial.printf(" - Mode: %i\n", sleepMode);
  **/
  sleepModeValue1 = digitalRead(sleepModePin1);
  sleepModeValue2 = digitalRead(sleepModePin2);
  Serial.print("Sleep PIN: ");
  Serial.print(sleepModePin1);
  Serial.print(" - Value: ");
  Serial.print(sleepModeValue1);
  Serial.print(" :: Sleep PIN: ");
  Serial.print(sleepModePin2);
  Serial.print(" - Value: ");
  Serial.print(sleepModeValue2);
  Serial.print(" ==> Sleep: ");

  if (sleepModeValue1 == HIGH and sleepModeValue2 == HIGH ) {
    sleepMode = 1;
    Serial.print("enabled");
  } 
  else if (sleepModeValue1 == LOW and sleepModeValue2 == LOW) {
    sleepMode = 0;
    Serial.print("disabled");
  } 
  else { 
    sleepMode = 2;
    Serial.print("enabled");
  }
  Serial.printf(" - Mode: %i\n", sleepMode); 

}


/*
Method to print the reason by which ESP32 has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
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

  if (sensorState == LOW && sensorValue == 0) {
    display.setCursor(0,disPos_y5);
    display.printf("Post : ERROR / %i", sensorValue);
  }
  else if (sensorState == LOW) {
    display.setCursor(0,disPos_y5);
    display.printf("Post :   TRUE / %i", sensorValue);
  } else {
    display.setCursor(0,disPos_y5);
    display.printf("Post :   FALSE / %i", sensorValue);
  }
  display.setCursor(0,disPos_y6);
  display.printf("LoRa send : %i", msgCounter);
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
  Serial.printf("Battery: %.2f Volts\n", VBAT); 
}


void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}



//Send data to receiver node using LoRa
void sendReadings() {

  /**
  //Watchdog.disable();
  int countdownMS = Watchdog.enable(20 * 1000);
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
  **/

  msgCounter++;

  // localAddress as 0xAA
  char nodeAddress[10];
  sprintf(nodeAddress, "0x%02X", localAddress);

  JsonDocument doc;
  char jsonSerial[500];
  JsonArray data = doc["data"].to<JsonArray>();
  JsonObject doc1 = data.createNestedObject();
  doc1["name"] = NAME;
  doc1["node"] = nodeAddress;
  doc1["type"] = "lora";
  doc1["message"] = msgCounter;
  doc1["temperature"] = Temperature;
  doc1["humidity"] = Humidity;
  doc1["pressure"] = Pressure;

  doc1["sensor_value"] = sensorValue;
  if (sensorState == LOW && sensorValue == 0) {
    doc1["sensor_state"] = "error";
    doc1["diepostistda"] = "not-defined";
  }
  else if (sensorState == LOW) {
    doc1["sensor_state"] = "low";
    doc1["diepostistda"] = "true";
  } else {
    doc1["sensor_state"] = "high";
    doc1["diepostistda"] = "false";
  }

  // enable wenn botten
  if ( enableButton == true ) {
    if (sensorState == HIGH) {
      doc1["sensor_state"] = "low";
      doc1["diepostistda"] = "true";
    } else {
      doc1["sensor_state"] = "high";
      doc1["diepostistda"] = "false";
    }
  }

  doc1["vbattery"] = VBAT;
  doc1["wakeup_reason"] = esp_sleep_get_wakeup_cause();
  
  serializeJson(doc, jsonSerial);
  
  //Send LoRa packet to receiver
  Serial.println("LoRa, begin to send packet to Gateway");
  LoRa.beginPacket();
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(byte(msgCounter));         // add message ID
  LoRa.write(LoRaMessage.length());     // add payload length
  LoRa.print(jsonSerial);
  LoRa.endPacket();
  //LoRa.endPacket(true); // true = async / non-blocking mode
  
  Serial.printf("LoRa, send packet done: %d\n", msgCounter);
  Serial.println(jsonSerial);

  // 
  //Watchdog.reset();
  displayReadings();

}


// prepare for sleep
void do_ready_for_sleep() {
  display.dim(true);
  display.clearDisplay();
  LoRa.sleep();
  LoRa.idle(); 
  LoRa.end();                    // set standby mode
  digitalWrite(pwrPin, LOW);    // disable Power US   
}


//function for fetching All readings at once
void getReadings() {
  getDHTreadings();
  if ( enableBMP == true ) {
    getBMPreadings();
  }
  /** Sensor enable one of them **/
  if ( enableButton == true ) {
    getButtonState();     // as digital IO based on Press-Button
  }
  else {
    getSensorUSValue();   // as analog IO
  }
}



void loop() {

  Serial.println("----------------------------------------------------------------------");
  getReadings();
  getVbat();
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
    Serial.printf("Go into deep sleep mode for %i seconds\n", DEEP_SLEEP);
    display.clearDisplay();
    print_sleep_info();
    delay(1 * mS_TO_S_FACTOR); 
    do_ready_for_sleep();
    esp_sleep_enable_timer_wakeup((DEEP_SLEEP -4) * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  } 
  else if (sleepMode == 2) {
    Serial.printf("Go into sleep mode for %i seconds\n", SLEEP);
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
    // not sleep but wait some seconds
    delay((wait_for -9) * mS_TO_S_FACTOR);  // delay
  }

}


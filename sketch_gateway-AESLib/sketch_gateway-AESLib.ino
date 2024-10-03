
/**

This is LoRa Receiver / Gateway Node

Feaures
- OLED SSD1306 I2C Address 0x3C
- WebServer
- Time base is UTC
- Receive LoRa Message and forward to MQTT Server

      MQTT Message
        2024-07-13T07:33:36+0000  : lora/nodea1/data : 
         {"ts":1720856015,"data":[
          {"name":"Test-Node","node":"0xA1","type":"lora","message":349,
           "temperature":15.30000019,"humidity":90,
           "pressure":null,"buttonState":0,"vbattery":4.283311367}]}


**/


// Sensitive configs
#include "secrets.h"   
#include "config.h"

#include "mqtt.h"

// https://github.com/adafruit/Adafruit_SleepyDog/tree/master
#include <Adafruit_SleepyDog.h>
int countdownMS = Watchdog.enable(60 * 1000);

// Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Import Wi-Fi library
#include <WiFi.h>
//#include <ESP8266WiFi.h>   welches ???
//#include "ESPAsyncWebServer.h"

//Libraries for LoRa
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#include <Adafruit_SleepyDog.h>

// Libraries to get time from NTP Server
#include <NTPClient.h>
#include <WiFiUdp.h>

// https://registry.platformio.org/libraries/heman/AsyncMqttClient-esphome
#include <AsyncMqttClient.h>

#include <ArduinoJson.h>
char jsonSerial[250];  // length of JSON
JsonDocument doc;      // to store input
JsonDocument json_val; // contains JSON from Node


// AES
#include "aes.h"

// Receive LoRa packet
//const int ledPin =  25;


// Create objects to handle MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// heartbeat
unsigned long previousMillis = 0;    // Stores last time temperature was published

// Initialize variables to get and save LoRa data
unsigned long loraPacketRecv = 0;   // counter number of messages
String loraRecvFrom;
bool loraHasData = false;

// Initialize variables to get and save Wifi data
//int8_t rssiWiFi;

// Store number of MQTT messages
String publishMsgCount = "0";

// Create AsyncWebServer object on port 80
//AsyncWebServer server(80);

// Define OLED instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Instanziierung
#include "oled.h"

// OLED line 1..6, writePixel (x, y, color)
int displayRow1 = 0;
int displayRow2 = 11;
int displayRow3 = 21;
int displayRow4 = 31;
int displayRow5 = 41;
int displayRow6 = 51;
int displayRow7 = 56;

// Buffer to write message to display
char text[25] = {0};

WiFiClient espClient;



void showVersion()
{
  sprintf(text, "Version %s", String(VERSION));
  oledWriteMsg(0,displayRow1, text);
  oledWriteMsg(0,displayRow2, "LoRa Gateway");
  delay(2 * mS_TO_S_FACTOR); 
}


void setup()
{
  Serial.begin(115200);
  Serial.println("***LoRa Receiver " +String(VERSION));

  // watchdog, used when WiFi not comes up
  Watchdog.disable();
  Watchdog.reset(); // muss, sonst beleibt haengen
  int countdownMS = Watchdog.enable(60 * 1000);
  Serial.print(F("Enable Watchdog with max countdown of "));
  Serial.print(countdownMS, DEC);
  Serial.println(F(" milliseconds!"));

  initOLED();
  showVersion();
  initLoRA();

  // nicht getested
  //mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  //wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));


  WiFi.onEvent(WiFiEvent);

  connectToWifi();
  
  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);


  // Initialize a NTPClient to get time, we use UTC
  timeClient.begin();
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);

  connectToMqtt();

  Watchdog.disable();
  Serial.println(F("All seems Connected, disable Watchdog"));

  // AES
  aes_init();
  //aesLib.set_paddingmode(paddingMode::CMS);

  // initialize green LED to indicate receive packets
  //pinMode(ledPin, OUTPUT);

/**
  // Webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
  });

  // Health Endpoint
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/tech", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<250> data;
    timeClient.update();
    long ts = timeClient.getEpochTime();
    data["ts"] = ts;
    data["name"] = NAME;
    data["firmware_rev"] = VERSION;
    data["Wifi RSSI"] = rssiWiFi;
    data["LoRa Packets received"] = loraPacketRecv;
    data["MQTT Server"] = MQTT_BROKER;
    data["MQTT Status"] = "online";
    data["MQTT Packets published"] = publishMsgCount;

    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });

  server.onNotFound(notFound);
  server.begin();
  **/
}

/**
void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}
**/

// Initialize OLED Module
void initOLED()
{
  Serial.println(F("Initializing OLED Display"));
  //reset OLED display via software
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();
  /**
  display.display(); // zeigt den Grafikpuffer auf dem OLED-Display
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("OLED Display OK!");
  display.display();
  **/
  oledWriteMsg(0,displayRow3, "OLED Display OK");

  Watchdog.reset(); 
 }

// Initialize LoRa Module
void initLoRA()
{
  int counter;
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    counter++;
    delay(500);
  }

  LoRa.enableCrc();

  // so nicht optimal
  if (counter == 10) {
    // Increment readingID on every new reading
    Serial.println(F("Starting LoRa failed!")); 
  }
  Serial.println(F("LoRa Initialization OK!"));

  // display, reset line and write new message
  oledWriteMsg(0,displayRow4, "LoRa Module OK");
  
  // sleep to hold messages on display
  delay(2 * mS_TO_S_FACTOR);

  Watchdog.reset();
}




void connectToWifi() 
{
  Serial.printf("Connecting to WiFi '%s' .", SECRET_WIFI_SSID);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  
  // write new
  display.setCursor(0,10);
  display.print("IP: ");
  display.setCursor(20,10);
  display.print(WiFi.localIP());
  display.display();

  Watchdog.reset();
}


void connectToMqtt()
{
  Serial.printf("\nConnecting to MQTT '%s:%i'...\n", MQTT_BROKER, MQTT_PORT);
  mqttClient.connect();
  // loop until we are connected
  while (!mqttClient.connected() ) {
    Serial.println(mqttClient.connected());
    delay(1 * mS_TO_S_FACTOR); 
  }
  // send online message to MQTT
  uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String("true").c_str());
  Serial.printf("Publishing on topic %s, packetId %i: \n", MQTT_PUB_GW_ONLINE, packetIdPub);
}





void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //  Serial.println("WiFi Started");
    //  break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("\nWiFi connected, WiFi RSSI: %d, IP address: %s\n", WiFi.RSSI(), WiFi.localIP().toString());
      //rssiWiFi = WiFi.RSSI();
      //connectToMqtt();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      Serial.println(F("WiFi lost connection"));
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
    //case SYSTEM_EVENT_STA_DISCONNECTED:
    //  Serial.println("WiFi lost connection");
    //  xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		//  xTimerStart(wifiReconnectTimer, 0);
    //  break;
  }
}


void onMqttConnect(bool sessionPresent)
{
  Serial.print(F("Connected to MQTT, Session present: "));
  Serial.println(sessionPresent);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println(F("Disconnected from MQTT."));

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }

  //if (WiFi.isConnected()) {
  //  mqttReconnectTimer.once(2, connectToMqtt);
  //}

}



void onMqttPublish(uint16_t packetId)
{
  Serial.printf(" Received acknowledged for packetId %s\n", String(packetId));
  publishMsgCount = String(packetId);
}

/**
void getTs() {
  timeClient.update();
  long ts = timeClient.getEpochTime();
  String ts_string = timeClient.getFormattedTime();
  return long(ts), String(ts_string);
}
**/



void update_display()
{
  oledWriteMsg(0,displayRow1, "RUNNING...       OK");
  sprintf(text, "WiFi rssi: %s        ", String(WiFi.RSSI()));
  oledWriteMsg(0,displayRow3, text);
  sprintf(text, "MQTT tx: %s        ", String(publishMsgCount));
  oledWriteMsg(0,displayRow4, text);
  sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv));
  oledWriteMsg(0,displayRow6, text);    
}


// Read LoRa packet and get JSON
void getLoRaData()
{
  Serial.println(F("LoRa packet received: "));
  loraPacketRecv++;

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  loraRecvFrom = String(sender, HEX);

  // if message is for this device, or broadcast, print details:
  Serial.println(" Received from: 0x" + loraRecvFrom);
  Serial.println(" Sent to: 0x" + String(recipient, HEX));
  Serial.println(" Message ID: " + String(incomingMsgId));
  Serial.println(" Message length: " + String(incomingLength));
  //Serial.print(" RSSI: " + String(LoRa.packetRssi()));
  //Serial.println(" / Snr: " + String(LoRa.packetSnr()));

  // Get RSSI / SNR
 // loraRssi = LoRa.packetRssi();
  //loraSnr = LoRa.packetSnr();

  // if the recipient isn't for this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.print(F(" *This message is not for me. Received from "));
    Serial.println(loraRecvFrom)
    //String loraData = LoRa.readString();
    //Serial.print(" Data: ");
    //Serial.println(loraData); 
    
    /**
    if( true ) {
      timeClient.update();
      long ts = timeClient.getEpochTime();
      // add fields from gateway and node-data as json-evelope
      JsonDocument doc; 
      char jsonSerial[500]; 
      doc["ts"] = ts;
      //doc["ts_string"] = ts_string;
      doc["type"] = "message from unknow lora node";
      doc["recipient"] = recipient;
      doc["loraRecvFrom"] = loraRecvFrom;
      doc["incomingMsgId"] = incomingMsgId;
      doc["incomingLengt"] = incomingLength;
      serializeJson(doc, jsonSerial);

      uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_GW_EVENTS, 1, true, jsonSerial);
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: \n", MQTT_PUB_GW_EVENTS, packetIdPub3);
    }
    **/

    ;
    return;               // skip rest of function
  }


  // Read packet
  while (LoRa.available()) {
    String loraData = LoRa.readString();

    /**
    // plain JSON
    if(loraData.indexOf("data") > 0) {
      Serial.printf(" Receive Plain Data: \n ");  // muss so sein
      Serial.println(loraData);
      JsonDocument doc; 
      deserializeJson(doc, loraData);
      json_val = doc["data"];
      loraHasData = true;
    }
    // Error message
    else if(loraData.indexOf("ERROR") > 0) {
    **/
    // Error message
    if(loraData.indexOf("ERROR") > 0) {
      Serial.printf(" ERROR message received from Node: \n ");  // muss so sein
      Serial.println(loraData);
      JsonDocument doc; 
      //doc["ts"] = ts;
      doc["message"] = loraData;
      doc["recipient"] = recipient;
      doc["loraRecvFrom"] = loraRecvFrom;
      doc["incomingMsgId"] = incomingMsgId;
      doc["incomingLengt"] = incomingLength;
      serializeJson(doc, jsonSerial);
      loraHasData = true;
    } 
    // encrypted
    else {
      Serial.printf(" Received Encrypted Data: \n");
      Serial.print("  Ciphertext: "); Serial.println(loraData);
      Serial.print("  Ciphertext length: "); Serial.println(loraData.length());
      Serial.println(loraData.length());

      sprintf(ciphertext, "%s", loraData.c_str());
      byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      String decrypted = decrypt_impl(ciphertext, dec_iv);

      //Serial.println(ciphertext);
      Serial.println(decrypted);
      //JsonDocument doc;
      deserializeJson(doc, decrypted);
      json_val = doc["data"];
      loraHasData = true;
    }

  }

}

int stateLED = LOW;
unsigned long previousLEDmillis = 0;                    // timestamp stores time LED changed                            
void blinkts(int blinkLEDInterval)
{
  unsigned long timeNow = millis();                       //start a millis count
  if (timeNow - previousLEDmillis > blinkLEDInterval) {   // counts up time until set interval
    previousLEDmillis = timeNow;                          // save it as new time (interval reset)
    if (stateLED == LOW)  {                               // check if LED is LOW
      stateLED = HIGH;                                    // then set it HIGH for the next loop
      oledWriteMsg(80,displayRow1, "*");                  // Print "ON" at Serial console
    } else  {      
      stateLED = LOW;                                     // in case LED is HIGH make LED LOW
      oledWriteMsg(80,displayRow1, " ");                  // Print "OFF" at Serial console
    }
  }
}


// main
void loop()
{

  // Check if there are LoRa packets available
  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    //digitalWrite(ledPin, HIGH); 
    oledWriteMsg(100, displayRow6, "***");
    getLoRaData();
    //digitalWrite(ledPin, LOW); 
    oledWriteMsg(100,displayRow6, "   ");
    
    // we had lora data
    if(loraHasData) {

      timeClient.update();
      long ts = timeClient.getEpochTime();
      String ts_string = timeClient.getFormattedTime();
      //Serial.printf("Time:  %i - %s\n", ts, ts_string);
      Serial.print(ts);
      Serial.print(F(" - "));
      Serial.println(ts_string); 
      // Serial.println(timeClient.getFormattedTime());              

      // add fields from gateway and node-data as json-evelope
      doc["ts"] = ts;
      doc["ts_string"] = ts_string;
      doc["lora_rssi"] = String(LoRa.packetRssi());
      doc["lora_snr"] = String(LoRa.packetSnr());
      doc["data"] = json_val;
      serializeJson(doc, jsonSerial);

      loraRecvFrom = "/" + loraRecvFrom;
      String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
      //sprintf(topic, "%s/%s%s", MQTT_PUB_PREFIX, loraRecvFrom, MQTT_PUB_DATA);
      uint16_t packetIdPub = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
      Serial.printf("Publishing on topic %s, packetId: %i: Message %s\n", 
        topic.c_str(), packetIdPub, jsonSerial);
      
      loraHasData = false;
    }

    // update when someting has changed
    update_display();   
 
  }

  // Every X number of seconds, it publishes a new MQTT liveness message
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalMillis) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // send liveness
    mqttClient.publish(MQTT_PUB_GW_LIVENESS, 1, true, String("true").c_str());
  }

  blinkts(2000); //millis
  
}






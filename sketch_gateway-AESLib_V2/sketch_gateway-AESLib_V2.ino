
/**

This is LoRa Receiver / Gateway Node

https://github.com/jgromes/RadioLib/tree/master

Feaures
- OLED SSD1306 I2C Address 0x3C
- LoRa, receive AES encypted packets
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
#include "log.h"

#include "mqtt.h"

//Libraries for Communications
#include <Wire.h>
#include <SPI.h>

// https://github.com/adafruit/Adafruit_SleepyDog/tree/master
#include <Adafruit_SleepyDog.h>
int countdownMS = Watchdog.enable(60 * 1000);

// Libraries for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1351.h>

// Webserver
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"

// WiFi and Radio
#include <RadioLib.h>
#include <WiFi.h>

// to get time from NTP Server
#include <NTPClient.h>
#include <WiFiUdp.h>
//#include "time.h"

// https://registry.platformio.org/libraries/heman/AsyncMqttClient-esphome
#include <AsyncMqttClient.h>

#include <ArduinoJson.h>
char jsonSerial[250];  // length of JSON
JsonDocument doc;      // to store input
JsonDocument json_val; // contains JSON from Node

// AES
#include "aes.h"

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
//String loraRecvFrom;
bool loraHasData = false;

// Initialize variables to get and save Wifi data
int8_t rssiWiFi;

String lastReceivedLoraNode = "";
// store data from lora
String lora_data_str;


// Store number of MQTT messages
String acknowledgedMsgCount = "0";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
 

// OLED line 1..6, writePixel (x, y, color)
int displayRow1 = 0;
int displayRow2 = 11;
int displayRow3 = 21;
int displayRow4 = 31;
int displayRow5 = 41;
int displayRow6 = 51;
int displayRow7 = 56;

// Buffer to write message to serial port
char text[500] = {0};

WiFiClient espClient;

// ESPDateTime: https://blog.mcxiaoke.com/ESPDateTime/index.html
#include <DateTime.h>

// LoRa
//SX1262 radio = new Module(SPI_LORA_SS, SPI_LORA_DIO1, SPI_LORA_RST, SPI_LORA_BUSY);
SX1262 radio = new Module(4, 11, 5, 3);



// flag to indicate that a packet was received
volatile bool receivedFlag = false;


// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
//#if defined(ESP8266) || defined(ESP32)
//  ICACHE_RAM_ATTR
//#endif
void  ICACHE_RAM_ATTR setFlag();
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}


// Define OLED instance
#define OLED_RESET  -1
#define SSD1306_I2C_ADDRESS 0x3c
Adafruit_SSD1306 oled1(SCREEN_1_WIDTH, SCREEN_1_HEIGHT, &Wire, OLED_RESET); // Instanziierung
#include "oled.h"



void showDisplay_Version() {
  sprintf(text, "Version %s", String(VERSION));
  oled1WriteMsg(0,displayRow1, text);
  oled1WriteMsg(0,displayRow2, "LoRa Gateway");
  delay(2 * mS_TO_S_FACTOR); 
}

#include "esp_heap_caps.h"

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name) {
  printf("%s was called but failed to allocate %d bytes with 0x%X capabilities. \n",function_name, requested_size, caps);
}

void setup() {
  Serial.begin(BAUD);
  // Init Logger
  isDebugEnabled();

  debugOutput("***LoRa Receiver - Version " +String(VERSION), 5);

  // Set the I2C pins before begin
  Wire.setPins(I2C_SDA, I2C_SCL); 
  Wire.begin(); // join i2c bus (address optional for master)

  //SPI default pins for Board ESP32-C6-N8
  SPI.begin(SCK, MISO, MOSI, SS);

  // Green LED to indicate a packets received
  pinMode(LED_PIN, OUTPUT);

  // watchdog, used when WiFi not comes up
  Watchdog.disable();
  Watchdog.reset(); // muss, sonst beleibt haengen

  int countdownMS = Watchdog.enable(60 * 1000);
  debugOutput("Enable Watchdog with max countdown of " +String(countdownMS, DEC)+ " milliseconds", 5);


  initOLED1();
  //initOLED2();  // vertraget sich so nicht mit SX1262
  showDisplay_Version();

  sprintf(text, "oled done ... "); oled1WriteMsg(2, displayRow6, text); 

  initLoRA();
  /**
  debugOutputP1("[SX1262] Initializing... ", 4);
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    debugOutputP2("  success!", 4);
  } else {
    debugOutputP2("  failed, code " +String(state), 5);
    while (true) { delay(10); }
  }
  // set carrier frequency to 433.5 MHz
  if (radio.setFrequency(866.6) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    debugOutput("[SX1262] Selected frequency is invalid for this module!", 5);
    while (true) { delay(10); }
  }

  // set the function that will be called when new packet is received
  radio.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  debugOutputP1("[SX1262] Starting to listen ... ", 4);
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    debugOutputP2("   success!", 4);
  } else {
    debugOutputP2("   failed, code " +String(state), 5);
    while (true) { delay(10); }
  }
  **/
  sprintf(text, "lora done"); oled1WriteMsg(2, displayRow6, text);

  
  // nicht getested
  //mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  //wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));


  WiFi.onEvent(WiFiEvent);
  connectToWifi();
  sprintf(text, "wifi done ... "); oled1WriteMsg(2, displayRow6, text);
  
  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);


  // Initialize a NTPClient to get time, we use UTC
  timeClient.begin();

  setupDateTime();
  //showTime();
  sprintf(text, "time done ... "); oled1WriteMsg(2, displayRow6, text);
   

  // mqtt
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);

  connectToMqtt();
  sprintf(text, "mqtt done ... "); oled1WriteMsg(2, displayRow6, text);
  

  Watchdog.disable();
  debugOutput("All seems Connected, disable Watchdog", 5);

  Debug.println(F("------"));

  // AES
  aes_init();
  //aesLib.set_paddingmode(paddingMode::CMS);
  sprintf(text, "aes done ... "); oled1WriteMsg(2, displayRow6, text);

  webserver();
  sprintf(text, "web done "); oled1WriteMsg(2, displayRow6, text);

/**
  oled2.fillScreen(OLED_Backround_Color);
  oled2.setTextColor(OLED_Text_Color);
  oled2.setTextSize(1);
  oled2.setCursor(0,700);
  oled2.print("OLED_Text_Color");
**/

  Debug.setDebugLevel(1);
  Debug.println(F("Debug 5 is on again"));

  // clear
  sprintf(text, "            ");
  oled1WriteMsg(0,displayRow3, text);

  sprintf(text, "finished  "); oled1WriteMsg(2, displayRow6, text);  

}



//screen 1
const unsigned char PROGMEM screen1 [] = {
 
};




// initialize OLED SSD1306 with the I2C addr 0x3D (for the 128x64)
// bool:reset set to TRUE or FALSE depending on you display
void initOLED1() {
  debugOutput("[SSD1306] Initializing...", 5);
  if (!oled1.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, true)) { 
    debugOutput("[SSD1306]* allocation failed", 2);
    for (;;)
      ; // Don't proceed, loop forever
  }
  // Clear the buffer.
  oled1.display();
  delay(20); //omit delay to hide adafruit slashscreen, sorry!
  oled1.clearDisplay();
  oled1.display();

  testdrawline();
 
 /**
  // display splashscreen bitmap display
  oled1.drawBitmap(5, 5,  screen1, 120, 56, 1);
  oled1.display();
  delay(2000);
  oled1.clearDisplay();
  oled1.display();

  oled1.setTextColor(BLACK, BLACK);
  oled1.clearDisplay();
  oled1.display();
  **/

  sprintf(text, "OLED  OK");
  oled1WriteMsg(0,displayRow3, text);

  Watchdog.reset(); 
 }


void testdrawline() {
  int16_t i;
  oled1.clearDisplay(); // Clear display buffer
  for(i=0; i<oled1.width(); i+=4) {
    oled1.drawLine(0, 0, i, oled1.height()-1, WHITE);
    oled1.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<oled1.height(); i+=4) {
    oled1.drawLine(0, 0, oled1.width()-1, i, WHITE);
    oled1.display();
    delay(1);
  }
  oled1.clearDisplay();
}


// Initialize LoRa Module SX1262
void initLoRA()
{
  debugOutputP1("[SX1262] Initializing... ", 4);
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    debugOutputP2("  success!", 4);
  } else {
    debugOutputP2("  failed, code " +String(state), 5);
    while (true) { delay(10); }
  }

  // set carrier frequency to 433.5 MHz
  if (radio.setFrequency(866.6) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    debugOutput("[SX1262] Selected frequency is invalid for this module!", 5);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  debugOutputP1("[SX1262] Starting to listen ... ", 4);
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    debugOutputP2("   success!", 4);
  } else {
    debugOutputP2("   failed, code " +String(state), 5);
    while (true) { delay(10); }
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.scanChannel();

}






/**
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
**/



int wifi_status = WL_IDLE_STATUS;

void connectToWifi() 
{
  debugOutput("Connecting to WiFi " +String(SECRET_WIFI_SSID), 5);

  while (WiFi.status() != WL_CONNECTED) {
    debugOutput("Attemping to connect...", 5);
    wifi_status = WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
    delay(10000);
  }

  Serial.println(wifi_status);

  /**
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("wifi."));
  }
  **/
  int state = WiFi.status();
  Serial.println(state);
  
  Watchdog.reset();


}


void showDisplay_WifiIp()
{
  oled1.setCursor(0,10);
  oled1.print("IP: ");
  oled1.setCursor(20,10);
  oled1.print(WiFi.localIP());
  oled1.display();
}

void WiFiEvent(WiFiEvent_t event)
{
  debugOutput("[WiFi-event] event: " +String(event), 5);
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //  Serial.println("WiFi Started");
    //  break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      debugOutput("Wifi connected, RSSI: " +String(WiFi.RSSI())+ ", IP address: " +String(WiFi.localIP().toString()), 4);
      showDisplay_WifiIp();
      //rssiWiFi = WiFi.RSSI();
      //connectToMqtt();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      debugOutput("Wifi lost connection!", 3);
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


void connectToMqtt()
{
  debugOutput("Connecting to MQTT " +String(MQTT_BROKER)+ ":" +String(MQTT_PORT), 5);
  mqttClient.connect();
  // loop until we are connected
  while (!mqttClient.connected() ) {
    Serial.println(mqttClient.connected());
    delay(1 * mS_TO_S_FACTOR); 
  }
  // send online message to MQTT
  uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String("true").c_str());
  debugOutput("Publishing on topic " +String(MQTT_PUB_GW_ONLINE)+ "PacketId " +String(packetIdPub), 5);
}


void onMqttConnect(bool sessionPresent) {
  debugOutput("Connected to MQTT, Session present: " +String(sessionPresent), 4);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  debugOutput("Disconnected from MQTT!", 4);

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }

  //if (WiFi.isConnected()) {
  //  mqttReconnectTimer.once(2, connectToMqtt);
  //}

}


void onMqttPublish(uint16_t packetId) {
  debugOutput(" Received acknowledged for packetId " +String(packetId), 4);
  acknowledgedMsgCount = String(packetId);
}
// END mqtt

/**
void getTs() {
  timeClient.update();
  long ts = timeClient.getEpochTime();
  String ts_string = timeClient.getFormattedTime();
  return long(ts), String(ts_string);
}
**/



void webserver() {  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
  });

  // Health Endpoint
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/tech", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument data;
    //timeClient.update();
    //long ts = timeClient.getEpochTime();

    long ts = DateTime.now();
    String formattedDate = DateTime.toISOString().c_str();
  
    data["ts"] = ts;
    data["DateTimeUTC"] = formattedDate;
    data["uptime"] = esp_timer_get_time() / 1000000;
    data["name"] = NAME;
    data["firmware_rev"] = VERSION;
    data["wifi_rssi"] = String(WiFi.RSSI());

    // Get the current uptime in milliseconds
    uint64_t uptime_ms = esp_timer_get_time() / 1000;
    // Display the uptime in seconds
    ESP_LOGI("main", "Uptime: %llu seconds", uptime_ms / 1000);
    
    JsonDocument doc_lora;
    doc_lora["Packets received"] = loraPacketRecv;
    doc_lora["Node last seen"] = lastReceivedLoraNode;
    data["lora"] = doc_lora;

    // nested
    JsonDocument doc_mqtt; 
    doc_mqtt["Server"] = MQTT_BROKER;
    doc_mqtt["Status"] = "online";
    doc_mqtt["Packets send"] = "0";    
    doc_mqtt["Packets acknowledged"] = acknowledgedMsgCount;
    data["mqtt"] = doc_mqtt;

    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });

  server.onNotFound(notFound);
  server.begin();
}


void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}
// END webserver


void update_display()
{
  oled1WriteMsg(0,displayRow1, "RUNNING...       OK");
  sprintf(text, "WiFi rssi: %s        ", String(WiFi.RSSI()));
  oled1WriteMsg(0,displayRow3, text);
  sprintf(text, "MQTT tx: %s        ", String(acknowledgedMsgCount));
  oled1WriteMsg(0,displayRow4, text);
  sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv));
  oled1WriteMsg(0,displayRow6, text);    
}

/**
// Read LoRa packet and get JSON
void getLoRaDataX()
{
  debugOutput("LoRa packet received:" , 4);
  loraPacketRecv++;

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  loraRecvFrom = String(sender, HEX);

  // if message is for this device, or broadcast, print details:
  debugOutput(" Received from: 0x" +String(loraRecvFrom), 5);
  debugOutput(" Sent to: 0x" +String(recipient, HEX), 5);
  debugOutput(" Message ID: " +String(incomingMsgId), 5);
  debugOutput(" Message length: " +String(incomingLength), 5);
  //Serial.print(" RSSI: " + String(LoRa.packetRssi()));
  //Serial.println(" / Snr: " + String(LoRa.packetSnr()));

  // Get RSSI / SNR
 // loraRssi = LoRa.packetRssi();
  //loraSnr = LoRa.packetSnr();

  // if the recipient isn't for this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    debugOutput(" *This message is not for me. Received from " +String(loraRecvFrom), 3);
    String loraData = LoRa.readString();
    Serial.print(" Data: ");
    Serial.println(loraData); 
    
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
   
    ;
   // return;               // skip rest of function
   
  }
**/
/**
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
  /**  if(loraData.indexOf("ERROR") > 0) {
      debugOutput(" ERROR message received from Node: " +String(loraData), 3);
      JsonDocument doc; 
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
      debugOutput(" Received Encrypted Data:", 5);
      debugOutput("  Ciphertext: " +String(loraData), 5);
      debugOutput("  Ciphertext length: " +String(loraData.length()), 5);

      sprintf(ciphertext, "%s", loraData.c_str());
      byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      String decrypted = decrypt_impl(ciphertext, dec_iv);

      debugOutput(String(decrypted), 5);

      JsonDocument doc2;
      deserializeJson(doc2, decrypted);
      json_val = doc2;

      loraHasData = true;
    }

  }

}
**/



void processLoRaData(){

    // plain JSON
  JsonDocument doc;   
  if(lora_data_str.indexOf("data") > 0) {
      debugOutput(" Received Plain Data:", 5);
      debugOutput("  Plaintext: " +String(lora_data_str), 5);
      debugOutput("  Plaintext length: " +String(lora_data_str.length()), 5);
      //JsonDocument doc; 
      deserializeJson(doc, lora_data_str);
      json_val = doc["data"];
      loraHasData = true;
    }
    // Error message
    else if(lora_data_str.indexOf("ERROR") > 0) {
      debugOutput(" ERROR message received from Node: " +String(lora_data_str), 3);
      debugOutput("  Message: " +String(lora_data_str), 5);
      debugOutput("  Message length: " +String(lora_data_str.length()), 5);
      //JsonDocument doc; 
      deserializeJson(doc, lora_data_str);
      json_val = doc["data"];
      loraHasData = true;
    } 
    
    // encrypted
    else {
      debugOutput(" Received Encrypted Data:", 5);
      debugOutput("  Ciphertext: " +String(lora_data_str), 5);
      debugOutput("  Ciphertext length: " +String(lora_data_str.length()), 5);

      sprintf(ciphertext, "%s", lora_data_str.c_str());
      byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      String decrypted = decrypt_impl(ciphertext, dec_iv);

      debugOutput(String(decrypted), 5);

      JsonDocument doc2;
      deserializeJson(doc2, decrypted);
      json_val = doc2;

      loraHasData = true;
    }

    // was ist wenn nicht lesbar try/except

}





// 
int stateLED = LOW;
unsigned long previousLEDmillis = 0;                      // timestamp stores time LED changed                            
void blinkts(int blinkLEDInterval)
{
  unsigned long timeNow = millis();                       // start a millis count
  if (timeNow - previousLEDmillis > blinkLEDInterval) {   // counts up time until set interval
    previousLEDmillis = timeNow;                          // save it as new time (interval reset)
    if (stateLED == LOW)  {                               // check if LED is LOW
      stateLED = HIGH; 
      sprintf(text, "*   ");                                   // then set it HIGH for the next loop
      oled1WriteMsg(100,displayRow1, text);              // Print "ON" at Serial console
    } else  {      
      stateLED = LOW;  
      sprintf(text, "    ") ;                                   // in case LED is HIGH make LED LOW
      oled1WriteMsg(100,displayRow1, text);              // Print "OFF" at Serial console
    }
  }
}






// main
void loop()
{
  // check if the flag is set
  if(receivedFlag) {

    debugOutput("[SX1262] receivedFlag = true", 5);
    //digitalWrite(ledPin, HIGH);
    sprintf(text, "*"); oled1WriteMsg(100, displayRow6, text);
    
    // packet was received
    loraPacketRecv++;
    debugOutput("[SX1262] Received packet " +String(loraPacketRecv), 4);
    sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv));
    oled1WriteMsg(0,displayRow6, text);

    // aktiv wenn keine Daten kommen, miss 2 15 min samples
//    int countdownMS = Watchdog.enable(2000 * 1000);
 
     int state = radio.readData(lora_data_str);

  	if (state == RADIOLIB_ERR_NONE) {
      // reset flag
      receivedFlag = false;

      debugOutput("[SX1262] RSSI:\t" +String(radio.getRSSI())+ " dBm",5);
      debugOutput("[SX1262] SNR:\t" +String(radio.getSNR())+ " dB",5);
      debugOutput("[SX1262] Data:\t" +String(lora_data_str),5);

      // processLoRaData(lora_data_str);
    }
  
/**
    // we had lora data
    if(loraHasData) {

     // timeClient.update();
     // String formattedDate = timeClient.getFormattedTime();

/**
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  
  // Get epochtime
  long ts = timeClient.getEpochTime();
  
  /**
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  String formattedDate = timeClient.getFormattedTime();
  Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  Serial.print("DATE: ");
  Serial.println(dayStamp);
  // Extract time
  String timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  Serial.print("HOUR: ");
  Serial.println(timeStamp);


  getDateTime();

    if (!DateTime.isTimeValid()) {
      Serial.println("Failed to get time from server, retry.");
      DateTime.begin();
    } else {
      showTime();

  **/

  
  /** EXCLUDE FOR DEBUG
      String formattedDate = DateTime.toISOString().c_str();
      long ts = DateTime.now();
      debugOutput("TS: " +String(ts)+ " - " +String(formattedDate), 5);

      // add fields from gateway
      doc["ts"] = ts;
      doc["ts_string"] = formattedDate;
      doc["lora_rssi"] = String(radio.getRSSI());
      doc["lora_snr"] = String(radio.getSNR());


      // get 
      //String substring = str.substring(Math.max(str.length() - 2, 0));
      String loraRecvFrom = json_val["node"];
      int x = loraRecvFrom.length();
      String str = loraRecvFrom.substring(2,4);
      
      Serial.println(loraRecvFrom);
      Serial.println(str);
      str.toLowerCase();
      loraRecvFrom = str;

      lastReceivedLoraNode = loraRecvFrom;

      // add data from LoRa
      doc["data"] = json_val;

      serializeJson(doc, jsonSerial);

      loraRecvFrom = "/" + loraRecvFrom;
      String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
      uint16_t packetIdPub = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
      debugOutput("Publishing on topic " +String(topic)+ " PacketId: " +String(packetIdPub)+ " Message: " +String(jsonSerial), 5); 
      
      loraHasData = false;
  
      Watchdog.disable();
  
    }
    
  
    

    // update when someting has changed
    update_display();   
 
    }

  **/   // EXCLUDE for DEBUG

    // START DEBUG, ab hier nur for debugging
    sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv));
    oled1WriteMsg(0,displayRow6, text);
    // END DEBUG    
    
    // lora received and processed, so we disable
    //Watchdog.disable();

    //delay(1000);

    //digitalWrite(ledPin, LOW);
    sprintf(text, " "); oled1WriteMsg(100, displayRow6, text);

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



void setupDateTime() {
  // setup this after wifi connected
  // you can use custom timeZone,server and timeout
  // DateTime.setTimeZone(-4);
  //   DateTime.begin(15 * 1000);
  DateTime.setServer("ch.pool.ntp.org");
  //DateTime.setTimeZone("CST-8");
  DateTime.begin();
  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  } else {
    Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
    Serial.printf("Timestamp is %ld\n", DateTime.now());
  }
}

void showTime() {
  Serial.printf("TimeZone:      %s\n", DateTime.getTimeZone());
  Serial.printf("Up     Time:   %lu seconds\n", millis() / 1000);
  Serial.printf("Boot   Time:   %ld seconds\n", DateTime.getBootTime());
  Serial.printf("Cur    Time:   %ld seconds\n", DateTime.getBootTime() + millis() / 1000);
  Serial.printf("Now    Time:   %ld\n", DateTime.now());
  Serial.printf("OS     Time:   %ld\n", DateTime.osTime());
  Serial.printf("NTP    Time:   %ld\n", DateTime.ntpTime(2 * 1000L));
  Serial.println("===========");
  Serial.printf("Local  Time:   %s\n", DateTime.format(DateFormatter::SIMPLE).c_str());
  Serial.printf("ISO86  Time:   %s\n", DateTime.toISOString().c_str());
  Serial.printf("UTC    Time:   %s\n", DateTime.formatUTC(DateFormatter::SIMPLE).c_str());
  Serial.printf("UTC86  Time:   %s\n", DateTime.formatUTC(DateFormatter::ISO8601).c_str());
  Serial.println("===========");
  time_t t = time(NULL);
  Serial.printf("OS local:     %s", asctime(localtime(&t)));
  Serial.printf("OS UTC:       %s", asctime(gmtime(&t)));
}






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

//#include <Ticker.h>

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
#include "ESPAsyncWebServer.h"

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
#if ASYNC_TCP_SSL_ENABLED
#define MQTT_SECURE true
#define MQTT_SERVER_FINGERPRINT {0x7e, 0x36, 0x22, 0x01, 0xf9, 0x7e, 0x99, 0x2f, 0xc5, 0xdb, 0x3d, 0xbe, 0xac, 0x48, 0x67, 0x5b, 0x5d, 0x47, 0x94, 0xd2}
#define MQTT_PORT 8883
#else
#define MQTT_PORT 1883
#endif

#include <ArduinoJson.h>
char jsonSerial[500];  // length of JSON
JsonDocument doc;      // to store input
JsonDocument json_val; // 


// Create objects to handle MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// heartbeat
unsigned long previousMillis = 0;    // Stores last time temperature was published

// Variables to save date and time
String epochtime;

// Initialize variables to get and save LoRa data
unsigned long loraPacketRecv = 0;   // counter number of messages
String loraRecvFrom;
String DATA;
String loraRssi;
String loraSnr;

// Initialize variables to get and save Wifi data
String rssiWiFi;

// Store number of MQTT messages
String publishMsgCount = "0";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Define OLED instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Instanziierung

// OLED line 1..6, writePixel (x, y, color)
int disPos_y0 = 0;
int disPos_y1 = 11;
int disPos_y2 = 19;
int disPos_y3 = 28;

int disPos_y4 = 38;
int disPos_y5 = 47;
int disPos_y6 = 56;

WiFiClient espClient;


const char* PARAM_MESSAGE = "message";



void showVersion(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Version ");
  display.println(VERSION);
  display.setCursor(0,10);
  display.print("LoRa Gateway");
  display.display();
  delay(2 * mS_TO_S_FACTOR); 
}


void setup() {
  Serial.begin(115200);
  Serial.println("***LoRa Receiver " +String(VERSION));

  // enable watchdog, used when WiFi not comes up
  int countdownMS = Watchdog.enable(60 * 1000);
  Serial.print("Enable Watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");

  initOLED();
  showVersion();
  initLoRA();

  // nicht getested
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));


  WiFi.onEvent(WiFiEvent);

  connectToWifi();
  
  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);


  // Initialize a NTPClient to get time, we use UTC
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  //timeClient.setTimeOffset(3600);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);

  #if ASYNC_TCP_SSL_ENABLED
    mqttClient.setSecure(MQTT_SECURE);
    if (MQTT_SECURE) {
      mqttClient.addServerFingerprint((const uint8_t[])MQTT_SERVER_FINGERPRINT);
    }
  #endif

  connectToMqtt();

  Watchdog.disable();
  Serial.println("All seems Connected, disable Watchdog");

  // Webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
  });

  // Health Endpoint
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"status\":\"up\"}");
  });

  server.on("/tech", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<500> data;
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

  // TEST, no function
  server.on("/get-message", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<100> data;
    if (request->hasParam("message")) {
      data["message"] = request->getParam("message")->value();
    }
    else {
      data["message"] = "No message parameter";
    }
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });

  // TEST, no function, send a POST request to <IP>/post with a form field message set to <message>
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam(PARAM_MESSAGE, true)) {
      message = request->getParam(PARAM_MESSAGE, true)->value();
    } else {
       message = "No message sent";
    }
        request->send(200, "text/plain", "Hello, POST: " + message);
  });

  server.onNotFound(notFound);
  server.begin();

}

// Initialize OLED Module
void initOLED(){
  Serial.println("Initializing OLED Display");
  //reset OLED display via software
	//display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display(); // zeigt den Grafikpuffer auf dem OLED-Display
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("OLED Display OK!");
  display.display();
  Watchdog.reset(); 
 }

// Initialize LoRa Module
void initLoRA() {
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
    Serial.println("Starting LoRa failed!"); 
  }
  Serial.println("LoRa Initialization OK!");

  display.setCursor(0,30);
  // reset line
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  //display.print("LoRa Initializing OK!");
  display.setCursor(0,30);
  display.print("LoRa receiver: OK");
  display.display();
  delay(2000);
  Watchdog.reset();
}


void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}


// Read LoRa packet and get JSON
void getLoRaData() {
  Serial.println("Lora packet received: ");
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
  Serial.println(" RSSI: " + String(LoRa.packetRssi()));
  Serial.println(" Snr: " + String(LoRa.packetSnr()));
  //Serial.println();

  // Get RSSI / SNR
  loraRssi = LoRa.packetRssi();
  loraSnr = LoRa.packetSnr();

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println(" *This message is not for me.");
    DATA = LoRa.readString();
    Serial.print("*LoRa Data: ");
    Serial.print(DATA); 
    ;
    return;               // skip rest of function
  }

  display.setCursor(0,disPos_y5);
  // reset line
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,disPos_y5);
  display.printf("LoRa recv: %s  ", String(loraPacketRecv));

  //display.setCursor(0,disPos_y6);
  // reset line
  //display.setTextColor(BLACK, BLACK);
  //display.println("                               "); 
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,disPos_y6);
  display.printf("LoRa RSSI: %s  ", String(loraRssi));
  display.display();

  // Read packet
  while (LoRa.available()) {
    DATA = LoRa.readString();
    Serial.print("*LoRa Data: ");
    Serial.print(DATA); 
    //Serial.print(" with RSSI ");    
    //Serial.println(loraRssi);

    if(DATA.indexOf("data") > 0) {
      JsonDocument doc;
      deserializeJson(doc, DATA);
      json_val = doc["data"];
    } 

  }

}


void connectToWifi() {
  Serial.printf("Connecting to WiFi '%s' .", SECRET_WIFI_SSID);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  display.setCursor(0,10);
  // reset line
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,10);
  display.print("IP: ");
  display.setCursor(20,10);
  display.print(WiFi.localIP());
  display.display();
  Watchdog.reset();
}


void connectToMqtt() {
  Serial.printf("\nConnecting to MQTT '%s:%i'...\n", MQTT_BROKER, MQTT_PORT);
  mqttClient.connect();
  // loop until we are connected
  while (!mqttClient.connected() ) {
    Serial.println(mqttClient.connected());
    delay(1 * mS_TO_S_FACTOR); 
  }
  // send online message to MQTT
  Serial.printf("MQTT connected: %i, send online message\n", mqttClient.connected());
  uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, String("true").c_str());
  Serial.printf("Publishing on topic %s at QoS 1, packetId %i: \n", MQTT_PUB_GW_ONLINE, packetIdPub3);
}


void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //  Serial.println("WiFi Started");
    //  break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("\nWiFi connected, WiFi RSSI: %d, IP address: %s\n", WiFi.RSSI(), WiFi.localIP().toString());
      //Serial.printf(", WiFi RSSI: %d\r\n", WiFi.RSSI());
      rssiWiFi = WiFi.RSSI();
      //connectToMqtt();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
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


void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT, Session present: ");
  Serial.println(sessionPresent);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (reason == AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT) {
    Serial.println("Bad server fingerprint.");
  }
  
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }

  //if (WiFi.isConnected()) {
  //  mqttReconnectTimer.once(2, connectToMqtt);
  //}

}



void onMqttPublish(uint16_t packetId) {
  Serial.printf(" Received acknowledged for packetId %s\n", String(packetId));
  publishMsgCount = String(packetId);
}



void update_display() {

  display.setCursor(0,0);
  // reset line
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  //  display.setTextColor(WHITE);  
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,0);
  display.print("Running... Status: OK");   
  display.display();   
  
  // reset line
  display.setCursor(0, disPos_y2);
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  //  display.setTextColor(WHITE);  
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,disPos_y2);
  display.print("WiFi RSSI: ");   
  display.setCursor(70,disPos_y2);
  display.print(rssiWiFi);      

  // reset line
  display.setCursor(0, disPos_y3);
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  //  display.setTextColor(WHITE);  
  display.setTextColor(WHITE, BLACK);
  // write new
  display.setCursor(0, disPos_y3);
  display.printf("MQTT send: %s", publishMsgCount);
  display.display();
}


void loop() {

  // Check if there are LoRa packets available
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    getLoRaData();
    
    // we had data
    if(DATA.indexOf("data") > 0) {

      timeClient.update();
      long ts = timeClient.getEpochTime();
      String ts_string = timeClient.getFormattedTime();
      //Serial.printf("Time:  %i - %s\n", ts, ts_string);
      Serial.print(ts);
      Serial.print(" - ");
      Serial.print(ts_string); 
      // Serial.println(timeClient.getFormattedTime());              

      // add fields from gateway and node-data as json-evelope
      doc["ts"] = ts;
      doc["ts_string"] = ts_string;
      doc["lora_rssi"] = loraRssi;
      doc["lora_snr"] = loraSnr;
      doc["data"] = json_val;
      serializeJson(doc, jsonSerial);

      loraRecvFrom = "/" + loraRecvFrom;
      String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
      uint16_t packetIdPub0 = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i: Message %s\n", 
        topic.c_str(), packetIdPub0, jsonSerial);
    
    }
 
  }

  // Every X number of seconds, it publishes a new MQTT liveness message
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalMillis) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // send liveness plain text
    {
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_GW_LIVENESS, 1, true, String("true").c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i\n", MQTT_PUB_GW_LIVENESS, packetIdPub1);
    }

    // send WiFi RSSI as JSON
    {
      timeClient.update();
      long ts = timeClient.getEpochTime(); 
      doc["ts"] = ts;
      doc["value"] = rssiWiFi;
      doc["units"] = "dbm";
      serializeJson(doc, jsonSerial);
      String topic = MQTT_PUB_GW_RSSI;
      uint16_t packetIdPub2 = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: Message: %s\n", 
         topic.c_str(), packetIdPub2, jsonSerial);
    }

  }
  update_display();   
}






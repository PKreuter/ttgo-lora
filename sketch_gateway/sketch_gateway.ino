
/**

 This is LoRa Receiver / Gateway Code - (WebServer)

OLED SSD1306 I2C Address 0x3C

10  - Running
20
30
40   LoRa received
50   MQTT message send


Sensitive config needs to stored in file 'secrets.h'

// Network credentials
const char* SECRET_WIFI_SSID = "ssidname";      
const char* SECRET_WIFI_PASSWORD = "password"; 

// MQTT credentials
const char* SECRET_MQTT_USERNAME = "username";
const char* SECRET_MQTT_PASSWORD = "password";

*/


//Sensitive configs
#include "secrets.h"   // do store secrets, needs to edit before using

#include "config.h"
#include "mqtt.h"

#include <Ticker.h>

// Import Wi-Fi library
#include <WiFi.h>
#include "ESPAsyncWebServer.h"

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Libraries to get time from NTP Server
#include <NTPClient.h>
#include <WiFiUdp.h>

// MQTT - https://registry.platformio.org/libraries/heman/AsyncMqttClient-esphome
#include <AsyncMqttClient.h>

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


unsigned long loraPacketRecv = 0;   // counter number of messages
byte localAddress = 0xAA;           // address of this device

// Variables to save date and time
String formattedDate;
String timestamp;
String epochtime;

// Initialize variables to get and save LoRa data
String loraRecvFrom;
String DATA;

// LoRa
String loraRssi;
String loraSnr;

// Wifi data
String rssiWiFi;

// store number of MQTT messages
String publishMsgCount = "0";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// define OLED instance
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

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}


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


//------------------Initialize OLED Module-------------------------------------------//
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
 }


//----------------------Initialize LoRa Module-----------------------------------------//
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
}


//-----------------Read LoRa packet and get the sensor readings-----------------------//
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
  Serial.println("Received from: 0x" + loraRecvFrom);

  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  //Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();


  // Get 
  loraRssi = LoRa.packetRssi();
  loraSnr = LoRa.packetSnr();

 // if the recipient isn't this device or broadcast,
 if (recipient != localAddress && recipient != 0xFF) {
  Serial.println("This message is not for me.");
   ;
    return;               // skip rest of function
  }

  display.setCursor(0,disPos_y5);
  // reset line
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  //  display.setTextColor(WHITE);  
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,disPos_y5);
  display.printf("LoRa recv: %s", String(loraPacketRecv));

  // Read packet
  while (LoRa.available()) {

    DATA = LoRa.readString();
    Serial.print("LoRa Data: ");
    Serial.print(DATA); 
    Serial.print(" with RSSI ");    
    Serial.println(loraRssi);

    if(DATA.indexOf("data") > 0) {
      // Serial.print("is JSON: ");
      // Serial.println(DATA);
      JsonDocument doc;
      deserializeJson(doc, DATA);
      json_val = doc["data"];
    } 



  }

  display.setCursor(0,disPos_y6);
  // reset line
  display.setTextColor(BLACK, BLACK);
  display.println("                             "); 
  //  display.setTextColor(WHITE);  
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,disPos_y6);
  display.printf("LoRa RSSI: %s", String(loraRssi));
  display.display();

}

void connectToWifi() {
  //Serial.printf("Connecting to WiFi '%s' ...\n", WIFI_SSID);
  Serial.printf("Connecting to WiFi '%s' .", SECRET_WIFI_SSID);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString());
  Serial.printf("Connected, IP address: %s\n", WiFi.localIP().toString());
  
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
}


void connectToMqtt() {
  Serial.printf("Connecting to MQTT '%s:%i' ...\n", MQTT_BROKER, MQTT_PORT);
  mqttClient.connect();
}


void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //    Serial.println("WiFi Started");
    //    break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString());
      Serial.printf("WiFi RSSI: %d\r\n", WiFi.RSSI());
      rssiWiFi = WiFi.RSSI();
      //connectToMqtt();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
    //case SYSTEM_EVENT_STA_DISCONNECTED:
    //    Serial.println("WiFi lost connection");
    //    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		//    xTimerStart(wifiReconnectTimer, 0);
    //    break;
  }
}



void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT, Session present: ");
  Serial.println(sessionPresent);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttPublish(uint16_t packetId) {
  Serial.printf(" Received acknowledged for packetId %s\n", String(packetId));
  publishMsgCount = String(packetId);

}

void setup() {
  Serial.begin(115200);
  Serial.println("***LoRa Receiver " +String(VERSION));

  initOLED();
  showVersion();
  initLoRA();

  WiFi.onEvent(WiFiEvent);

  connectToWifi();
  
 // wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
 // wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  //timeClient.setTimeOffset(3600);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);
  
  connectToMqtt();

  uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_GW_ONLINE, 1, true, "true");
  Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_GW_ONLINE, packetIdPub3);



    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });


server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "application/json", "{\"status\":\"up\"}");
});


server.on("/tech", HTTP_GET, [](AsyncWebServerRequest *request) {
  StaticJsonDocument<500> data;
  timeClient.update();
  long ts = timeClient.getEpochTime(); 
  data["ts"] = ts;
  data["Wifi RSSI"] = rssiWiFi;
  data["LoRa Packets received"] = loraPacketRecv;
  data["MQTT Server"] = MQTT_BROKER;
  data["MQTT Status"] = "online";
  data["MQTT Packets published"] = publishMsgCount;

  String response;
  serializeJson(data, response);
  request->send(200, "application/json", response);
});


server.on("/get-message", HTTP_GET, [](AsyncWebServerRequest *request) {
  StaticJsonDocument<100> data;
  if (request->hasParam("message"))
  {
    data["message"] = request->getParam("message")->value();
  }
  else {
    data["message"] = "No message parameter";
  }
  String response;
  serializeJson(data, response);
  request->send(200, "application/json", response);
});


    // Send a POST request to <IP>/post with a form field message set to <message>
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
    //displayReadings();

    timeClient.update();

    Serial.println(timeClient.getFormattedTime());              
    Serial.println(timeClient.getEpochTime());  

    long ts = timeClient.getEpochTime();  

    if(DATA.indexOf("data") > 0) {

      /** Publish MQTT messages
        2024-07-13T07:33:36+0000  : lora/nodea1/data : 
         {"ts":1720856015,"data":[
          {"name":"Test-Node","node":"0xA1","type":"lora","message":349,
           "temperature":15.30000019,"humidity":90,
           "pressure":null,"buttonState":0,"vbattery":4.283311367}]}
      */
      JsonDocument doc;
      char jsonSerial[500];
      doc["ts"] = ts;
      doc["lora_rssi"] = loraRssi;
      doc["lora_snr"] = loraSnr;

      doc["data"] = json_val;
      serializeJson(doc, jsonSerial);
      loraRecvFrom = "/" + loraRecvFrom;
      String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
      //String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
      uint16_t packetIdPub0 = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i: Message %s\n", 
        topic.c_str(), packetIdPub0, jsonSerial);
    }
 


  }

  unsigned long currentMillis = millis();
  // Every X number of seconds, it publishes a new MQTT liveness message
  if (currentMillis - previousMillis >= intervalMillis) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // send heartbeat
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_GW_LIVENESS, 1, true, String("true").c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i\n", MQTT_PUB_GW_LIVENESS, packetIdPub1);
  
    // send WiFi RSSI
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






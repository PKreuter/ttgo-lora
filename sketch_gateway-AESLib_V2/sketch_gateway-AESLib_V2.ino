
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


#include "ESPAsyncWebServer.h"

// include the library
#include <RadioLib.h>

// Libraries for WiFi
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

// Buffer to write message to display
char text[25] = {0};

WiFiClient espClient;

// ESPDateTime: https://blog.mcxiaoke.com/ESPDateTime/index.html
#include <DateTime.h>

// LoRa
// SX1262 radio = new Module(4, 11, 5, 3);
//SX1262 radio = new Module(SPI_LORA_SS, SPI_LORA_DIO1, SPI_LORA_RST, SPI_LORA_BUSY);
SX1262 radio = new Module(4, 11, 5, 3);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

// store lora data
//String str;
// END LoRA




// Define OLED instance
Adafruit_SSD1306 display(SCREEN_1_WIDTH, SCREEN_1_HEIGHT, &Wire, -1); // Instanziierung
#include "oled.h"

// *** OLED2
// declare size of working string buffers. Basic strlen("d hh:mm:ss") = 10
const size_t    MaxString               = 16;
// the string being displayed on the SSD1331 (initially empty)
char oldTimeString[MaxString]           = { 0 };

// OLED 128x128
#define SCREEN_2_WIDTH  128
#define SCREEN_2_HEIGHT 128 // Change this to 96 for 1.27" OLED.
Adafruit_SSD1351 oled = Adafruit_SSD1351(
  SCREEN_2_WIDTH, SCREEN_2_HEIGHT, 
  SPI_OLED_2_SS, 
  MISO, 
  MOSI,
  SCK,
  SPI_OLED_2_RST);

// SSD1331 color definitions
const uint16_t  OLED_Color_Black        = 0x0000;
const uint16_t  OLED_Color_Blue         = 0x001F;
// The colors we actually want to use
const uint16_t        OLED_Text_Color         = OLED_Color_Blue;
const uint16_t        OLED_Backround_Color    = OLED_Color_Black;  

void initOLED2() {
      // initialise the SSD1331
    oled.begin();
    oled.fillScreen(0x0000); // This clears entire screen (BLACK)
    oled.setFont();
    oled.fillScreen(OLED_Backround_Color);
    oled.setTextColor(OLED_Text_Color);
    oled.setTextSize(1);
    oled.setCursor(0,100);
    oled.print("OLED_Text_Color");
}


void displayUpTime()
{
    // calculate seconds, truncated to the nearest whole second
    unsigned long upSeconds = millis() / 1000;
    // calculate days, truncated to nearest whole day
    unsigned long days = upSeconds / 86400;
    // the remaining hhmmss are
    upSeconds = upSeconds % 86400;
    // calculate hours, truncated to the nearest whole hour
    unsigned long hours = upSeconds / 3600;
    // the remaining mmss are
    upSeconds = upSeconds % 3600;
    // calculate minutes, truncated to the nearest whole minute
    unsigned long minutes = upSeconds / 60;
    // the remaining ss are
    upSeconds = upSeconds % 60;

    // allocate a buffer
    char newTimeString[MaxString] = { 0 };

    // construct the string representation
    sprintf(
        newTimeString,
        "%lu %02lu:%02lu:%02lu",
        days, hours, minutes, upSeconds
    );

    // has the time string changed since the last oled update?
    if (strcmp(newTimeString,oldTimeString) != 0) {
        // yes! home the cursor
        oled.setCursor(0,0);
        // change the text color to the background color
        oled.setTextColor(OLED_Backround_Color);
        // redraw the old value to erase
        oled.print(oldTimeString);
        // home the cursor
        oled.setCursor(0,0);
        // change the text color to foreground color
        oled.setTextColor(OLED_Text_Color);
        // draw the new time value
        oled.print(newTimeString);
        oled.setCursor(0,20);
        oled.setTextColor(OLED_Backround_Color);
        oled.print(oldTimeString);
        oled.setCursor(0,20);
        oled.setTextColor(OLED_Color_Blue);
        oled.print(newTimeString);
        // and remember the new value
        strcpy(oldTimeString,newTimeString);
        
    }

}
// *** END OLED2


void showVersion()
{
  sprintf(text, "Version %s", String(VERSION));
  oledWriteMsg(0,displayRow1, text);
  oledWriteMsg(0,displayRow2, "LoRa Gateway");
  delay(2 * mS_TO_S_FACTOR); 
}


void setup()
{
  Serial.begin(BAUD);

  // Init Logger
  isDebugEnabled();

  debugOutput("***LoRa Receiver - Version " +String(VERSION), 5);

  Wire.setPins(I2C_SDA, I2C_SCL); // Set the I2C pins before begin
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

  initOLED();
  //initOLED2();
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

  setupDateTime();
  showTime();
  
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
  debugOutput("All seems Connected, disable Watchdog", 5);

  // AES
  aes_init();
  //aesLib.set_paddingmode(paddingMode::CMS);

  webserver();
  
  oled.fillScreen(OLED_Backround_Color);
  oled.setTextColor(OLED_Text_Color);
  oled.setTextSize(1);
  oled.setCursor(0,700);
  oled.print("OLED_Text_Color");

}



// Initialize OLED Module
void initOLED()
{
  debugOutput("Initializing OLED Module", 5);
  //reset OLED display via software
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3C for 128x32
    debugOutput("*SSD1306 allocation failed", 2);
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();
  oledWriteMsg(0,displayRow3, "OLED Module OK");

  Watchdog.reset(); 
 }


// Initialize LoRa Module
void initLoRA()
{
  debugOutput("Initializing LoRa Module", 5);  

  // initialize SX1276 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

    // set carrier frequency to 433.5 MHz
  if (radio.setFrequency(866.6) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("[SX1262] Selected frequency is invalid for this module!"));
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);


  // start listening for LoRa packets
  Serial.print(F("[SX1262] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
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









void connectToWifi() 
{
  debugOutput("Connecting to WiFi " +String(SECRET_WIFI_SSID), 5);
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("wifi."));
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





void WiFiEvent(WiFiEvent_t event)
{
  debugOutput("[WiFi-event] event: " +String(event), 5);
  switch(event) {
    //case SYSTEM_EVENT_STA_START:
    //  Serial.println("WiFi Started");
    //  break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      debugOutput("Wifi connected, RSSI: " +String(WiFi.RSSI())+ ", IP address: " +String(WiFi.localIP().toString()), 4);
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


void onMqttConnect(bool sessionPresent)
{
  debugOutput("Connected to MQTT, Session present: " +String(sessionPresent), 4);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  debugOutput("Disconnected from MQTT!", 4);

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }

  //if (WiFi.isConnected()) {
  //  mqttReconnectTimer.once(2, connectToMqtt);
  //}

}



void onMqttPublish(uint16_t packetId)
{
  debugOutput(" Received acknowledged for packetId " +String(packetId), 4);
  acknowledgedMsgCount = String(packetId);
}

/**
void getTs() {
  timeClient.update();
  long ts = timeClient.getEpochTime();
  String ts_string = timeClient.getFormattedTime();
  return long(ts), String(ts_string);
}
**/



void webserver()
{  
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
    data["wifi_rssi"] = String(WiFi.RSSI());
    data["LoRa Packets received"] = loraPacketRecv;
    data["MQTT Server"] = MQTT_BROKER;
    data["MQTT Status"] = "online";
    data["MQTT Packets acknowledged"] = acknowledgedMsgCount;

    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });

  server.onNotFound(notFound);
  server.begin();
}


void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}


void update_display()
{
  oledWriteMsg(0,displayRow1, "RUNNING...       OK");
  sprintf(text, "WiFi rssi: %s        ", String(WiFi.RSSI()));
  oledWriteMsg(0,displayRow3, text);
  sprintf(text, "MQTT tx: %s        ", String(acknowledgedMsgCount));
  oledWriteMsg(0,displayRow4, text);
  sprintf(text, "LoRa rx: %s        ", String(loraPacketRecv));
  oledWriteMsg(0,displayRow6, text);    
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



void getLoRaData(String lora_data){
  loraPacketRecv++;

  // packet was successfully received
  debugOutput("[SX1262] Received packet: " +String(loraPacketRecv), 4);

  // print data of the packet
  debugOutput("[SX1262] Data:\t" +String(lora_data),5);

  // print RSSI (Received Signal Strength Indicator)
  debugOutput("[SX1262] RSSI:\t" +String(radio.getRSSI())+ " dBm",5);

  // print SNR (Signal-to-Noise Ratio)
  debugOutput("[SX1262] SNR:\t" +String(radio.getSNR())+ " dB",5);


    // plain JSON
  if(lora_data.indexOf("data") > 0) {
      Serial.printf(" Receive Plain Data: \n ");  // muss so sein
      Serial.println(lora_data);
      JsonDocument doc; 
      deserializeJson(doc, lora_data);
      json_val = doc["data"];
      loraHasData = true;
    }
    // Error message
    else if(lora_data.indexOf("ERROR") > 0) {
      debugOutput(" ERROR message received from Node: " +String(lora_data), 3);
      JsonDocument doc; 
      doc["message"] = lora_data;
    //  doc["recipient"] = recipient;
    //  doc["loraRecvFrom"] = loraRecvFrom;
    //  doc["incomingMsgId"] = incomingMsgId;
    //  doc["incomingLengt"] = incomingLength;
      serializeJson(doc, jsonSerial);
      loraHasData = true;
    } 
    // encrypted
    else {
      debugOutput(" Received Encrypted Data:", 5);
      debugOutput("  Ciphertext: " +String(lora_data), 5);
      debugOutput("  Ciphertext length: " +String(lora_data.length()), 5);

      sprintf(ciphertext, "%s", lora_data.c_str());
      byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      String decrypted = decrypt_impl(ciphertext, dec_iv);

      debugOutput(String(decrypted), 5);

      JsonDocument doc2;
      deserializeJson(doc2, decrypted);
      json_val = doc2;

      loraHasData = true;
    }

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
      stateLED = HIGH;                                    // then set it HIGH for the next loop
      oledWriteMsg(100,displayRow1, "*   ");              // Print "ON" at Serial console
    } else  {      
      stateLED = LOW;                                     // in case LED is HIGH make LED LOW
      oledWriteMsg(100,displayRow1, "    ");              // Print "OFF" at Serial console
    }
  }
}





// main
void loop()
{

  // check if the flag is set
  if(receivedFlag) {
    Serial.println("receivedFlag = true");
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */
    
    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet!"));

 
      //digitalWrite(ledPin, HIGH); 
      oledWriteMsg(100, displayRow6, "***");
      
      getLoRaData(str);

      //digitalWrite(ledPin, LOW); 
      oledWriteMsg(100,displayRow6, "   ");
    
    
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


      // add data from LoRa
      doc["data"] = json_val;

      serializeJson(doc, jsonSerial);

      loraRecvFrom = "/" + loraRecvFrom;
      String topic = MQTT_PUB_PREFIX + loraRecvFrom + MQTT_PUB_DATA;
      uint16_t packetIdPub = mqttClient.publish(String(topic).c_str(), 1, true, jsonSerial);
      debugOutput("Publishing on topic " +String(topic)+ " PacketId: " +String(packetIdPub)+ " Message: " +String(jsonSerial), 5); 
      
      loraHasData = false;
    }
    
    

    // update when someting has changed
    update_display();   
 
    }

     delay(1000);

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






//

// MQTT Broker
const char* MQTT_BROKER = "192.168.1.10";
const int   MQTT_PORT = 1883;
const char* MQTT_USERNAME = "emqx";
const char* MQTT_PASSWORD = "password";
const char* MQTT_CLIENT_ID = "lora-receiver";
const char* MQTT_TOPIC = "emqx/esp32";

// Messages from LoRa Gateway
const char* MQTT_PUB_GW_ONLINE = "lora/gateway/online";
const char* MQTT_PUB_GW_LIVENESS = "lora/gateway/liveness";
const char* MQTT_PUB_GW_RSSI = "lora/gateway/rssi/wifi";

// Messages from LoRa Node
const char* MQTT_PUB_PREFIX = "lora/node";
const char* MQTT_PUB_DATA = "/data";

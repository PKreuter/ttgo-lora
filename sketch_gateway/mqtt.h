
// Config for MQTT

// MQTT Broker
const char* MQTT_BROKER = "192.168.1.10";
const int   MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "lora-gateway";
const char* MQTT_TOPIC = "emqx/esp32";

// Message queue from LoRa Gateway
const char* MQTT_PUB_GW_ONLINE = "lora/gateway/online";
const char* MQTT_PUB_GW_LIVENESS = "lora/gateway/liveness";
const char* MQTT_PUB_GW_RSSI = "lora/gateway/rssi/wifi";
const char* MQTT_PUB_GW_EVENTS = "lora/gateway/events";

// Messages queue from LoRa Node
const char* MQTT_PUB_PREFIX = "lora/node";
const char* MQTT_PUB_DATA = "/events";



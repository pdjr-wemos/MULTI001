 /* 
 * 
 * The code was designed to support the SmartDim Sensor 2 PIR/LUX
 * device (part number 86 454 523) sold by LuxControl as part of its
 * lighting management system. These 12VDC devices were top-quality
 * sensors and were discontinued when the manufacturer revised their
 * product range. At this time they could be obtained for a while at a
 * franction of their original, eye-watering, price. The sensor supports
 * the following connections:
 * 
 * BROWN  - +12VDC power in.
 * YELLOW - GND
 * ORANGE - PIR (normally 0VDC, 12VDC pulse when movement detected)
 * YELLOW - LUX (0 - 12VDC representing LUX level)
 * 
 * Interfacing these sensors to the Wemos board simply requires the the output voltages on PIR and LUX
 * have to be adjusted 
 * 
 *  
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/**********************************************************************
 * SERIAL DEBUG
 * 
 * Define DEBUG_SERIAL to enable serial output and arrange for the
 * function debugDump() to be called from loop() every
 * DEBUG_SERIAL_INTERVAL ms. DEBUG_SERIAL_START_DELAY prevents data
 * being written to the serial port immediately after system boot.
 */
#define DEBUG_SERIAL 1
#define DEBUG_SERIAL_START_DELAY 4000
#define DEBUG_SERIAL_INTERVAL 1000UL

#define GPIO_ONE_WIRE_BUS 4 // D1-MINI pin D2
#define GPIO_PIR_SENSOR 14 // D1-MINI pin D5
#define GPIO_LUX_SENSOR A0 // D1-MINI pin A0

#define WIFI_SERVER_PORT 80
#define WIFI_ACCESS_POINT_NAME_PREFIX "MULTISENSOR-"

#define MQTT_DEFAULT_SERVER_NAME "192.168.1.206"
#define MQTT_DEFAULT_SERVER_PORT 1883
#define MQTT_DEFAULT_USERNAME "preeve9534"
#define MQTT_DEFAULT_PASSWORD "samsam"

#define MQTT_PUBLISH_INTERVAL 30000
#define MQTT_SENSOR_TOPIC_PREFIX "homeassistant/multisensor"

#define STORAGE_TEST_ADDRESS 0
#define STORAGE_TEST_VALUE 0xAE
#define MQTT_CONFIG_STORAGE_ADDRESS 1
struct MQTT_CONFIG { char serverName[40]; int serverPort; char userName[20]; char password[20]; };

#define TEMPERATURE_SENSOR_DETECT_TRIES 5
#define TEMPERATURE_SENSOR_I2C_ADDRESS 18

WiFiServer wifiServer(WIFI_SERVER_PORT);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

OneWire oneWire(GPIO_ONE_WIRE_BUS);
DallasTemperature temperatureSensors(&oneWire);

/**********************************************************************
 * Setup a WiFi connection to <ssid>, <password> and only return once
 * a connection is established.
 */ 
void setup_wifi(const char* ssid, const char* password) {
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void connect_to_mqtt(MQTT_CONFIG &config) {
  while (!mqttClient.connected()) {
    #ifdef DEBUG_SERIAL
      Serial.print("Trying to connect to MQTT server '"); Serial.print(config.serverName); Serial.print("'...");
    #endif

    if (mqttClient.connect(config.serverName, config.userName, config.password)) {
      #ifdef DEBUG_SERIAL
        Serial.println("connected");
      #endif
    } else {
      #ifdef DEBUG_SERIAL
        Serial.print("failed (result code = ");
        Serial.print(mqttClient.state());
        Serial.println("). Will try again in 5 seconds.");
      #endif
      delay(5000);
    }
  }
}

void loadConfig(MQTT_CONFIG &config) {
  #ifdef DEBUG_SERIAL
  Serial.println("Loading configuration from EEPROM");
  #endif
  EEPROM.begin(512);
  if (EEPROM.read(0) == STORAGE_TEST_VALUE) {
    #ifdef DEBUG_SERIAL
    Serial.println("Loading configuration from EEPROM");
    #endif
    EEPROM.get(MQTT_CONFIG_STORAGE_ADDRESS, config);
  } else {
    #ifdef DEBUG_SERIAL
    Serial.println("Loading configuration from defaults");
    #endif
    strcpy(config.serverName, MQTT_DEFAULT_SERVER_NAME);
    config.serverPort = MQTT_DEFAULT_SERVER_PORT;
    strcpy(config.userName, MQTT_DEFAULT_USERNAME);
    strcpy(config.password, MQTT_DEFAULT_PASSWORD);
  }
  EEPROM.end();
}

void saveConfig(MQTT_CONFIG &config, const char *server, const char *port, const char *username, const char *password) {
  strcpy(config.serverName, server);
  config.serverPort = atoi(port);
  strcpy(config.userName, username);
  strcpy(config.password, password);
  EEPROM.begin(512);
  EEPROM.write(0, STORAGE_TEST_VALUE);
  EEPROM.put(MQTT_CONFIG_STORAGE_ADDRESS, config);
  EEPROM.commit();
  EEPROM.end();
}

bool shouldSaveConfig = false;

void saveConfigCallback() {
  shouldSaveConfig = true; 
}



char wifiAccessPointName[60];
char mqttStatusTopic[60];
char mqttStatusMessage[60];
byte macAddress[6];
MQTT_CONFIG mqttConfig;

float DETECTED_TEMPERATURE = 0.0; // Degrees Celsius
int DETECTED_MOTION = 0; // 0 = no motion detected, 1 = motion detected
int DETECTED_LUX = 0; // 0..1023
  
IRAM_ATTR void motionDetectionHandler() { DETECTED_MOTION = 1; }

void setup() {
  #ifdef DEBUG_SERIAL
  Serial.begin(57600);
  delay(DEBUG_SERIAL_START_DELAY);
  #endif

  loadConfig(mqttConfig);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqttConfig.serverName, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", "" + mqttConfig.serverPort, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqttConfig.userName, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqttConfig.password, 20);

  // Recover the wifi interface MAC address and use it to make unique
  // names for the our configuration access point and our MQTT client. 
  WiFi.macAddress(macAddress);
  sprintf(wifiAccessPointName, "%s-%02X%02X%02X%02X%02X%02X", WIFI_ACCESS_POINT_NAME_PREFIX, macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
  sprintf(mqttStatusTopic, "%s/%02x%02x%02x%02x%02x%02x/status", MQTT_SENSOR_TOPIC_PREFIX, macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
    
  WiFiManager wifiManager;
  
  // Uncomment next line to clean FS for testing 
  //WiFi.disconnect(true);
  //wifiManager.resetSettings();
  //WiFi.disconnect(true);

  
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  bool res = wifiManager.autoConnect(wifiAccessPointName);
  if (!res) {
    #ifdef DEBUG_SERIAL
      Serial.println("Failed to connect. Restarting system.");
    #endif
    ESP.restart();
  } else {
    #ifdef DEBUG_SERIAL
      Serial.print("Connected to wireless network.");
    #endif
    // We have a WiFi connection
    // If the configuration data has changed, then we should save it...
    if (shouldSaveConfig) saveConfig(mqttConfig, custom_mqtt_server.getValue(), custom_mqtt_port.getValue(), custom_mqtt_user.getValue(), custom_mqtt_pass.getValue());
    // Configure the MQTT client with the remote MQTT server name and port... 
    mqttClient.setServer(mqttConfig.serverName, mqttConfig.serverPort);
    temperatureSensors.begin();
    attachInterrupt(digitalPinToInterrupt(GPIO_PIR_SENSOR), motionDetectionHandler, RISING);
  }
}

void loop() {
  static long mqttDeadline = 0L;
  long now = millis();

  if (!mqttClient.connected()) connect_to_mqtt(mqttConfig);
  mqttClient.loop();

  if (now > mqttDeadline) {
    // Recover temperature sensor reading
    temperatureSensors.requestTemperatures();
    DETECTED_TEMPERATURE = temperatureSensors.getTempCByIndex(0);
    // Recover motion sensor reading
    DETECTED_MOTION = digitalRead(GPIO_PIR_SENSOR);
    // Recover LUX sensor reading
    DETECTED_LUX = analogRead(GPIO_LUX_SENSOR);


    sprintf(mqttStatusMessage, "{ \"temperature\": %f, \"motion\": %d, \"lux\": %d }", DETECTED_TEMPERATURE, DETECTED_MOTION, DETECTED_LUX);
    mqttClient.publish(mqttStatusTopic, mqttStatusMessage, true);

    DETECTED_MOTION = 0;
    DETECTED_LUX = 0;
    mqttDeadline = (now + MQTT_PUBLISH_INTERVAL);
    
    #ifdef DEBUG_SERIAL
      Serial.print("Writing ");
      Serial.print(mqttStatusMessage);
      Serial.print(" to ");
      Serial.println(mqttStatusTopic);
    #endif
  }
}

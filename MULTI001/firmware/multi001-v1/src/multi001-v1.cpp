/*********************************************************************
 * multi001-v1.cpp - multiple sensor wireless MQTT node.
 * 2021(c) Paul Reeve <preeve@pdjr.eu>
 */
 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG_SERIAL                      // Enable debug messages
#define DEBUG_SERIAL_START_DELAY 2000     // Milliseconds wait before output

#define GPIO_ONE_WIRE_BUS 4               // D1-MINI pin D2 (DS18B20 temp sensor)
#define GPIO_PIR_SENSOR 14                // D1-MINI pin D5
#define GPIO_LUX_SENSOR A0                // D1-MINI pin A0

#define WIFI_SERVER_PORT 80               
#define WIFI_ACCESS_POINT_NAME "MULTISENSOR-%s"
#define WIFI_ACCESS_POINT_PORTAL_TIMEOUT 180 // In seconds

#define MQTT_PUBLISH_INTERVAL 30000
#define MQTT_STATUS_TOPIC "multisensor/%s/status"
#define MQTT_STATUS_MESSAGE "{ \"temperature\": %f, \"motion\": %d, \"lux\": %d }" 

#define STORAGE_TEST_ADDRESS 0
#define STORAGE_TEST_VALUE 0xAE
#define MQTT_CONFIG_STORAGE_ADDRESS 1

/**********************************************************************
 * Structure to store MQTT configuration properties.
 */
struct MQTT_CONFIG { 
  char servername[40];  // MQTT server Hostname or IP address
  int  serverport;      // MQTT service port (normally 1883)
  char username[20];    // Name of user who can publish to the server
  char password[20];    // Password of named user
  char deviceid[20];    // Identifier for this device (defaults to MAC)
};

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
      Serial.print("Trying to connect to MQTT server '"); Serial.print(config.servername); Serial.print("'...");
    #endif

    if (mqttClient.connect(config.servername, config.username, config.password)) {
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

void dumpConfig(MQTT_CONFIG &config) {
  #ifdef DEBUG_SERIAL
  Serial.println("MQTT configuration:");
  Serial.print("  servername: "); Serial.println(config.servername);
  Serial.print("  serverport: "); Serial.println(config.serverport);
  Serial.print("  username: "); Serial.println(config.username);
  Serial.print("  password: "); Serial.println(config.password);
  Serial.print("  deviceid: "); Serial.println(config.deviceid);
  #endif
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
  }
  EEPROM.end();
}

void saveConfig(MQTT_CONFIG &config) {
  Serial.print("Saving "); dumpConfig(config);
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
byte macAddress[6];
MQTT_CONFIG mqttConfig;
bool MOTION_DETECTED = false;

float DETECTED_TEMPERATURE = 0.0; // Degrees Celsius
int DETECTED_MOTION = 0; // 0 = no motion detected, 1 = motion detected
int DETECTED_LUX = 0; // 0..1023
  
IRAM_ATTR void motionDetectionHandler() {
  MOTION_DETECTED = true;
}

void setup() {
  #ifdef DEBUG_SERIAL
  Serial.begin(57600);
  delay(DEBUG_SERIAL_START_DELAY);
  #endif

  // Recover device MAC address and make a nice string representation.
  //
  WiFi.macAddress(macAddress);
  char macAddressString[13];
  sprintf(macAddressString, "%02x%02x%02x%02x%02x%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);

  // Initialise the MQTT default configuration then try to load any saved data.
  strcpy(mqttConfig.servername, "");
  mqttConfig.serverport = 1883;
  strcpy(mqttConfig.username, "");
  strcpy(mqttConfig.password, "");
  strcpy(mqttConfig.deviceid, macAddressString);
  loadConfig(mqttConfig);
  
  // Create some parameters for our bespoke MQTT properties.
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqttConfig.servername, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", "" + mqttConfig.serverport, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqttConfig.username, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqttConfig.password, 20);
  WiFiManagerParameter custom_mqtt_deviceid("device", "mqtt device", mqttConfig.deviceid, 20);
  // Create a WiFiManager instance and configure it.
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(WIFI_ACCESS_POINT_PORTAL_TIMEOUT);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_deviceid);
  // Finally, Make a name for the our configuration access point and
  // start the WiFi manager. 
  sprintf(wifiAccessPointName, WIFI_ACCESS_POINT_NAME, macAddressString);
  bool res = wifiManager.autoConnect(wifiAccessPointName);

  // If the configuration data has changed, then get it and save it...
  if (shouldSaveConfig) {
      strcpy(mqttConfig.servername, custom_mqtt_server.getValue());
      mqttConfig.serverport = atoi(custom_mqtt_port.getValue());
      strcpy(mqttConfig.username, custom_mqtt_user.getValue());
      strcpy(mqttConfig.password, custom_mqtt_pass.getValue());
      strcpy(mqttConfig.deviceid, custom_mqtt_deviceid.getValue());
      saveConfig(mqttConfig);
    }

  // If we get to this point then the WiFi manager has either entered
  // configuration mode and being timed out or we are connected to the
  // configured network.  
  if (!res) {
    #ifdef DEBUG_SERIAL
      Serial.println("WiFi configuration or connection failure: restarting system.");
    #endif
    ESP.restart();
  } else {
    #ifdef DEBUG_SERIAL
      Serial.print("Connected to wireless network '");
      Serial.print(WiFi.SSID());
      Serial.println("'");
    #endif
    // We have a WiFi connection, so configure thrMQTT connection
    mqttClient.setServer(mqttConfig.servername, mqttConfig.serverport);
    // Personalise the MQTT status topic with our device id
    sprintf(mqttStatusTopic, MQTT_STATUS_TOPIC, mqttConfig.deviceid);
    // Start sensing things
    temperatureSensors.begin();
    pinMode(GPIO_PIR_SENSOR, INPUT);
    attachInterrupt(digitalPinToInterrupt(GPIO_PIR_SENSOR), motionDetectionHandler, CHANGE);
  }
}

/**********************************************************************
 * Check that we have an MQTT connection and if not, try and make one.
 * Otherwise, once every MQTT_PUBLISH_INTERVAL miliseconds read the
 * sensors and update the MQTT server.
 */
void loop() {
  static long mqttPublishDeadline = 0L;
  static char mqttStatusMessage[60];
  long now = millis();

  if (!mqttClient.connected()) connect_to_mqtt(mqttConfig);
  mqttClient.loop();

  if ((MOTION_DETECTED) || (now > mqttPublishDeadline)) {
    // Recover temperature and lux sensor readings. There is no need to
    // explicitly recover the motion sensor reading because it is
    // maintained by an interrupt service routine. 
    temperatureSensors.requestTemperatures();
    DETECTED_TEMPERATURE = temperatureSensors.getTempCByIndex(0);
    DETECTED_MOTION = digitalRead(GPIO_PIR_SENSOR);
    DETECTED_LUX = analogRead(GPIO_LUX_SENSOR);

    sprintf(mqttStatusMessage, MQTT_STATUS_MESSAGE, DETECTED_TEMPERATURE, DETECTED_MOTION, DETECTED_LUX);
    mqttClient.publish(mqttStatusTopic, mqttStatusMessage, true);

    MOTION_DETECTED = false;
    mqttPublishDeadline = (now + MQTT_PUBLISH_INTERVAL);
    
    #ifdef DEBUG_SERIAL
      Serial.print("Writing ");
      Serial.print(mqttStatusMessage);
      Serial.print(" to ");
      Serial.println(mqttStatusTopic);
    #endif
  }
}

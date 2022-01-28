/*********************************************************************
 * NAME
 *   multiHTT-0.90.cpp - multi-sensor wireless MQTT node.
 * PLATFORM
 *   Wemos MINI-D1
 * SENSORS
 *   AM2320 (humidity and temperature)
 *   HXD-2801 (ball tilt switch)
 * DESCRIPTION
 *   This firmware implements an IoT MQTT client which monitors
 *   ambient temperature and humidity and the orientation of an
 *   attached tilt sensor.
 * 
 *   Readings are published to a configured MQTT server (see
 *   below) as a JSON MQTT message of the form:
 * 
 *   '{ "humidity": humidity, "temperature": temperature, "tilt": tilt  }'
 * 
 *   where:
 * 
 *     humidity is a decimal percentage value in the range 0.0 - 100.0;
 *     temperature is decimal degrees Celsius value in the range -40.0 - 80.0;
 *     tilt is integer boolean, 0 or 1, indicating whether the tilt switch is 'on' or 'off'. 
 * 
 * CONFIGURATION
 * 
 * The module automatically enters configuration mode when it is unable
 * to connect to an already configured WiFi host network or on first use
 * when no host network has been configured.
 * 
 * In configuration mode the module operates as a wireless access pointOn first use the
 * module must be configured by the user who is required to specify
 * the SSID and password of the hsensed data is
 * published to a user specified MQTT topic on a user defined MQTT
 * server accessed over a user-specfifiea JSON styled message of the form:
 * 
 *   '{ "temperature": t, "lux": l, "motion": m }'
 * 
 * to a user specified topic on a nominated MQTT server.
 * 
 * The code supports a DS18B20 temperature sensor reporting the sensed
 * value <t> in degrees centigrade.
 * 
 * Illumination (lux) level <l> (in the range 0..1023) and detected
 * motion <m> (as 0 or 1) are assumed to derive from a luxControl
 * SmartDim Sensor 2.
 * 
 * On first use (and also when the device is unable to connect to a
 * previously configured wireless network) the device will operate as
 * a wireless access point with the SSID "MULTISENSOR-xxxxxxxxxxxx",
 * where "xxxxxxxxxxxx" is the MAC address of the host wireless
 * interface.
 * 
 * Connecting to the access point will open a captured portal that
 * allows configuration of the following settings:
 * 
 * network - the SSID of the network to which the device should connect
 * password - any password required for connection to ssid
 * server name - the name or IP address of the target MQTT server
 * server port - the port on which the server listens (default 1886)
 * username - login user name required for access to the server
 * password - login password for username
 * topic - the topic on server to which sensor data should be published
 * 
 * Once the entered settings are saved the device will re-boot and
 * immediately attempt to report sensor readings to the configured
 * destination. Detected movement results in an immediate report, but
 * otherwise readings will be reported once every 60 seconds.
 * 
 * AUTHOR
 *   2022(c) Paul Reeve <preeve@pdjr.eu>
 */
 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <Wire.h>
#include "AM232X.h"

#define DEBUG_SERIAL                      // Enable debug messages
#define DEBUG_SERIAL_START_DELAY 2000     // Milliseconds wait before output

#define GPIO_SW0 14                       // D1-MINI pin D5

#define MODULE_ID_FORMAT "MULTISENSOR-%02x%02x%02x%02x%02x%02x"
#define MQTT_DEFAULT_TOPIC_FORMAT "%s/status"

#define WIFI_SERVER_PORT 80               
#define WIFI_ACCESS_POINT_PORTAL_TIMEOUT 180 // In seconds

#define MQTT_PUBLISH_INTERVAL 30000
#define MQTT_CLIENT_ID "%02x%02x%02x%02x%02x%02x"
#define MQTT_STATUS_MESSAGE "{ \"humidity\": %f, \"temperature\": %f, \"tilt\": %d }" 

#define STORAGE_TEST_ADDRESS 0
#define STORAGE_TEST_VALUE 0xAE
#define MQTT_CONFIG_STORAGE_ADDRESS 1

#define TEMPERATURE_ERROR_VALUE 999.99
#define HUMIDITY_ERROR_VALUE 999.99

/**********************************************************************
 * Structure to store MQTT configuration properties.
 */
struct MQTT_CONFIG { 
  char servername[40];  // MQTT server Hostname or IP address
  int  serverport;      // MQTT service port (normally 1883)
  char username[20];    // Name of user who can publish to the server
  char password[20];    // Password of named user
  char topic[60];       // MQTT topic on which to publish
};

#define TEMPERATURE_SENSOR_DETECT_TRIES 5
#define TEMPERATURE_SENSOR_I2C_ADDRESS 18

WiFiServer wifiServer(WIFI_SERVER_PORT);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

AM232X AM2322;

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

void connect_to_mqtt(const char* servername, const int serverport, const char* username, const char* password, const char* clientid) {
  while (!mqttClient.connected()) {
    #ifdef DEBUG_SERIAL
      Serial.print("Trying to connect to MQTT server ");
      Serial.print(servername); Serial.print(":"); Serial.print(serverport);
      Serial.print(" as ");
      Serial.print(username); Serial.print("("); Serial.print(password); Serial.print(")");
      Serial.print(" with client id ");
      Serial.println(clientid);
    #endif

    if (mqttClient.connect(clientid, username, password)) {
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
  Serial.print("MQTT server name: "); Serial.println(config.servername);
  Serial.print("MQTT server port: "); Serial.println(config.serverport);
  Serial.print("MQTT username: "); Serial.println(config.username);
  Serial.print("MQTT password: "); Serial.println(config.password);
  Serial.print("MQTT topic: "); Serial.println(config.topic);
  #endif
}

boolean loadConfig(MQTT_CONFIG &config) {
  boolean retval = false;
  EEPROM.begin(512);
  if (EEPROM.read(0) == STORAGE_TEST_VALUE) {
    EEPROM.get(MQTT_CONFIG_STORAGE_ADDRESS, config);
    retval = true;
  }
  EEPROM.end();
  return(retval);
}

void saveConfig(MQTT_CONFIG &config) {
  #ifdef DEBUG_SERIAL
  Serial.println("Saving module configuration to EEPROM");
  dumpConfig(config);
  #endif
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

byte macAddress[6];
char moduleId[40];
char defaultTopic[60];
MQTT_CONFIG mqttConfig;

float DETECTED_HUMIDITY = 0.0; // Percent
float DETECTED_TEMPERATURE = 0.0; // Degrees Celsius
int DETECTED_TILT = 0; // 0 = no tilt detected, 1 = tilt detected

void setup() {
  #ifdef DEBUG_SERIAL
  Serial.begin(57600);
  delay(DEBUG_SERIAL_START_DELAY);
  #endif

  // Recover device MAC address and make from it a module identifier
  // that will be used as access point name, MQTT client id and a
  // component of the topic path (unless overriden by the user).
  WiFi.macAddress(macAddress);
  sprintf(moduleId, MODULE_ID_FORMAT, macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
  sprintf(defaultTopic, MQTT_DEFAULT_TOPIC_FORMAT, moduleId);

  // Initialise the MQTT default configuration then try to load any saved data.
  // Create a WiFiManager instance and configure it.
  WiFiManager wifiManager;
  WiFiManagerParameter custom_mqtt_servername("server", "mqtt server", "", 40);
  WiFiManagerParameter custom_mqtt_serverport("port", "mqtt port", "1883", 6);
  WiFiManagerParameter custom_mqtt_username("user", "mqtt user", "", 20);
  WiFiManagerParameter custom_mqtt_password("pass", "mqtt pass", "", 20);
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", defaultTopic, 40);
  
  // Try to load the module configuration.
  if (loadConfig(mqttConfig)) {
    // When the module WiFi service starts it may not be able to
    // connect to a wifi network and in this case will create an
    // access point to allow module configuration. We need to
    // create and initialise the WiFi manager configuration properties.
    WiFiManagerParameter custom_mqtt_servername("server", "mqtt server", mqttConfig.servername, 40);
    WiFiManagerParameter custom_mqtt_serverport("port", "mqtt port", "" + mqttConfig.serverport, 6);
    WiFiManagerParameter custom_mqtt_username("user", "mqtt user", mqttConfig.username, 20);
    WiFiManagerParameter custom_mqtt_password("pass", "mqtt pass", mqttConfig.password, 20);
    WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqttConfig.topic, 40);
  } else {
    wifiManager.resetSettings();
  }  
  
  // Create a WiFiManager instance and configure it.
  wifiManager.setConfigPortalTimeout(WIFI_ACCESS_POINT_PORTAL_TIMEOUT);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.addParameter(&custom_mqtt_servername);
  wifiManager.addParameter(&custom_mqtt_serverport);
  wifiManager.addParameter(&custom_mqtt_username);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_mqtt_topic);
  
  // Finally, start the WiFi manager. 
  bool res = wifiManager.autoConnect(moduleId);

  // If the configuration data has changed, then get it and save it...
  if (shouldSaveConfig) {
    strcpy(mqttConfig.servername, custom_mqtt_servername.getValue());
    mqttConfig.serverport = atoi(custom_mqtt_serverport.getValue());
    strcpy(mqttConfig.username, custom_mqtt_username.getValue());
    strcpy(mqttConfig.password, custom_mqtt_password.getValue());
    strcpy(mqttConfig.topic, custom_mqtt_topic.getValue());
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
    // We have a WiFi connection, so configure the MQTT connection
    mqttClient.setServer(mqttConfig.servername, mqttConfig.serverport);
    // Start sensing things
    if (! AM2322.begin()) {
      Serial.println("AM2322 sensor not found");
      while (1);
    }
    AM2322.wakeUp();
    delay(2000);
    pinMode(GPIO_SW0, INPUT_PULLUP);
  }
}

/**********************************************************************
 * Check that we have an MQTT connection and if not, try and make one.
 * Otherwise, once every MQTT_PUBLISH_INTERVAL miliseconds read the
 * sensors and update the MQTT server.
 */
void loop() {
  static long mqttPublishDeadline = 0L;
  static char mqttStatusMessage[128];
  long now = millis();

  if (!mqttClient.connected()) connect_to_mqtt(mqttConfig.servername, mqttConfig.serverport, mqttConfig.username, mqttConfig.password, moduleId);
  mqttClient.loop();

  if (now > mqttPublishDeadline) {
    DETECTED_TILT = digitalRead(GPIO_SW0);
  
    if (AM2322.read() == AM232X_OK) {
      DETECTED_TEMPERATURE = AM2322.getTemperature();
      DETECTED_HUMIDITY = AM2322.getHumidity();
    } else {
      DETECTED_TEMPERATURE = TEMPERATURE_ERROR_VALUE;
      DETECTED_HUMIDITY = HUMIDITY_ERROR_VALUE;
    }

    sprintf(mqttStatusMessage, MQTT_STATUS_MESSAGE, DETECTED_HUMIDITY, DETECTED_TEMPERATURE, DETECTED_TILT);
    mqttClient.publish(mqttConfig.topic, mqttStatusMessage, true);

    #ifdef DEBUG_SERIAL
      Serial.print("Writing ");
      Serial.print(mqttStatusMessage);
      Serial.print(" to ");
      Serial.println(mqttConfig.topic);
    #endif

    mqttPublishDeadline = (now + MQTT_PUBLISH_INTERVAL);
  }
}

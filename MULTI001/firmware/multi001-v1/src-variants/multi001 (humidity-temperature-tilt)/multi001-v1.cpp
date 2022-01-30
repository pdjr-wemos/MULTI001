/*********************************************************************
 * NAME
 *   multiHTT-0.90.cpp - multi-sensor wireless MQTT node.
 * PLATFORM
 *   Wemos MINI-D1
 * SENSORS
 *   AM2320 (I2C humidity and temperature)
 *   SPST switches (x4)
 * DESCRIPTION
 *   This firmware implements an IoT MQTT client which reports sensor
 *   data form a range of devices that may be connected to a Wemos D1
 *   Mini microcontroller. The following sensors are supported.
 * 
 *   1. AM2320 humidity & temperature
 *      This sensor connects via the I2C bus on pins D1 & D2. The sensor
 *      is automatically detected and no user configuration is required.
 *      
 *      Property name       Value
 *      humidity            Integer percent in the range 0..100
 *      temperature         Integer Celsius in the range -40..80
 * 
 *   2. SPST switches
 *      A maximum of four switches (SW0, SW1, SW2 and SW3) are supported
 *      with active-low connection to pins D5, D6, D7 and D8. Switches
 *      must be manually configured by assigning a property name at module
 *      configuration to each active switch.
 * 
 *      Property name       Value
 *      as configured       Integer boolean 0 or 1 (OFF or ON) 
 * 
 *   A JSON object containing properties relating to detected and/or
 *   configured sensors are published to a user defined topic on a user
 *   configured MQTT server (see CONFIGURATION below).
 * 
 *   The value 999, meaning undefined, is published to indicate that
 *   reading the associated sensor failed for whatever reason.
 * 
 *   The defined MQTT topic is updated whenever a sensor value changes
 *   or once every 30 seconds. The maximum update rate is once every
 *   three seconds.
 * 
 * CONFIGURATION
 * 
 * On first use (and also when the device is unable to connect to a
 * previously configured wireless network) the device will operate as
 * an open wireless access point with the SSID "MULTISENSOR-xxxxxxxxxxxx",
 * where "xxxxxxxxxxxx" is the MAC address of the host wireless
 * interface.
 * 
 * Connection to this access point will open a captive portal that
 * allows the user to configure the following properties:
 * 
 * network                 The SSID of the host network to which the
 *                         device should connect.
 * 
 * password                The password (if any) required to connect to
 *                         specified host network.
 * 
 * server name             The name or IP address of the MQTT server to
 *                         which status updates should be sent.
 * 
 * server port             The port on which the server listens (default
 *                         1886).
 * 
 * username                The login user name required for access to
 *                         server name.
 * 
 * password                The login password for username.
 * 
 * topic                   The topic on which to publish data.
 * 
 * prop name for SW[0..3]  A JSON property name to be used for each of
 *                         switches SW[0..3]. Not supplying a property
 *                         name disables the associated switch input.
 * 
 * When the configuration is saved the device will immediately reboot
 * and attempt to enter production with the specified configuration.
 */
 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <Wire.h>
#include <AM232X.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG_SERIAL                      // Enable debug messages
#define DEBUG_SERIAL_START_DELAY 2000     // Milliseconds wait before output

#define GPIO_SCL 1                        // I2C SCL
#define GPIO_SDA 2                        // I2C SDA
#define GPIO_ONE_WIRE_BUS 4               // For Dallas temperature sensors
#define GPIO_SW0 5                        // SPST switch
#define GPIO_SW1 6                        // SPST switch
#define GPIO_SW2 7                        // SPST switch
#define GPIO_SW3 8                        // SPST switch

#define MODULE_ID_FORMAT "MULTISENSOR-%02x%02x%02x%02x%02x%02x"
#define MQTT_DEFAULT_TOPIC_FORMAT "%s/status"

#define WIFI_SERVER_PORT 80               
#define WIFI_ACCESS_POINT_PORTAL_TIMEOUT 180 // In seconds

#define MQTT_PUBLISH_SOFT_INTERVAL 3000
#define MQTT_PUBLISH_HARD_INTERVAL 30000

#define EEPROM_IS_CONFIGURED_TOKEN_STORAGE_ADDRESS 0
#define EEPROM_IS_CONFIGURED_TOKEN_VALUE 0xAE
#define MQTT_CONFIG_STORAGE_ADDRESS 1

#define JSON_BUFFER_SIZE 300
#define AM2322_STARTUP_DELAY 2000
#define SENSOR_UNDEFINED_VALUE 999

#define DALLAS_ADDRESS_FORMAT "DST%02x%02x%02x%02x%02x%02x%02x%02x"

/**********************************************************************
 * Structure to store module configuration properties.
 */
struct MQTT_CONFIG { 
  char servername[40];     // MQTT server Hostname or IP address
  int  serverport;         // MQTT service port (normally 1883)
  char username[20];       // Name of user who can publish to the server
  char password[20];       // Password of named user
  char topic[60];          // MQTT topic on which to publish
  char sw0propertyname[20];
  char sw1propertyname[20];
  char sw2propertyname[20];
  char sw3propertyname[20];
};

/**********************************************************************
 * Globals representing WiFi and MQTT entities.
 */
WiFiServer wifiServer(WIFI_SERVER_PORT);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/**********************************************************************
 * Globals representing sensor entities.
 */
AM232X AM2322;                    // I2C humidity/temperature
OneWire oneWire(GPIO_ONE_WIRE_BUS);
DallasTemperature temperatureSensors(&oneWire);

/**********************************************************************
 * Used by loop() to automatically reconnect to the specified MQTT
 * server if the connection fails for any reason.
 */
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

/**********************************************************************
 * Debug dump the content of the specified configuration object.
 */
void dumpConfig(MQTT_CONFIG &config) {
  #ifdef DEBUG_SERIAL
  Serial.print("MQTT server name: "); Serial.println(config.servername);
  Serial.print("MQTT server port: "); Serial.println(config.serverport);
  Serial.print("MQTT username: "); Serial.println(config.username);
  Serial.print("MQTT password: "); Serial.println(config.password);
  Serial.print("MQTT topic: "); Serial.println(config.topic);
  Serial.print("MQTT SW0 property name: "); Serial.println(config.sw0propertyname);
  Serial.print("MQTT SW1 property name: "); Serial.println(config.sw1propertyname);
  Serial.print("MQTT SW2 property name: "); Serial.println(config.sw2propertyname);
  Serial.print("MQTT SW3 property name: "); Serial.println(config.sw3propertyname);
  #endif
}

/**********************************************************************
 * Load the specified configuration object with data from EEPROM.
 */
boolean loadConfig(MQTT_CONFIG &config) {
  boolean retval = false;
  EEPROM.begin(512);
  if (EEPROM.read(EEPROM_IS_CONFIGURED_TOKEN_STORAGE_ADDRESS) == EEPROM_IS_CONFIGURED_TOKEN_VALUE) {
    EEPROM.get(MQTT_CONFIG_STORAGE_ADDRESS, config);
    retval = true;
  }
  EEPROM.end();
  return(retval);
}

/**********************************************************************
 * Save the specified configuration object to EEPROM.
 */
void saveConfig(MQTT_CONFIG &config) {
  #ifdef DEBUG_SERIAL
  Serial.println("Saving module configuration to EEPROM");
  dumpConfig(config);
  #endif
  EEPROM.begin(512);
  EEPROM.write(EEPROM_IS_CONFIGURED_TOKEN_STORAGE_ADDRESS, EEPROM_IS_CONFIGURED_TOKEN_VALUE);
  EEPROM.put(MQTT_CONFIG_STORAGE_ADDRESS, config);
  EEPROM.commit();
  EEPROM.end();
}

/**********************************************************************
 * Method called when the user updates the module configuration through
 * the captive portal and a global variable which is used to flag this
 * fact for subsequent action.
 */

bool shouldSaveConfig = false;

void saveConfigCallback() {
  shouldSaveConfig = true; 
}

byte macAddress[6];
char moduleId[40];
char defaultTopic[60];
MQTT_CONFIG mqttConfig;
StaticJsonDocument<JSON_BUFFER_SIZE> jsonBuffer;
int dallasDeviceCount = 0;

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
  WiFiManagerParameter custom_mqtt_property_name_0("prop0", "mqtt prop name for SW0", "", 20);
  WiFiManagerParameter custom_mqtt_property_name_1("prop1", "mqtt prop name for SW1", "", 20);
  WiFiManagerParameter custom_mqtt_property_name_2("prop2", "mqtt prop name for SW2", "", 20);
  WiFiManagerParameter custom_mqtt_property_name_3("prop3", "mqtt prop name for SW3", "", 20);
  
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
    WiFiManagerParameter custom_mqtt_property_name_0("prop0", "mqtt prop name for SW0", mqttConfig.sw0propertyname, 20);
    WiFiManagerParameter custom_mqtt_property_name_1("prop1", "mqtt prop name for SW1", mqttConfig.sw1propertyname, 20);
    WiFiManagerParameter custom_mqtt_property_name_2("prop2", "mqtt prop name for SW2", mqttConfig.sw2propertyname, 20);
    WiFiManagerParameter custom_mqtt_property_name_3("prop3", "mqtt prop name for SW3", mqttConfig.sw3propertyname, 20);
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
  wifiManager.addParameter(&custom_mqtt_property_name_0);
  wifiManager.addParameter(&custom_mqtt_property_name_1);
  wifiManager.addParameter(&custom_mqtt_property_name_2);
  wifiManager.addParameter(&custom_mqtt_property_name_3);
  
  // Finally, start the WiFi manager. 
  bool res = wifiManager.autoConnect(moduleId);

  // If the configuration data has changed, then get it and save it...
  if (shouldSaveConfig) {
    strcpy(mqttConfig.servername, custom_mqtt_servername.getValue());
    mqttConfig.serverport = atoi(custom_mqtt_serverport.getValue());
    strcpy(mqttConfig.username, custom_mqtt_username.getValue());
    strcpy(mqttConfig.password, custom_mqtt_password.getValue());
    strcpy(mqttConfig.topic, custom_mqtt_topic.getValue());
    strcpy(mqttConfig.sw0propertyname, custom_mqtt_property_name_0.getValue());
    strcpy(mqttConfig.sw1propertyname, custom_mqtt_property_name_1.getValue());
    strcpy(mqttConfig.sw2propertyname, custom_mqtt_property_name_2.getValue());
    strcpy(mqttConfig.sw3propertyname, custom_mqtt_property_name_3.getValue());
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

    // Sensor detection
    Serial.print("Detected sensors: ");

    // Dallas one-wire temperature sensors
    DeviceAddress dallasAddress;
    char dallasAddressString[20];
    temperatureSensors.begin();
    if (dallasDeviceCount = temperatureSensors.getDeviceCount()) {
      for (int i = 0; i < dallasDeviceCount; i++) {
        if (temperatureSensors.getAddress(dallasAddress, i)) {
          sprintf(dallasAddressString, DALLAS_ADDRESS_FORMAT, dallasAddress[0], dallasAddress[1], dallasAddress[2], dallasAddress[3], dallasAddress[4], dallasAddress[5], dallasAddress[6], dallasAddress[7]);
          Serial.print(dallasAddressString);
          Serial.print(" ");
        }
      }
    }

    // AM2322 initialisation
    if (AM2322.begin()) {
      Serial.print("AM2322 ");
      AM2322.wakeUp();
      delay(AM2322_STARTUP_DELAY);
    }

    // SW0
    if (strlen(mqttConfig.sw0propertyname)) {
      Serial.print("SW0["); Serial.print(mqttConfig.sw0propertyname); Serial.print("]");
      pinMode(GPIO_SW0, INPUT_PULLUP);
    }

    // SW1
    if (strlen(mqttConfig.sw1propertyname)) {
      Serial.print("SW1["); Serial.print(mqttConfig.sw1propertyname); Serial.print("]");
      pinMode(GPIO_SW1, INPUT_PULLUP);
    }

    // SW2
    if (strlen(mqttConfig.sw2propertyname)) {
      Serial.print("SW2["); Serial.print(mqttConfig.sw2propertyname); Serial.print("]");
      pinMode(GPIO_SW2, INPUT_PULLUP);
    }

    // SW3
    if (strlen(mqttConfig.sw3propertyname)) {
      Serial.print("SW3["); Serial.print(mqttConfig.sw3propertyname); Serial.print("]");
      pinMode(GPIO_SW3, INPUT_PULLUP);
    }

    Serial.println();
    // End of sensor detection
    
  }
}

/**********************************************************************
 * Begin by checking that we have an active MQTT connection.  If not,
 * then try to make one. 
 * 
 * Once every MQTT_PUBLISH_SOFT_INTERVAL miliseconds read the sensors.
 * If the sensor values have changed from those most recently published
 * or MQTT_PUBLISH_HARD_INTERVAL has elapsed then update the configured
 * topic on the connected MQTT server.
 */
void loop() {
  static long mqttPublishSoftDeadline = 0L;
  static long mqttPublishHardDeadline = 0L;
  static char mqttStatusMessage[256];
  DeviceAddress dallasAddress;
  char dallasAddressString[20];
  long now = millis();
  int dirty = false;

  // Try and recover a failed server connection
  if (!mqttClient.connected()) connect_to_mqtt(mqttConfig.servername, mqttConfig.serverport, mqttConfig.username, mqttConfig.password, moduleId);
  
  // Perform connection houskeeping
  mqttClient.loop();

  // Check if our time has come
  if (now > mqttPublishSoftDeadline) {

    if (dallasDeviceCount) {
      temperatureSensors.requestTemperatures();
      for (int i = 0; i < dallasDeviceCount; i++) {
        if (temperatureSensors.getAddress(dallasAddress, i)) {
          sprintf(dallasAddressString, DALLAS_ADDRESS_FORMAT, dallasAddress[0], dallasAddress[1], dallasAddress[2], dallasAddress[3], dallasAddress[4], dallasAddress[5], dallasAddress[6], dallasAddress[7]);
          if ((int) jsonBuffer[dallasAddressString] != (int) round(temperatureSensors.getTempC(dallasAddress))) { jsonBuffer[dallasAddressString] = (int) round(temperatureSensors.getTempC(dallasAddress)); dirty = true; }
        }
      }
    }

    if (AM2322.isConnected()) {
      if (AM2322.read() == AM232X_OK) {
        if ((int) jsonBuffer["humidity"] != (int) round(AM2322.getHumidity())) { jsonBuffer["humidity"] = (int) round(AM2322.getHumidity()); dirty = true; };
        if ((int) jsonBuffer["temperature"] != (int) round(AM2322.getTemperature())) { jsonBuffer["temperature"] = (int) round(AM2322.getTemperature()); dirty = true; };
      } else {
        if ((int) jsonBuffer["humidity"] != SENSOR_UNDEFINED_VALUE) { jsonBuffer["humidity"] = SENSOR_UNDEFINED_VALUE; dirty = true; };
        if ((int) jsonBuffer["temperature"] != SENSOR_UNDEFINED_VALUE) { jsonBuffer["temperature"] = SENSOR_UNDEFINED_VALUE; dirty = true; };
      }
    }

    if (strlen(mqttConfig.sw0propertyname)) {
      if (jsonBuffer[mqttConfig.sw0propertyname] != digitalRead(GPIO_SW0)) { jsonBuffer[mqttConfig.sw0propertyname] = digitalRead(GPIO_SW0); dirty = true; };
    }

    if (strlen(mqttConfig.sw1propertyname)) {
      if (jsonBuffer[mqttConfig.sw1propertyname] != digitalRead(GPIO_SW1)) { jsonBuffer[mqttConfig.sw1propertyname] = digitalRead(GPIO_SW1); dirty = true; };
    }

    if (strlen(mqttConfig.sw2propertyname)) {
      if (jsonBuffer[mqttConfig.sw2propertyname] != digitalRead(GPIO_SW2)) { jsonBuffer[mqttConfig.sw2propertyname] = digitalRead(GPIO_SW2); dirty = true; };
    }

    if (strlen(mqttConfig.sw3propertyname)) {
      if (jsonBuffer[mqttConfig.sw3propertyname] != digitalRead(GPIO_SW3)) { jsonBuffer[mqttConfig.sw3propertyname] = digitalRead(GPIO_SW3); dirty = true; };
    }

    // Check if we should actually publish this data
    if (dirty || (now > mqttPublishHardDeadline)) {
      serializeJson(jsonBuffer, mqttStatusMessage);
      mqttClient.publish(mqttConfig.topic, mqttStatusMessage, true);

      #ifdef DEBUG_SERIAL
        Serial.print("Publishing ");
        Serial.print(mqttStatusMessage);
        Serial.print(" to ");
        Serial.println(mqttConfig.topic);
      #endif

      mqttPublishHardDeadline = (now + MQTT_PUBLISH_HARD_INTERVAL);
    }
    mqttPublishSoftDeadline = (now + MQTT_PUBLISH_SOFT_INTERVAL);
  }
}

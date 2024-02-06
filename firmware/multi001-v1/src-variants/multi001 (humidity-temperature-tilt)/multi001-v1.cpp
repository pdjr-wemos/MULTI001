/*********************************************************************
 * NAME
 *   multi001-v1.cpp - multi-sensor wireless MQTT node.
 * PLATFORM
 *   ESP8266/Wemos MINI-D1
 * SENSORS
 *   AM2320 (I2C humidity and temperature)
 *   SPST switches (x4)
 * DESCRIPTION
 *   This firmware implements an IoT MQTT client which reports sensor
 *   data from SPST switches and a range of devices connected to the
 *   host microcontroller over I2C or one-wire busses.
 * 
 *   The generated MQTT message is a JSON object with properties
 *   reflecting data harvested from one or more of the following
 *   sensors.
 * 
 *   1. SPST switches
 *      Up to two active-low SPST switches can be connected to
 *      GPIO14(D5) and GPIO12(D6). The following two properties are
 *      always included in the output message (note that the reported
 *      property names can be overriden during module configuration).
 * 
 *      PROPERTY            VALUE
 *      sw0 (or alias)      Integer boolean 0 or 1 (OFF or ON) 
 *      sw1 (or alias)      Integer boolean 0 or 1 (OFF or ON)
* 
 *   2. AM2320 humidity & temperature
 *      
 *      A single sensor of this type can be connected to the I2C bus
 *      on GPIO5(D1/SCL) and GPIO4(D2/SDA). The sensor is automatically
 *      detected and no user configuration is required. Presence of the
 *      sensor adds the following properties to the output message.
 *      
 *      PROPERTY            VALUE
 *      humidity            Integer percent in the range 0..100
 *      temperature         Integer Celsius in the range -40..80
 * 
 *   3. DS18B20 temperature sensors
 * 
 *      An arbitrary number of sensors of this type can be connected to
 *      the one-wire bus on GPIO13(D?). Sensors are automatically
 *      detected and no user configuration is required. Each detected
 *      sensor adds a property of the following form to the output
 *      message.
 * 
 *      PROPERTY             VALUE
 *      DS-address           Integer Celsius in the range -40..120
 *  
 *   A JSON object containing properties relating to detected and/or
 *   configured sensors are published to a user defined topic on a user
 *   configured MQTT server (see CONFIGURATION below).
 * 
 *   The value 999, meaning undefined, is published to indicate that
 *   reading a detected or configured sensor failed for whatever reason.
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
 * sw0 alias               A JSON property name to be used instead of
 *                         the default (sw0)
 *
 * sw1 alias               A JSON property name to be used instead of
 *                         the default (sw1)
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

#define GPIO_SCL 5                        // I2C SCL
#define GPIO_SDA 4                        // I2C SDA
#define GPIO_ONE_WIRE_BUS 13              // For Dallas temperature sensors
#define GPIO_SW0 14                       // SPST switch
#define GPIO_SW1 12                       // SPST switch

#define MODULE_ID_FORMAT "MULTISENSOR-%02x%02x%02x%02x%02x%02x"

// User configuration access-point settings
#define AP_PORTAL_SERVICE_PORT 80               
#define AP_PORTAL_TIMEOUT 180

// User configuration property settings and defaults
#define CF_DEFAULT_MQTT_TOPIC_FORMAT "multisensor/%s"
#define CF_DEFAULT_MQTT_SERVICE_PORT 1886
#define CF_DEFAULT_PROPERTY_NAME_FOR_SW0 "sw0"
#define CF_DEFAULT_PROPERTY_NAME_FOR_SW1 "sw1"
#define CF_DEFAULT_MQTT_PUBLISH_SOFT_INTERVAL 3000
#define CF_DEFAULT_MQTT_PUBLISH_HARD_INTERVAL 30000

// Persistent storage addresses and default values
#define PS_IS_CONFIGURED_TOKEN_STORAGE_ADDRESS 0
#define PS_IS_CONFIGURED_TOKEN_VALUE 0xAE
#define PS_USER_CONFIGURATION_STORAGE_ADDRESS 1

// Miscellaneous sensor configuration settings 
#define AM2322_STARTUP_DELAY 2000
#define DS18B20_NAME_FORMAT "DS-%02x%02x%02x%02x%02x%02x%02x%02x"

#define JSON_BUFFER_SIZE 300
#define SENSOR_UNDEFINED_VALUE 999

/**********************************************************************
 * Structure to store user configuration. Note that the host network
 * settings are managed and persisted by the module itself.
 */
struct USER_CONFIGURATION { 
  char servername[40];            // MQTT server Hostname or IP address
  int  serverport;                // MQTT service port (normally 1883)
  char username[20];              // Name of user who can publish to the server
  char password[20];              // Password of named user
  char topic[60];                 // MQTT topic on which to publish
  int softpublicationinterval;    // Soft publication interval
  int hardpublicationinterval;    // Hard publication interval
  char sw0propertyname[20];       // Property name to use for first SPST switch
  char sw1propertyname[20];       // Property name to use for second SPST switch
};

/**********************************************************************
 * Globals representing WiFi and MQTT entities.
 */
WiFiServer wifiServer(AP_PORTAL_SERVICE_PORT);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/**********************************************************************
 * Globals representing sensor entities.
 */
AM232X AM2322;                    // I2C humidity/temperature
OneWire oneWire(GPIO_ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

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
void dumpConfig(USER_CONFIGURATION &config) {
  #ifdef DEBUG_SERIAL
  Serial.print("MQTT server name: "); Serial.println(config.servername);
  Serial.print("MQTT server port: "); Serial.println(config.serverport);
  Serial.print("MQTT username: "); Serial.println(config.username);
  Serial.print("MQTT password: "); Serial.println(config.password);
  Serial.print("MQTT topic: "); Serial.println(config.topic);
  Serial.print("MQTT SW0 property name: "); Serial.println(config.sw0propertyname);
  Serial.print("MQTT SW1 property name: "); Serial.println(config.sw1propertyname);
  Serial.print("MQTT soft publication interval: "); Serial.println(config.softpublicationinterval);
  Serial.print("MQTT hard publication interval: "); Serial.println(config.hardpublicationinterval);
  #endif
}

/**********************************************************************
 * Load the specified configuration object with data from EEPROM.
 */
boolean loadConfig(USER_CONFIGURATION &config) {
  #ifdef DEBUG_SERIAL
  Serial.println("Loading module configuration from EEPROM");
  #endif
  boolean retval = false;
  EEPROM.begin(512);
  if (EEPROM.read(PS_IS_CONFIGURED_TOKEN_STORAGE_ADDRESS) == PS_IS_CONFIGURED_TOKEN_VALUE) {
    EEPROM.get(PS_USER_CONFIGURATION_STORAGE_ADDRESS, config);
    retval = true;
  }
  EEPROM.end();
  #ifdef DEBUG_SERIAL
  dumpConfig(config);
  #endif
  return(retval);
}

/**********************************************************************
 * Save the specified configuration object to EEPROM.
 */
void saveConfig(USER_CONFIGURATION &config) {
  #ifdef DEBUG_SERIAL
  Serial.println("Saving module configuration to EEPROM");
  dumpConfig(config);
  #endif
  EEPROM.begin(512);
  EEPROM.write(PS_IS_CONFIGURED_TOKEN_STORAGE_ADDRESS, PS_IS_CONFIGURED_TOKEN_VALUE);
  EEPROM.put(PS_USER_CONFIGURATION_STORAGE_ADDRESS, config);
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
USER_CONFIGURATION mqttConfig;
boolean userConfigurationLoaded = false;
StaticJsonDocument<JSON_BUFFER_SIZE> jsonBuffer;
int DS18B20_DEVICE_COUNT = 0;

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
  char defaultTopic[60];
  char buffer[10];
  sprintf(defaultTopic, CF_DEFAULT_MQTT_TOPIC_FORMAT, moduleId);

  // Try to load user configuration
  userConfigurationLoaded = loadConfig(mqttConfig);

  // Initialise the WiFi portal with either the just loaded data or
  // with some anaemic defaults.
  WiFiManager wifiManager;
  if (!userConfigurationLoaded) wifiManager.resetSettings();
  WiFiManagerParameter custom_mqtt_servername("server", "mqtt server", (userConfigurationLoaded)?mqttConfig.servername:"", 40);
  sprintf(buffer, "%d", (userConfigurationLoaded)?mqttConfig.serverport:CF_DEFAULT_MQTT_SERVICE_PORT);
  WiFiManagerParameter custom_mqtt_serverport("port", "mqtt port", buffer, 6);
  WiFiManagerParameter custom_mqtt_username("user", "mqtt user", (userConfigurationLoaded)?mqttConfig.username:"", 20);
  WiFiManagerParameter custom_mqtt_password("pass", "mqtt pass", (userConfigurationLoaded)?mqttConfig.password:"", 20);
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", (userConfigurationLoaded)?mqttConfig.topic:defaultTopic, 40);
  sprintf(buffer, "%d", (userConfigurationLoaded)?mqttConfig.softpublicationinterval:CF_DEFAULT_MQTT_PUBLISH_SOFT_INTERVAL);
  WiFiManagerParameter custom_mqtt_softinterval("softinterval", "mqtt soft interval", buffer, 6);
  sprintf(buffer, "%d", (userConfigurationLoaded)?mqttConfig.hardpublicationinterval:CF_DEFAULT_MQTT_PUBLISH_HARD_INTERVAL);
  WiFiManagerParameter custom_mqtt_hardinterval("hardinterval", "mqtt hard interval", buffer, 6);
  WiFiManagerParameter custom_mqtt_sw0_alias("sw0alias", "alias for sw0", (userConfigurationLoaded)?mqttConfig.sw0propertyname:CF_DEFAULT_PROPERTY_NAME_FOR_SW0, 20);
  WiFiManagerParameter custom_mqtt_sw1_alias("sw1alias", "alias for sw1", (userConfigurationLoaded)?mqttConfig.sw1propertyname:CF_DEFAULT_PROPERTY_NAME_FOR_SW1, 20);
  
  // Create a WiFiManager instance and configure it.
  wifiManager.setConfigPortalTimeout(AP_PORTAL_TIMEOUT);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.addParameter(&custom_mqtt_servername);
  wifiManager.addParameter(&custom_mqtt_serverport);
  wifiManager.addParameter(&custom_mqtt_username);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_mqtt_topic);
  wifiManager.addParameter(&custom_mqtt_softinterval);
  wifiManager.addParameter(&custom_mqtt_hardinterval);
  wifiManager.addParameter(&custom_mqtt_sw0_alias);
  wifiManager.addParameter(&custom_mqtt_sw1_alias);
  
  // Finally, start the WiFi manager. 
  bool res = wifiManager.autoConnect(moduleId);

  // When we reach this point, the WiFi manager may have connected to
  // its host network or not as indicated by the value of res.
  //
  // In either case, it may have been in configuration mode and had its
  // configuration settings changed (as indicated by shouldSaveConfig)
  // and we need in this case to make sure we preserve our user
  // configuration.
  
  if (shouldSaveConfig) {
    strcpy(mqttConfig.servername, custom_mqtt_servername.getValue());
    mqttConfig.serverport = atoi(custom_mqtt_serverport.getValue());
    strcpy(mqttConfig.username, custom_mqtt_username.getValue());
    strcpy(mqttConfig.password, custom_mqtt_password.getValue());
    strcpy(mqttConfig.topic, custom_mqtt_topic.getValue());
    mqttConfig.softpublicationinterval = atoi(custom_mqtt_softinterval.getValue());
    mqttConfig.hardpublicationinterval = atoi(custom_mqtt_hardinterval.getValue());
    strcpy(mqttConfig.sw0propertyname, custom_mqtt_sw0_alias.getValue());
    strcpy(mqttConfig.sw1propertyname, custom_mqtt_sw1_alias.getValue());
    saveConfig(mqttConfig);
  }

  // If we are connected to our host network, then we can continue into
  // production; if not, then let's reboot and go around again.
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

    // We have a WiFi connection, so configure the MQTT connection. 
    // We'll leave actually registering with the MQTT server until we
    // are in the loop().
    mqttClient.setServer(mqttConfig.servername, mqttConfig.serverport);

    // Time now to detect, set-up and initialise any connected sensors.

    Serial.print("Detected sensors: ");

    // Dallas one-wire temperature sensors
    /*
    DeviceAddress deviceAddress;
    char deviceName[20];
    DS18B20.begin();
    if ((DS18B20_DEVICE_COUNT = DS18B20.getDeviceCount())) {
      for (int i = 0; i < DS18B20_DEVICE_COUNT; i++) {
        if (DS18B20.getAddress(deviceAddress, i)) {
          sprintf(deviceName, DS18B20_NAME_FORMAT, deviceAddress[0], deviceAddress[1], deviceAddress[2], deviceAddress[3], deviceAddress[4], deviceAddress[5], deviceAddress[6], deviceAddress[7]);
          Serial.print(deviceName);
          Serial.print(" ");
        }
      }
    }
    */

    // AM2322 initialisation
    if (AM2322.begin()) {
      Serial.print("AM2322 ");
      AM2322.wakeUp();
      delay(AM2322_STARTUP_DELAY);
    }

    // SW0
    Serial.print(mqttConfig.sw0propertyname);
    Serial.print(" ");
    pinMode(GPIO_SW0, INPUT_PULLUP);

    // SW1
    Serial.print(mqttConfig.sw1propertyname);
    Serial.print(" ");
    pinMode(GPIO_SW1, INPUT_PULLUP);

    Serial.println();
    // End of sensor detection
    
  }
}

/**********************************************************************
 * Begin by checking that we have an active MQTT connection.  If not,
 * then try to make one. 
 * 
 * Once every CF_DEFAULT_MQTT_PUBLISH_SOFT_INTERVAL miliseconds read the sensors.
 * If the sensor values have changed from those most recently published
 * or CF_DEFAULT_MQTT_PUBLISH_HARD_INTERVAL has elapsed then update the configured
 * topic on the connected MQTT server.
 */
void loop() {
  static long mqttPublishSoftDeadline = 0L;
  static long mqttPublishHardDeadline = 0L;
  static char mqttStatusMessage[256];
  DeviceAddress deviceAddress;
  char deviceName[20];
  long now = millis();
  int dirty = false;

  // If we aren't connected to the MQTT server then try and make that
  // connection now. The connection attempt will loop indefinitely if
  // a connection cannot be made. Doing this in the loop eliminates
  // issues with transient server connection errors.
  if (!mqttClient.connected()) connect_to_mqtt(mqttConfig.servername, mqttConfig.serverport, mqttConfig.username, mqttConfig.password, moduleId);
  
  // At this point we have an MQTT connection, so we perform some
  // mandatory connection houskeeping
  mqttClient.loop();

  // Check if our time has come to publish
  if (now > mqttPublishSoftDeadline) {

    /*if (DS18B20_DEVICE_COUNT) {
      DS18B20.requestTemperatures();
      for (int i = 0; i < DS18B20_DEVICE_COUNT; i++) {
        if (DS18B20.getAddress(deviceAddress, i)) {
          sprintf(deviceName, DS18B20_NAME_FORMAT, deviceAddress[0], deviceAddress[1], deviceAddress[2], deviceAddress[3], deviceAddress[4], deviceAddress[5], deviceAddress[6], deviceAddress[7]);
          if ((int) jsonBuffer[deviceName] != (int) round(DS18B20.getTempC(deviceAddress))) { jsonBuffer[deviceName] = (int) round(DS18B20.getTempC(deviceAddress)); dirty = true; }
        }
      }
    }*/

    if (AM2322.isConnected()) {
      if (AM2322.read() == AM232X_OK) {
        if ((int) jsonBuffer["humidity"] != (int) round(AM2322.getHumidity())) { jsonBuffer["humidity"] = (int) round(AM2322.getHumidity()); dirty = true; };
        if ((int) jsonBuffer["temperature"] != (int) round(AM2322.getTemperature())) { jsonBuffer["temperature"] = (int) round(AM2322.getTemperature()); dirty = true; };
      } else {
        if ((int) jsonBuffer["humidity"] != SENSOR_UNDEFINED_VALUE) { jsonBuffer["humidity"] = SENSOR_UNDEFINED_VALUE; dirty = true; };
        if ((int) jsonBuffer["temperature"] != SENSOR_UNDEFINED_VALUE) { jsonBuffer["temperature"] = SENSOR_UNDEFINED_VALUE; dirty = true; };
      }
    }

    if (jsonBuffer[mqttConfig.sw0propertyname] != digitalRead(GPIO_SW0)) { jsonBuffer[mqttConfig.sw0propertyname] = digitalRead(GPIO_SW0); dirty = true; };
    if (jsonBuffer[mqttConfig.sw1propertyname] != digitalRead(GPIO_SW1)) { jsonBuffer[mqttConfig.sw1propertyname] = digitalRead(GPIO_SW1); dirty = true; };

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

      mqttPublishHardDeadline = (now + mqttConfig.softpublicationinterval);
    }
    mqttPublishSoftDeadline = (now + mqttConfig.hardpublicationinterval);
  }
}

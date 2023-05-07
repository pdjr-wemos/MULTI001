# MULTI001
 
__MULTI001__ is an MQTT wireless sensor module based on the
[Wemos D1]().

The module can be connected to the following sensor devices and will
report readings from the connected devices over MQTT.

**AM2320** is connects to the I2C terminals and supplies temperature
and humidity readings.

**DS18B20** connects to the DS terminals and supplies temperature
readings.

**SmartDim** is a movement/occupancy sensor which can be connected
to the SDIM terminals and supplies lux and occupancy readings.

Additionally, up to four active-low SPST switches can be connected
to the CO, SW0, SW1, SW2 and SW3 terminals to provide switch state
readings.

BROWN  - +12VDC power-in\
YELLOW - GND\
ORANGE - PIR (normally 0VDC, 12VDC pulse when movement detected)\
YELLOW - LUX (0 - 12VDC representing LUX level)\
 
__MULTI001__ obtains temperature data from a DS18B20 digital thermometer.

## Installation

Connect the sensor module's power input terminals to a DC supply
voltage between 9VDC and 24VDC.

The module has an intermittent maximum power consumption of 500mW.

## Configuration

When powered for the first time __MULTI001__ launches a wireless access
point with an SSID of the form "MULTISENSOR-*mac-address*".
The access point implements a captive portal web-interface that allows
the user of a WiFi client to enter the following configuration parameters
that will subsequently be used by the sensor in normal operation:

* host network SSID (this can be selected from a scan list)
* host network password
* MQTT server hostname or IP address
* MQTT username, and
* MQTT password that will be used to authenticate publish requests
* MQTT sensor name (defaults to the device MAC address)

## Operation

After configuration, a power cycle on __MULT001__ will cause it to
reboot as a WiFi client on the host network specified during
configuration.
If the module cannot connect to the specified WiFi hotspot within two
minutes, then it will revert to configuration mode.

Once a connection to the host WiFi network is made the module will
immediately begin reporting sensor readings to the MQTT server using the
topic "multisensor/*sensor-name*/status".

The MQTT message payload is a JSON string of the form:

{\
  "temperature": *temperature*, // in degrees celsius\
  "motion": *yesno*, // 0 says no motion detected, 1 says motion detected\
  "lux": *percent* // 0..100 of the sensor range\
}\

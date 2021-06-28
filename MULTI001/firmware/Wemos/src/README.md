# MULTI001
 
__MULTI001__ implements a wireless sensor module which reports
temperature, light-level and motion detection to an MQTT server.
The module is based on a Wemos Mini-D1 SOC.

__MULTI001__ was designed to obtain motion and light-level data from a
LuxControl SmartDim Sensor 2 (part number 86 454 523).
These 12VDC devices were top-quality sensors and were discontinued
when the manufacturer revised their product range.
At this time they could be obtained for a while at a franction of their
original, eye-watering, price.
The sensor supports the following connections:

BROWN  - +12VDC power in.
YELLOW - GND
ORANGE - PIR (normally 0VDC, 12VDC pulse when movement detected)
YELLOW - LUX (0 - 12VDC representing LUX level)
 
Interfacing these sensors to the Wemos board simply requires adjustment
of the output voltages on PIR and LUX through a resistance dividor.

The sensor module obtains temperature data from a DS18B20 digital
thermometer.

## Installation

Connect the sensor module's power input terminals to a DC supply
voltage between 9VDC and 24VDC.

The module has an intermittent maximum power consumption of 500mW.

## Configuration

When powered for the first time __MULT001__ launches a wireless access
point with an SSID of the form "MULTISENSOR-*mac-address*".
The access point incorporates a captive portal that allows the user of
a WiFi client to enter the following configuration parameters that will
subsequently be used by the sensor in normal operation:

* host network SSID (this can be selected from a scan list)
* host network password
* MQTT server hostname or IP address
* MQTT username, and
* MQTT password that will be used to authenticate publish requests.


## Operation

After configuration, a power cycle on __MULT001__ will cause it to
reboot as a WiFi client on the host network specified during
configuration.
The module will immediately begin reporting sensor readings to the MQTT
server using the topic "multisensor/*mac-address*/status".

The MQTT message payload is a JSON string of the form:

{\
  "temperature": *temperature*, // in degrees celsius\
  "motion": *yesno*, // 0 says no motion detected, 1 says motion detected\
  "lux": *percent* // 0..100 of the sensor range\
}\

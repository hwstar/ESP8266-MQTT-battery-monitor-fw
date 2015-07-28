**esp_8266_MQTT_io_node**
==========
This is an implementation of an MQTT Relay Node which runs natively on an ESP8266 ESP-03 module or other variant with enough free GPIO's.
Code is compiled using the toolchain referenced below.

**Features:**

Provides one relay channel on one GPIO or one bistable latching relay channel using 2 GPIO's and one button channel. The button channel can be linked to the relay for local control or isolated for separate use.
A GPIO can optionally be reserved for a connection state LED. GPIO ports are configurable in the source code. 

**Device Path**

The device path encompasses subtopics command and status. Commands are sent to $devicepath/command (which the nodes subscribes to.) All status messages are
published by the node on $devicepath/status except for the node configuration which is published on /node/info. The device path is set using the patching procedure described later.

**Control Messages**

Control messages are received by all nodes on /node/control. These are meant to be used to interrogate the nodes connected to the network, 
and perform other system-wide control functions.

One control message is currently supported: *muster*. This directs the node to re-send the node configuration information to /node/info. See the power on message below for further details


**Command Messages**

Command messages are sent using JSON encoding as follows:

{"command":"command from table below"} For commands without a parameter

{"command":"$COMMAND","param","$PARAM"} For commands with a parameter

Because of limitations with the Espressif JSON parser library, all numbers should be sent as text fields 
(i.e. quoted)

MQTT commands supported:

|Command| Description |
|--------| ----------- |
|on 	 | Turns relay on|
|off	 | Turns relay off|
|pulse   | Pulses relay for $PARAM milliseconds|
|toggle	 | Toggles relay state|
|query	 | Returns relay state|
|survey	 | Returns WIFI survey information as seen by the node|
|btlocal | $PARAM: 1 = link button to relay toggle, 0 = keep button separate|
|ssid    | Query or set SSID|
|restart | Restart system|
|wifipass| Query or set WIFI Password|
|cycle   | Start or stop relay cycling where $PARAM is the half cycle time in milliseconds

Notes:
* $ indicates a variable. e.g.: $COMMAND would be one of the commands in the table above.
* Sending an ssid, or wifi command without "parameter":"$PARAM" will return the current value.
* ssid, wifipass change not effective until next system restart

**Status Messages**

These are JSON encoded as follows:

Status messages which can be published:

* {"buttonstate":"depressed"}
* {"buttonstate":"released"}
* {"relaystate":"on"}
* {"relaystate":"off"}
* WIFI survey data in the following format: {"access_points":["$AP":{"chan":"$CHAN","rssi":"$RSSI"}...]} 

**Power on Message**

After booting, the node posts a JSON encoded message to /node/info with the following data:

|Field		| Description|
|-----      | -----------|
|connstate  | Connection state (online)
|device		| A device path (e.g. /home/lab/relay)|
|ip4		| The IP address assigned to the node|
|schema		| A schema name of hwstar_relaynode (vendor_product)|
|ssid       | SSID utilized|


The schema may be used to design a database of supported commands for each device.

Here is an example:

{"muster":{"connstate":"online","device":"/home/lab/relay","ip4":"$IP","schema":"hwstar_relaynode","ssid":"$SSID"}}

**Last Will and Testament**

The following will be published to /node/info if the node is not heard from by the MQTT broker:

{"muster":{"connstate":"offline","device":"$DEVICE"}}

Where $DEVICE is the configured device path

**Configuration Patcher**

NB: WIFI and MQTT Configration is not stored in the source files. It is patched in using a custom Python utility which is available on my github account as
a separate project:

https://github.com/hwstar/ESP8266-MQTT-config-patcher

Post patching allows the configuration to be changed without having sensitive information in the source files.

**Electrical Details**

The code is configured to be used with an ESP module with 1 uncommitted I/O for standard mode and 2 GPIO's for bistable latching mode. GPIO12 is the default. It can be reconfigured, but use of GPIO2 is problematic as that pin needs to be
high during boot, and that makes the electrical interface more complex.

The relay GPIO outputs are low true to be compatible with the bootloader's initial pin states. 
(This prevents the relay from pulsing at power on).

**Toolchain**

Requires the ESP8266 toolchain be installed on the Linux system per the instructions available here:

https://github.com/pfalcon/esp-open-sdk

toolchain should be installed in the /opt directory. Other directories will require Makefile modifications.

NB:Current Makefile supports Linux build hosts only at this time.

**LICENSE - "MIT License"**

Copyright (c) 2015 Stephen Rodgers 
Copyright (c) 2014-2015 Tuan PM, https://twitter.com/TuanPMT

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

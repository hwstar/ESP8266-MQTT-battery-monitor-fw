
#
# Quick and dirty data display script for the energy meter project
#
# Requires Python 2.7, Tkinter, and paho
#
# The command line takes 5 optional parameters:
#
# --host        host name       default mqtt
# --port        port number     default 1883
# --user        user name       default None
# --pw          password        default None
# --basetopic   base topic to use when sending commands and subscribing to status messages default /home/lab/acpowermon
#

__author__ = 'srodgers'

import paho.mqtt.client as mqtt
import json
import argparse
import Tkinter


#
# Convert unicode dict to dict of strings
#

def byteify(input):
    if isinstance(input, dict):
        return {byteify(key):byteify(value) for key,value in input.iteritems()}
    elif isinstance(input, list):
        return [byteify(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def request_data_received():
    global root
    root.after_cancel(cancel_id)


def request_data_timeout():
    print ("Request timed out\n")
    request_data()

# Request data from node

def request_data():
    global cancel_id
    global root
    client.publish(commandtopic, payload="{\"command\":\"query\"}")
    # Arm request timer
    cancel_id = root.after(5000, request_data_timeout)



# MQTT Connected callback

def on_connect(client, userdata, flags, rc):
    print("MQTT connected\n")
    # Subscribe to status topic
    client.subscribe(statustopic)
    # Request metering data from energy monitoring node
    request_data()





#
# MQTT Message received callback
# MQTT to xPL path
#
def on_message(client, userdata, msg):
    data = byteify(json.loads(msg.payload))
    if 'voltage' in data:
        request_data_received()
        voltage.configure(text=data['voltage'])
        current.configure(text=data['current'])
        power.configure(text=data['power'])

    # Re-request query data
	request_data()







#
# Main code
#
if __name__ == '__main__':
    root = Tkinter.Tk()
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--host",help="host name of mqtt server", default='mqtt')
    parser.add_argument("--port",type=int, help="port number of mqtt server",default=1883)
    parser.add_argument("--user",help="username", default=None)
    parser.add_argument("--pw",help="password", default=None)
    parser.add_argument("--basetopic", default='/home/lab/battmon')
    args = parser.parse_args()


    # Make command and status topics
    commandtopic = args.basetopic+'/command'
    statustopic = args.basetopic+'/status'

    # Instantiate MQTT client

    client = mqtt.Client()

    # Initialize MQTT callbacks
    client.on_connect = on_connect
    client.on_message = on_message

    # Set username and password if supplied
    if args.user is not None:
        client.username_pw_set(args.user, args.pw)

    # Connect to mqtt server
    client.connect(args.host, args.port, 60)
    print("MQTT Started\n")
    # Dedicate thread to mqtt client
    client.loop_start()
   

    #Set geometry
    root.geometry("800x200")
    root.columnconfigure(0, minsize=50)
    root.columnconfigure(1, minsize=50)

    #Set window title to base topic
    root.title(args.basetopic)

    # Set up display fields
    Tkinter.Label(master=root, text="mVDC" ).grid(row=0, column=1)
    voltage = Tkinter.Label(master=root, width=10, anchor=Tkinter.E, text='', relief=Tkinter.SUNKEN)
    voltage.grid(row=0, column=0)
    Tkinter.Label(master=root, text="mADC").grid(row=1, column=1)
    current = Tkinter.Label(master=root, text='', width=10, anchor=Tkinter.E, relief=Tkinter.SUNKEN)
    current.grid(row=1, column=0)
    Tkinter.Label(master=root, text="mW").grid(row=2, column=1)
    power = Tkinter.Label(master=root, text='', width=10, anchor=Tkinter.E, relief=Tkinter.SUNKEN)
    power.grid(row=2, column =0)
   
    # Enter Tk main loop
    cancel_id = None

    root.mainloop()






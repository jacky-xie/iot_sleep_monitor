import paho.mqtt.client as mqtt
import sys
import datetime
import time
from influxdb import InfluxDBClient

running = True

# Set up a client for InfluxDB
dbclient = InfluxDBClient('0.0.0.0', 8086, 'root', 'root', 'sleepsensor')

# Write sensor values to database.
def on_message(client, userdata, message):
    receiveTime = datetime.datetime.utcnow()
    val = float(message.payload)
    json_body = [
        {
            "measurement":message.topic,
            "time":receiveTime,
            "fields":{
                "value":val
            }
        }
    ]
    dbclient.write_points(json_body)

broker_address = "192.168.8.1"

client = mqtt.Client()
client.connect(broker_address)

client.on_message = on_message

client.subscribe("bpm")
client.subscribe("temperatureC");
client.subscribe("humidity");
client.subscribe("accel_x");
client.subscribe("accel_y");
client.subscribe("accel_z");

client.loop_forever()
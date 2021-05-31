# Decenter FMS Manager
Manager between Decenter module and Robotnik FMS

## Dependencies:

- pymongo
- inject==3.5.4
- paho-mqtt>=1.2
- msgpack-python>=0.4.8


## Environment variables

| Variable         | Default                         | Meaning               |
| ---------------- | ------------------------------- | --------------------- |
| `NODES_SELECTED`  | *203 403*                             | Nodes to disable       |
| `IMG_ENABLE_WAIT_TIME` | *120*                           | Time without AI image processing and replanning |
| `MQTT_HOST`      | *localhost*                     | mqtt broker hostname  |
| `MQTT_PORT`      | *1883*                          | mqtt broker port      |
| `MQTT_PROTOCOL`  | *4*                             | mqtt protocol version |
| `MQTT_KEEPALIVE` | *60*                            | mqtt keep alive       |
| `MQTT_TOPIC`     | */DECENTER/UC2/object-detector* | mqtt topic to read    |

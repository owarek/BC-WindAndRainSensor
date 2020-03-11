# BC-WindAndRainSensor
HARDWARIO node with wind vane (speed,gust and angle) and rain gauge. 
--------
Connection (to BC sensor module with three channels):  
Channel A: Anemometer to ground  
Channel B: Wind vane to ground  
Channel C: Rain gauge to ground  

# Climate Module support

by uncommenting `//#define CLIMATE_ENABLE` you can add Climate Module to the sensor

# MQTT2Influx configuration

To log wind data from MQTT to InfluxDB you need to add this configuration to /etc/bigclown/mqqt2influxdb.yml

```
  - measurement: rainfall
    topic: node/+/rainfall/+/mm
    fields:
      value: $.payload
    tags:
      id: $.topic[1]
      channel: $.topic[3]

  - measurement: wind
    topic: node/+/wind/+/+
    fields:
      value: $.payload
    tags:
      id: $.topic[1]
      channel: $.topic[4]

```

# Grafana dashboard

import file `grafana-dasboard-import.json` from the repository to display graphs for wind and rain.

![alt text](https://raw.githubusercontent.com/owarek/BC-WindAndRainSensor/master/img/IMG_20181106_155253.jpg)

--------
This is modified version of https://github.com/hubmartin/bcf-sigfox-wind-station. Thank you Martin :)

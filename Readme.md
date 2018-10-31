# AirQual aka "Air Quality"

This project was NPRA's attempt to create a custom air quality device running entirely on battery power.
The device needed to updated the measurement every hour (or 30 min) over NB-IoT (Narrowband IoT network) proveded by Telenor.


The air quality measurement device is using the PMS5003 high precision sensor.


![device1](./imgs/device1_cropped.jpg)
![device2](./imgs/device2_c.jpg)

## Hardware

This project consists of the following hardware components:

- PMS5003 air quality sensor
-

## Code

The code for this project is an Arduino sketch file. It's very simple and straightforward. Have a loop at [AirQual.ino](./AirQual.ino).


## Purpose

We wanted to experiment with some air quality sensors and check whether they gave a "good enough" result in comparison with much more expensive "high end" products.
And also measure both indoor an outdoor air quality. The outdoor air quality is very dependent on the season - winter season has a lot of small dangerous particles due to cars with dudded winter tires.


# Author

- Christian Skjetne (project / hardware / software)
- Tomas Levin (project / hardware / software)
- Asbjørn A. Fellinghaug (documentation only)

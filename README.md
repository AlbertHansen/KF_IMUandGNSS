# KF_IMUandGNSS

## Title
Kalman Filter for Velocity Approximation

## Participants
* Albert Berg Hansen - ES8
* Bjørn Højmose Grevenkop-Castenskiold - ES8
* Kristoffer Martinsen - ES8

## Description
This miniproject will combine the built-in IMU on the Arduino 33 IoT with <s>an external GNSS sensor</s> in order to estimate the velocity of a product. The IMU will provide information about acceleration (and velocity), <s>whereas the GNSS sensor will provide information about the relative positioning of the product</s>. This information will be processed through a Kalman filter, with the purpose of a ”better” state-estimation. The built-in IoT functionalities of the Arduino 33 IoT will be used tosend information, wherewith the state can be remotely observed/monitored. *Arduino Cloud* will be used as the GUI. In order to create functioning product, the sensors will be implemented as containers.

## Target User
Velocity estimation has many interesting applications however, in this mini project it is intended for velocity estimation on commercial drones. The intended user of the product is the drone manufacturers and indirectly the consumers who buy the drones.

## IDE
Arduino (Through VSCode)

## Sensors
* Built-in IMU on Arduino Nano 33 IoT: LSM6DS3.
* <s>GNSS sensor: NEO-M9N.</s>

## Lectures
* For the IMU: Lecture 3.
* Kalman filtering: Lectures 6-7.
* Programming containers: Lectures 8-9.
* <s>For the GNSS: Lecture 16.</s>
* General improvement of the code: Lectures 17-19

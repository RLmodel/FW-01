# FW-01
YUHESEN FW-01 Omnidirectional UGV ROS2 driver

# ROS2 Packages for Yuhesen FW-01 Mobile Robot

## Packages

This repository contains minimal packages to control the FW-01 robot using ROS. 


* yhs_can_control: a ROS wrapper around to monitor and control the fw-01 robot
* yhs_can_interfaces: fw-01 related message definitions
* fw_01_bringup: bringup all launch files related to fw-01

## Supported Hardware

* fw-01

## Basic usage of the ROS packages

1. Clone the packages into your colcon workspace and compile


    ```
    $ sudo apt-get update
    $ sudo apt-get install build-essential git cmake libasio-dev
    $ git clone https://github.com/RLmodel/FW-01.git
    ```

    

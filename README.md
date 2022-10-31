# TI mmWave Config

## Overview
The TI mmWave Config package is used to configure and control TI mmWave radar sensors.

The package was tested under ROS2 Galactic and Humble on Ubuntu 20.04 and Ubuntu 22.04 respectively.

## Installation
1. Install ROS Galactic or Humble.
2. Make sure that `colcon` is installed:

    ```
    sudo apt install python3-colcon-common-extensions
    ```

3. Clone this repo into your workspace:

    ```
    git clone 
    ```

4. Install dependencies and build the workspace:

    ```
    rosdep update
    rosdep install --ignore-src --from-paths src -y -r
    colcon build
    source install/setup.bash
    ```

## Usage
To launch the node using the parameters defined in `config/params.yaml` use the following command:

```
ros2 launch ti_mmwave_config ti_mmwave_config.launch.py
```

## Config files
 - **params.yaml** Contains the parameters for the ti_mmwave_config_node node.
 - **iwr6843isk.cfg** The configuration file to be sent to the IWR6843ISK sensor. This file is generated using [TI mmWave Demo Visualizer](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/).
 
## Nodes
### iwr6843isk_node

#### Parameters
- **`config_port`** (string, default: "None")
    
    The configuration port of the sensor.
- **`config_file`** (string, default: "None")
    
    The path of the sensor configuration file.
#### Services
- **`send_config`** (ti_mmwave_config_interface/srv/SendConfig)

    Establish a connection to the configuration port and send the configuration to the sensor.

- **`start_sensor`** (ti_mmwave_config_interface/srv/StartSensor)

    Start the sensor.

- **`stop_sensor`** (ti_mmwave_config_interface/srv/StopSensor)

    Stop the sensor.

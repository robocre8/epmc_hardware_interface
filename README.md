## EPMC (Easy PID Motor Controller) ROS2 Hardware Interface Package
This the **ROS2** Hardware Interface Package for the **`EPMC (Easy PID Motor Controller) Module`** (i.e **`L298N EPMC Module`** or any **`Custom EPMC Interface Board`**) with **ROS2** in a PC or microcomputer, after successful setup with the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application).

#

## How to Use the Package

#### Prequisite
- ensure you've already set up your microcomputer or PC system with ROS2

- install the `libserial-dev` package on your linux machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  ```

#

#### How to build the epmc_hardware_interface plugin package 
- In the `src/` folder of your `ros workspace`, clone the repo
  (or you can download and add it manually to the `src/` folder)
  ```shell
  git clone https://github.com/robocre8/epmc_hardware_interface.git
  ```

- from the `src/` folder, cd into the root directory of your `ros workspace` and run rosdep to install all necessary ros dependencies
  ```shell
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```
- build the `epmc_hardware_interface` package with colcon (in the root folder of your ros workspace):
  ```shell
  colcon build --packages-select epmc_hardware_interface
  ```
> [!NOTE]   
> The **epmc_hardware_interface** package will now be available for use in any project in your ros workspace.
> You can see example of how the use the `epmc_hardware_interface` in the `example_control_file`

#

#### Sample robot test
 - Please check out the [**`demo_bot`**](https://github.com/robocre8/demo_bot) package to see a proper sample of how the EPMC is used.

#

#### Serial port check for the EPMC Module
- ensure the **`Easy PID Motor Controller (EPMC) Module`** (i.e **`L298N EPMC Module`** or any **`Custom EPMC Interface Board`**), with the motors connected and fully set up for velocity PID, is connected to the microcomputer or PC via USB.

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on

- once you have gotten the **port**, update the **port** parameter in the **`<ros2_control>`** tag in the URDF's **`epmc_ros2_control.xacro`**

- you can the build and run your robot.

- Good Luck !!!

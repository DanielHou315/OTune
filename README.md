# OTune

A wheel Robot Odometry Tuning CLI Toolkit for VEX Robotics.

OTune is developed by Huaidian Daniel Hou ‘22 at the Haverford School as a Graduation project. The project is mentored by Mr. Myers and supported by the Haverford VEX Robotics Team 169. Some hardware for early development of this program is sponsored by Josiah Somani ‘22. 

# Table of Contents
- [OTune Hardware Requirements](#otune-hardware-requirements)
- [Install OTune](#install-otune)
  - [Environment](#environment)
  - [Configure Linux and ROS](#configure-linux-and-ros)
  - [Configure PROS Project](#configure-pros-project)
- [Use OTune](#use-otune)
- [Note](#note)
  - [ros_lib](#ros_lib-tune_rosserial-dependency)
- [License](#license)

# OTune Hardware Requirements

The project requires a few hardware components in order to function properly. In the case that some hardware is missing, I would like to attach the list of hardware required by this program:

- A functioning computer with Linux installed

- A Functioning robot brain with a working Micro USB port

- A long Micro-USB to USB-A cable
  
    [Micro USB Charger Cable, [15 Ft] Durable Extra Long USB 2.0 Charge Cord Compatible for Android/Windows/Smartphones/Samsung/HTC/Motorola/Nokia/LG/Tablet and More(Blue)](https://www.amazon.com/dp/B07LCFGKGF?psc=1&ref=ppx_yo2ov_dt_b_product_details)

- [A Vive Tracker package](https://www.amazon.com/HTC-Vive-Tracker-3-0-PC/dp/B08WFS5BMY/ref=sr_1_3?crid=MZ1DPQHBIZ0T&keywords=vive+tracker&qid=1654988920&sprefix=vive+tracke%2Caps%2C53&sr=8-3), which includes:
  
  - Tracker itself.
  
  - A USB-A to USB-C cable
  
  - A USB-C base
  
  - A USB-A Receiver

- [Lighthouses (base stations)](https://www.amazon.com/HTC-Vive-Base-Station-pc/dp/B01M01B92P/ref=sr_1_3?crid=R5424A66L3P5&keywords=vive+base+station&qid=1654988974&sprefix=vive+base+station%2Caps%2C52&sr=8-3) for the Vive Tracker
  
  - Either 1 or 2 base stations will work, but 2 base stations will result in more consistent performance.
    
# Install OTune

## Environment

1. Make sure you have a PC with Linux installed
   
   1. OTune is developed on Ubuntu 20.04 as of June 2022, but most other Linux distros should also work. 

2. [Install ROS Noetic](https://wiki.ros.org/melodic/Installation/Ubuntu) (the latest version of ROS as of OTune development)
   - Newer version of ROS should also work, but it is not a guarantee.

3. Create Your Catkin Workspace with [this official guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
   
   - Since your Catkin Workspace may not be the same as the one used during the time of development, the path to your Catkin Workspace is represented by the following for the rest of the guide

4. Install Steam

5. Install SteamVR

6. (Optional) Install VSCode 
   
   1. For intuitive access to some config files later in this guide, installing a GUI script editor like VSCode is recommended, although you can still use editors like vim or nano. 

## Configure Linux and ROS

1. Change SteamVR Settings
   
   ```
   code <YOUR_STEAM_PATH>/steamppps/common/SteamVR/resources/settings/default.vrsettings
   # In the section  "SteamVR"
   # Change Parameter "requireHMD" from "true" to "false"
   ```
   
   - In this case, “code” command is used to open file in VSCode. If you wish to use other editors, replace “code” with commands to open other editors.
   
   - <YOUR_STEAM_PATH> is usually
     
     ```
     ~/.local/share/Steam
     ```
     
       but it may vary depending on your Steam installation. 

2. Install ROS Package vive_ros 
   
   - Install with provided instructions on the Vive_ROS Github Page
     
       [https://github.com/robosavvy/vive_ros](https://github.com/robosavvy/vive_ros)
   
   - Replace the source code folder with the vive_ros folder in the cloned folder with the one from the provided package.
     
     - This package contains minor changes to the code so that the node can run at 100 Hz and print every received information to the terminal.

3. Install ROS Package rosserial
   
   1. Follow the instructions on their Github Page
      
       [https://github.com/ros-drivers/rosserial](https://github.com/ros-drivers/rosserial)
      
      1. Clone this to the src folder in your catkin workspace
         
         ```
         cd <YOUR_CATKIN_WS>/src
         git clone https://github.com/ros-drivers/rosserial.git
         ```
      
      2. Add otune_rosserial.launch to rosserial_vex_v5 launch folder
         
         ```
         cp <PROVIDED_OTUNE_ROSSERIAL_PATH>/rosserial_vex_v5/otune_rosserial.launch catkin_ws/src/rosserial/rosserial_vex_v5/launch
         ```

4. Install ROS Package OTune from the provided package
   
   1. Copy and paste the provided rosserial package to the catkin_ws directory
   2. Note: because this is a custom package, it is not open source anywhere, so make sure you always have a copy of the provided package securely stored locally and in the cloud. 

5. return to your main catkin workspace and compile these packages
   
   ```
   cd <YOUR_CATKIN_WS>/
   source devel/setup.bash
   catkin_make
   ```

6. Tips: every time a new instance of terminal is opened, if you want to run script or launch file on a complied package, run the following command
   
   ```
   cd <YOUR_CATKIN_WS>
   source devel/setup.bash
   ```
   
    Or you can add this to your .bashrc file so this is done automatically every time a new bash instance is started
   
   ```
   code ~/.bashrc
   # Add thie following line to the end of the bash file: 
   #
   #   source <YOUR_CATKIN_WS>/devel/setup.bash
   #
   # Close the .bashrc file and source it:
   source ~/.bashrc
   ```

## Configure PROS project

1. Configure your user access on USB devices so that you can upload programs to the Brain in Linux as well as receive messages from it. 
   
   1. Add the current user to the group “dialout” and “tty”
      
      ```
      usermod -aG dialout ${USER}
      usermod -aG tty ${USER}
      ```
   
   2. Restart the computer. 
   
   3. Now pros should be able to communicate with the brain

2. Install the template otune_rosserial in your PROS project

3. Configure your drive to follow the path when program starts or when it receives some form of signal. 
   
   1. Because the tuner uses the hypotenuse of planar velocity, this velocity is always positive. Therefore, if you are using left-right configuration of any kind, make sure both of your wheels run in the positive direction at any point during the tuning. If one wheel travels the opposite direction of the other wheel, the tuning will become less accurate. 

4. Configure your robot to transmit the appropriate string to OTune
   
   1. Format 1: “<TUNING_MODE>_<PROGRESS>_<LEFT_WHEEL_VELOCITY_RAW>_<RIGHT_WHEEL_VELOCITY_RAW>”
   2. Format 3: “<TUNING_MODE>_<PROGRESS>_<CENTER_WHEEL_VELOCITY_RAW>_<BACK_WHEEL_VELOCITY_RAW>”
   3. Format 3 (Not supported by otune_optimizer.py as of June 2022): “<TUNING_MODE>_<PROGRESS>_<LEFT_WHEEL_VELOCITY_RAW>_<RIGHT_WHEEL_VELOCITY_RAW>_<BACK_WHEEL_VELOCITY_RAW>”
   4. For Example, a Center_back wheel configuration robot driving 20% through the desired path, with center wheel rotational velocity 1400deg/s and back wheel rotational velocity 200deg
      1. The string should be “1_20_1400_200”

# Use OTune

1. Open a new terminal and start a roscore
   
   ```
   roscore
   ```

2. Start Steam 

3. Start SteamVR

4. Connect Tracker
   
   1. Turn on lighthouse and make sure they are properly installed and synced 
      
      [Installing the base stations](https://www.vive.com/us/support/vive/category_howto/installing-the-base-stations.html)
      
      b. Turn on tracker and connect to SteamVR
      
      Since we are not using a headset, we cannot and don’t need to do calibraiton

5. Start the vive node in a new terminal
   
   ```
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch vive_ros vive.launch
   ```
   
   - You should be able to see the output velocities in the terminal window

6. Turn on your PROS program on the VEX brain

7. Start the otune_rosserial node in a new terminal
   
   ```
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch rosserial_vex_v5 otune_rosserial.launch
   ```
   
   - You should be able to see the string messages in the terminal window
   - If not, check your USB connection

8. Run the otune_listener.py script in the otune package
   
   ```
   rosrun otune otune_listener.py
   ```
   
   - You should see prompts from the script telling you if the tuning is in progress and how much it has done
   - At the end of the tuning, the script will automatically tune the parameters based on the recorded bag file.

# Note:

## ros_lib (otune_rosserial dependency)

1. otune_rosserial is dependent on ros_lib, the package provided in rosserial to enable communication between the VEX Brain and the ROS system. 
2. If you are using a different version of ROS (the version tested is ROS Noetic as of June 2022)
- Make sure your rosserial and vive_ros packages are compiled under the currently installed ROS version.

- Use the following script to create a pros project.
  
  ```
  catkin_ws/src/rosserial_vex_v5/src/scripts/genproject.sh <YOUR_PROJECT_DIRECTORY>
  ```

- Delete the old autonomous.cpp, opcontrol.cpp, and initialize.cpp files, located in the src folder, as they were for older versions of PROS.

- Copy the following folders to the respective folders in the newly generated project
  
  ```
  otune_rosserial/include/otune
  otune_rosserial/src/otune
  ```

- Compile and see if there are error due to change in the version of ROS.

- If not, Congratulations!

- Edit the makefile and use the PROS conductor to make the new project a template
  
  - See official PROS Conductor documentation for detailed usecases
  
  - As of June 2022, PROS 3.3.3, this command is used to make the project into a template:
    
    ```
    pros make template
    ```

# License

This project is distributed under the MIT License. For details, read LICENSE. 

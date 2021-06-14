
# Mecanum Wheel Robot.  Lidar+ROS+Raspberry+Arduino+Matlab+Kalman 

Thanks to yoraish https://github.com/yoraish for the clear tutorial. 

## 1. Build the robot.

I have used the chasis of the robot model "4WD mecanum wheel mobile arduino robotics car" from Nexusrobot.com . The electronic components are: 
* Arduino Due
* Raspberry Pi 4 ( I think that also Raspberry Pi 3 Model B will work)
* Polulu Motors 37Dx70L with encoder 64 CPR and reductor 70:1
* Adafruit MotorShield v2
* Adafruit BNO055 IMU 9DOF
* YDLIDAR X4 lidar

## 2. Follow yoraish  tutorial
Go and check his project here https://github.com/yoraish/lidar_bot
I followed step by step his tutorial, I will report here just the modification maded to fit my setup.

### 2.2 - Installing ROS
I used this Image with ROS Melodic preinstalled. 
https://disk.yandex.ru/d/YfLc4stnCBljTA.

### 2.3 - Remotely connecting to ROS
Same as yoraish. My observer machine is based on Ubuntu 20.04 and I use this PC to run the Matlab GUI.

### 2.4 - Connecting to WiFi
I preferred to assign a static ip to the Raspberry like this:
Add your network to wpa_supplicant.conf
-  `sudo nano /etc/wpa_supplicant/wpa_supplicant.conf`

Add this line at the end of the file:

    network={
            ssid="your-networks-SSID"
            psk="your-networks-password"
        }
Open and modify /etc/dhcpcd.conf:

-  `sudo nano /etc/dhcpcd.conf`

Add this line at the end of the file matching your network setup. 

    interface wlan0
    static ip_address=192.168.11.13
    static routers=192.168.11.1
    
### 2.5 - Testing the Lidar
I am using the YDLIDAR X4 for this build. The first step is to install the necessary drivers. The driver is a ROS package.

-   `cd catkin_ws/src`
    
-   `git clone https://github.com/YDLIDAR/ydlidar_ros.git`
    
-   `cd ..`
    
-   `catkin_make`
    
-   `source devel/setup.bash`
    
-   Follow the directions from the repository, written below:
    
-   -   `roscd ydlidar_ros/startup`
-   -   `sudo chmod 777 ./*`
-   -   `sudo sh initenv.sh`
-   Note: every reboot, you need to `source devel/setup.bash` from `cd catkin_ws` or consider to include in **.bashrc** on home user folder.
    

Test the lidar with `roslaunch ydlidar_ros X4.launch`. Visualize the scans in Rviz, by adding the topic `/scan`.

### 2.6 - Final
Step 6 and 7 are the same as yoraish. I haven't performed step 8,9,10.

## 3. Install Matlab GUI
The robot works also without the Matlab application. You can control the robot sending message directly via ROS, but with the app is easier! 

Dependencies:
 - MATLAB Support Package for Raspberry Pi Hardware
 - Ros Toolbox
 
Install the application:
Open Matlab -> Apps-> Install App -> Install MatlabGUI.mlappinstall
Or run MatlabGUI.mlapp in /Matlab/src folder. 

## 4. Raspberry Setup
Copy the two script from Raspberry folder to your Raspberry in home position. 
In order to work with your setup **change the ip address**.  
This script are called by the Matlab app and executed in the raspberry to run ros packages, if you don't want to use the app, run it by terminal.

## 5. YouTube tutorial
I tried to explain the system on my YouTube channel here:
https://www.youtube.com/channel/UCOK905LMFaZ4vKWw7bKHyUA




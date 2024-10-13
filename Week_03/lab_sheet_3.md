# COMP0182 (Multi-Agent Systems): Lab Sheet 3

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1: Install and connect the Host PC (Laptop) and TurtleBot3 Burger (Mandatory before coming to the Lab)
Follow the offcial tutorial from
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Make sure you select the correct version of ROS: **Noetic**
![TurtleBot3](/Week_03/img/noetic.png)

### Key points

**3.1. PC Setup**

You can skip the introduction video first, it is talking about setting up turtlebot in Gazebo simulation world. You can go back and watch this after finishing this lab's work.


**1.1.1 & 1.1.2**: 

You should have finish these steps in previous lab sessions


**1.5**: 

- `ROS_MASTER_URI` specifies the location of your ROS master which means  all other machines will communicate with it. This can be either your PC or the Raspberry pi, but you have to make it consistent among all the machines.
- `ROS_HOSTNAME` Specifies the hostname or IP address of your machine, so it is unique for each machine.

**3.2. SBC Setup**

The offcial document gives an ideal situation of installing the OS on raspberry pi with a monitor. Unfortunately we might not have that much monitors to use as the same time. Therefore, here is another way to install it without monitor. And you may skip step **0.2.2** to **0.2.6**.

1. Download the Raspberry Pi Imaer from:
https://www.raspberrypi.com/software/

2. Choose `RASPBERRY PI 4` as device, `Ubuntu Server 20.045 LTS (64bit)` as operation system, and the SD card as storage.

![Imager](/Week_03/img/imager_0.png)

3. After clicking `NEXT`, select `EDIT SETTINGS` to modify the settings so that you can access it without the monitor.

![Edit](/Week_03/img/imager_1.png)

4. Set a unique hostname and username for your turtlebot, and also a simple password since you are going to use these info to access it without monitor. Also make sure you provide correct Wi-Fi SSID and password to it, otherwise you will not be able to access either.

![General](/Week_03/img/General.png)

5. Enable SSH, this is the key function to build the bridge between your PC and the raspberry. Then you are ready to start writing the imager to it.

![SSH](/Week_03/img/ssh.png)

6. Insert back the SD card to your pc, access through your file explorer, open file `network-config` by any editor and check if line `hidden` is set to `false`. If not, change it to `false`

7. Use `nmap` to get the IP Address of your Raspberry Pi. This is a tool to discover the IP Address of all the device connected to your Wi-Fi. It won't tell you which IP Address is your Raspberry Pi's but you can reason it out by comparing the change between before & after you enable your Raspberry Pi. To install it:

- MacOS
```bash
brew install nmap
```
- Ubuntu
```bash
sudo apt install nmap
```
Scan the network: Run the following command to scan the local network for connected devices (replace 192.168.0.0/24 with Lab's network IP range):
```bash
nmap -sn 192.168.0.0/24
```
Before Raspberry Pi run:
![Before](/Week_03/img/nmap1.png)
After Raspberry Pi run:
![After](/Week_03/img/nmap2.png)
It is clear that one device with IP Address of `192.168.0.108` appears after the Raspberry Pi enabled. So this is possibly the one for it.

**If you fail in this step, please ask one of the TAs to get a monitor, it will make the life easier**

8. Connect to your Raspberry Pi by SSH
Use following command to SSH to Raspberry Pi, from your PC:
```bash
ssh {USERNAME}@{IP ADDRESS}
```
You will be asked for password, type it and press ENTER, you should be directed to Raspberry Pi's Terminal

![Terminal](/Week_03/img/piterminal.png)

9. Install ROS Noetic on your Raspberry Pi. Just repeat what you did in Lab Sheet 1. The only difference is it is recommanded to install the base version rather than the desktop version due to its light feature.

    In case you don't want to go back, here is the command for it:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


After this, go back to the turtlebot offcial document and continue from **0.2.7. ROS Network Configuration**

**3.3 OpenCR Setup**

Just follow the instructions

**3.4 Hardware Assembly**

This can be skipped as the turtlebots have already been built.

**3.5 Bringup**

Just follow the instructions

**3.6 Basic Operation**

Just follow the instructions

Expected output:
Teleoperating of TurtleBot3 from your laptop.

Hints: 3 terminals from you pc, one for SSH, one for roscore, one fore teleop.

## Task 2: Sensing and Perception using Camera (Aruco Marker) and LiDAR (SLAM)
Follow https://github.com/JacksonChiy/turtlebot3_burger_auto_navigation/blob/main/auto_aruco_marker_finder/launch/aruco_marker_finder.launch

Expected output:
Identification of Aruco Markers.







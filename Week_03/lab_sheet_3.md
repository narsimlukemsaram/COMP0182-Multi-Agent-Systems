# COMP0182 (Multi-Agent Systems): Lab Sheet 3

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1 (Mandatory before coming to the Lab): Install and connect the Remote PC (your Laptop) and Target Robot (TurtleBot3 Burger)
Follow the official tutorial from
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Make sure you select the correct version of ROS: **Noetic**
![TurtleBot3](/Week_03/img/noetic.png)

### Key points

**3. 1. PC Setup**

You can skip the introduction video first, it is talking about setting up a TurtleBot3 in the Gazebo simulation world. You can go back and watch this after finishing this lab's work.

**1. 1. 1. Download and Install Ubuntu on PC  & 1. 1. 2. Install ROS on Remote PC**: 

You should have finished these steps in previous lab sessions, if not please do it.

Please follow the instructions in 
**1. 1. 3 Install Dependent ROS Packages and 1. 1. 4. Install TurtleBot3 Packages**

**1. 1. 5. Network Configuration**: 

- `ROS_MASTER_URI` specifies the location of your ROS master which means  all other machines will communicate with it. This can be either your PC or the Raspberry pi, but you have to make it consistent among all the machines.
- `ROS_HOSTNAME` Specifies the hostname or IP address of your machine, so it is unique for each machine.

**3.2. SBC Setup**

The official document gives an ideal situation for installing the OS on Raspberry Pi with a monitor. Unfortunately, we might not have that many monitors to use at the same time. Therefore, here is another way to install it without a monitor. And you may skip steps **0.2.2** to **0.2.6**.

1. Download the Raspberry Pi Imaer from:
https://www.raspberrypi.com/software/

2. Choose `RASPBERRY PI 4` as the device, `Ubuntu Server 20.045 LTS (64bit)` as the operation system, and the SD card as storage.

![Imager](/Week_03/img/imager_0.png)

3. After clicking `NEXT`, select `EDIT SETTINGS` to modify the settings so that you can access it without the monitor.

![Edit](/Week_03/img/imager_1.png)

4. Set a unique hostname and username for your turtlebot, and also a simple password since you are going to use this info to access it without a monitor. Also, make sure you provide the correct Wi-Fi SSID and password to it, otherwise you will not be able to access it either.

![General](/Week_03/img/General.png)

5. Enable SSH, this is the key function to build the bridge between your PC and the Raspberry. Then you are ready to start writing the image to it.

![SSH](/Week_03/img/ssh.png)

6. Insert the SD card to your pc, access through your file explorer, open file `network-config` by any editor, and check if line `hidden` is set to `false`. If not, change it to `false`

7. Use `nmap` to get the IP Address of your Raspberry Pi. This is a tool to discover the IP Address of all the devices connected to your Wi-Fi. It won't tell you which IP Address is your Raspberry Pi's but you can reason it out by comparing the change between before & after you enable your Raspberry Pi. To install it:

- MacOS
```bash
brew install nmap
```
- Ubuntu
```bash
sudo apt install nmap
```
Scan the network: Run the following command to scan the local network for connected devices (replace 192.168.0.0/24 with the Lab's network IP range):
```bash
nmap -sn 192.168.0.0/24
```
Before Raspberry Pi run:
![Before](/Week_03/img/nmap1.png)
After Raspberry Pi run:
![After](/Week_03/img/nmap2.png)
It is clear that one device with an IP Address of `192.168.0.108` appears after the Raspberry Pi is enabled. So this is possibly the one for it.

**If you fail in this step, please ask one of the TAs to get a monitor, it will make life easier**

8. Connect to your Raspberry Pi by SSH
Use the following command to SSH to Raspberry Pi, from your PC:
```bash
ssh {USERNAME}@{IP ADDRESS}
```
You will be asked for a password, type it and press ENTER, you should be directed to Raspberry Pi's Terminal

![Terminal](/Week_03/img/piterminal.png)

9. Install ROS Noetic on your Raspberry Pi. Just repeat what you did in Lab Sheet 1. The only difference is it is recommended to install the base version rather than the desktop version due to its light feature.

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

After this, go back to the Turtlebot official document and continue from **0.2.7. ROS Network Configuration**.

**3.3 OpenCR Setup**

Just follow the instructions.

**3.4 Hardware Assembly**

This can be skipped as the turtlebots have already been built.

**3.5 Bringup**

Just follow the instructions.

**3.6 Basic Operation**

Just follow the instructions.

Expected output:
Teleoperating of TurtleBot3 from your laptop.

Hints: 3 terminals from your pc, one for SSH, one for roscore, and one for teleoperation.

## Task 2 (During Lab Session): Finding Single and Multiple ArUco Markers using Camera
Follow [https://github.com/JacksonChiy/turtlebot3_burger_auto_navigation/blob/main/auto_aruco_marker_finder/launch/aruco_marker_finder.launch](https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder)

## Task 3 (During Lab Session): Naïve obstacle Avoidance using LiDAR
Follow https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/Week_03/turtlebot3_burger_auto_navigation/naive_obstacle_avoidance

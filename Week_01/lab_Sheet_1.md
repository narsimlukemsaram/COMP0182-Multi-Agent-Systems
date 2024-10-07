# COMP0182: Lab Sheet 1

## Lab Sheet 1: Installation of Ubuntu and ROS

For this course, everyone **must** use ROS Noetic. The best way to use it is to set up a Ubuntu 20.04 system on your computer. We strongly suggest a dual boot rather than a virtual machine. 

For online version for this file, please visit: [https://brick-slouch-eb1.notion.site/COMP0182-Real-world-Multi-agent-Systems-8876543ee87747a3aa30972c9f4631d9](https://www.notion.so/COMP0182-Real-world-Multi-agent-Systems-8876543ee87747a3aa30972c9f4631d9?pvs=21)

***Please read* the 1st & 2nd part: BitLocker & Disable Secure Boot, in Problem encountered (at the end of the file)** ***before you start your installation*** 

## Options for different computer:

- Option 1: [Install Ubuntu 20.04 Dual Boot with Windows 11 (High Recommended)](#task-1-1-dual-boot-setup-tested-on-windows-machine)
 
- Option 2: [Install Virtual Machine on macOS](#task-1-2-ubuntu-virtual-machine-on-macos)
 
- Option 3: [Install RoboStack on Windows 11 and macOS (Optional)](#task-1-3-optional-install-robostack)

### Task 1-1: Dual boot setup (Tested on Windows machine)

Adapted from:

 https://www.youtube.com/watch?v=-iSAyiicyQY&t=562s&ab_channel=KskRoyal

1. Pre-requisites:
- A Windows 10 or higher computer, with at least 8GB pendrive
- At least 25 GB free disk space for Ubuntu 20.04
- An USB drive that can be formatted

2. Disable Secure Boot

    For some computers, `Secure Boot` will still reject you from installing or booting new operation systems. In order to disable this:
        - Turn your computer off. Then, turn it back on and press the BIOS entry key during the boot process. This varies between hardware types, but is generally F1, F2, F12…
        - Find the **Secure Boot** option. If possible, set it to **Disabled**. It is usually found in the Security tab, Boot tab, or Authentication tab.
        - **Save and Exit**. Your system will reboot.
    
    Reference:
    
    [How to Disable UEFI Secure Boot to Dual Boot Any System](https://www.makeuseof.com/tag/disable-secure-uefi-dual-boot/)

3. Create new partition on your computer for Ubuntu
- Click ***Win + R*** and type ***cmd*** to open the terminal
- Type `diskmgmt.msc` and enter to open the Disk management
- Right click the last available partition and select ***Shrink volume***, shrink at least 25 GB
- An unallocated partition will come up once it is shrunk successfully

![Untitled](imgs/Untitled.png)

4. Download ***Ubuntu 20.04*** from the official website, select ***Desktop image***

[Ubuntu 20.04.6 LTS (Focal Fossa)](https://releases.ubuntu.com/focal/)

5. Download ***Rufus*** for making your USB drive bootable, select the ***Standard*** type with your computers platform

[Rufus - Create bootable USB drives the easy way](https://rufus.ie/en/)

![image.png](imgs/image.png)

For most if you, choose the standard one with platform Windows x64.

6. Format the USB drive and make it bootable
- Insert your USB into the port
- In the file Explorer, right click your USB and select ***Format*** option and follow the instructions
- Right click the ***Rufus*** and run as an administrator
- Leave everything default, click ***SELECT*** option and select the ISO image file just downloaded.
- Click ***START*** and follow the steps

![Untitled](imgs/Untitled%201.png)

7. Install Ubuntu dual boot
- Restart the computer. Press F11 (the button might be different for different computer brand, search for how to enter the ***Boot Menu*** for yours)
- Choose the USB drive as the boot device

8. Finish the Ubuntu installation setup
- Follow the instructions to go ahead. In ***Update and other software*** tab, it is recommended to select ***Normal installation*** and tick both of the two ***Other options***
- In ***Installation type*** tab, it is recommended to select ***Something else*** and continue
- In the next tab, you should see the disk info of your Windows system and the free space just created. Then select the free space and click on the ‘**+**’ button. Create a root partition (’**/**’) and allocate (recommended) 25 GB space for it, make sure you tick the same option as shown in the following figure

![Untitled](imgs/Untitled%202.png)

- Use the remaining free space to create a ***swap*** partition. It is recommended to allocate at least 4 GB for it

![Untitled](imgs/Untitled%203.png)

- Lastly, still in the installation type tab, change the device for boot loader installation to the same device as your root partition. After all, click ***Install Now*** and follow the instructions on your screen

![Untitled](imgs/Untitled%204.png)

After restarting your computer, the Ubuntu dual boot will be installed successfully

Reference:

 https://www.freecodecamp.org/news/how-to-dual-boot-windows-10-and-ubuntu-linux-dual-booting-tutorial/

### Task 1-2: Ubuntu Virtual Machine on MacOS

As mentioned in Lab Session 1, accessing an Nvidia GPU from within a virtual machine is often complex. However, since MacBooks do not use Nvidia GPUs, there is generally no significant difference between using a virtual machine or a dual boot setup on a MacBook in terms of GPU access (but you can still choose to use dual boot on Macbook for better performance and resolution). Additionally, installing a virtual machine is usually more user-friendly than setting up a dual boot environment.

1. Download a virtual machine platform
There are several options for virtual machine platform on the internet (Parallels Desktop, UTM…), you can do some research and select by your own. In this tutorial, we use VMware Fusion Pro for demonstration.
Go to the website:
https://blogs.vmware.com/teamfusion/2024/05/fusion-pro-now-available-free-for-personal-use.html
Scroll down and click “VMware Fusion Pro Download”
    
    ![image.png](imgs/image%201.png)
    
    You might be asked for creating an account, then just do it and follow the instructions.
    
2. Download Ubuntu 20.04 image
Similar as dual boot, you need to download an image file from the official website to install the operation system. 
[64-bit ARM (ARMv8/AArch64) server install image](https://cdimage.ubuntu.com/releases/20.04/release/ubuntu-20.04.5-live-server-arm64.iso)
Normally, we prefer to download the desktop image which can give us the GUI desktop directly after installation. Unfortunately, the desktop image for ARM64 is no more released currently. But we can still install the server image first, and install the GUI desktop manually.
3. Open thn virtual machine platform, in this case VMware gives:
    
    ![image.png](imgs/image%202.png)
    
    Simply drag the .iso file you just download here. Follow the instructions, there is no setting you need to change (you can if you want).
    
    ![image.png](imgs/image%203.png)
    
    ![image.png](imgs/image%204.png)
    
4. Finish the installation setup
There are not too many things to change from default in this step, if you want a quicker setup, just skip everything and select `Continue` or `Done` at each step.

    
    ![image.png](imgs/image%205.png)
    
    ![image.png](imgs/image%206.png)
    
    **Suggested optimisation on partition:**
    
    - **Reduce `/boot` Size:** You can safely reduce `/boot` to 500 MB - 1 GB.
    - **Reduce EFI Partition Size:** Consider reducing `/boot/efi` to around 300 MB to free up some space.
    - **Increase the Root (`/`) Size:** If you plan to use this virtual machine for more than basic tasks, use all the rest space to increase the root partition.
    
    ![image.png](imgs/image%207.png)
    
5. Finish the setup
After a few minutes, you will see `install complete` on the top left corner, and the system will turns to an update stage. Normally, this update stage will spend quite a long time. You can skip it by select `Cancel update and reboot`. 
    
    ![image.png](imgs/image%208.png)
    
6. Install GUI
After reboot, you will be lead to the Ubuntu Server terminal, as we need to use simulation for further work, a GUI desktop is necessary. First, type the username and password you just set to login and you will be free to use it.
    
    ![image.png](imgs/image%209.png)
    
    To install the GUI, run the following command in the terminal.
    
    ```bash
    sudo apt update
    sudo apt install ubuntu-desktop
    ```
    
    After installing and rebooting, the Ubuntu desktop in virtual machine has been successfully installed and ready to use.
    
    ```bash
    sudo reboot
    ```
    

7. (Optional) Make sure your virtual machine platform select the disk for startup
If the platform is still using the image(CD) file to start the virtual machine, you will be lead to install the operation system every time you reboot it. To avoid this:
    - Go to the setting menu of the virtual machine, and check the boot order in startup tab.
    
    ![image.png](imgs/image%2010.png)
    
    ![image.png](imgs/image%2011.png)
    
    - Select the `Hard Disk` to start up the virtual machine.
    - Run `sudo reboot` in terminal to see if it works.

### Task 1-3: (Optional) Install RoboStack

For some of you studying COMP0245, you should already have RoboStack installed on your computer. This is an alternative platform to Ubuntu virtual machine or dual boot. We strongly recommend you to use Ubuntu since it is more stable for ROS and Pybullet. But you can still try RoboStack since it is convenient, light and easy to install. For some parts of the script or software using in this module, there might be problems with RoboStack. If you meet some problems on happen on RoboStack and manage to fix it, please share it on moodle so that more people can use it.

1. Install conda on MacOS
    
    To get started with conda (or mamba) as package managers, you need to have a base conda installation. Please do *not* use the Anaconda installer, but rather start with [`miniforge`](https://github.com/conda-forge/miniforge) that is much more "minimal" installer. This installer will create a "base" environment that contains the package managers conda and mamba. After this installation is done, you can move on to the next steps.
    
    Download the installer using curl or wget or your favourite program and run the script.
    
    ```bash
    curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
    bash Miniforge3-$(uname)-$(uname -m).sh
    ```
    
    run `conda` or `mamba` to see if it is installed successfully.
    
2. Setting up the environment
    - Clone the `Multi-agent-system` repository
        
        ```bash
        git clone https://github.com/VModugno/Multi_agent_system.git
        ```
        
3. Install the Conda Environment:
    
    The environment_multi_agent_sys.yaml file includes all the dependencies required. To create the environment, run the below commands.
    

```bash
mamba env create -f environment_multi_agent_sys.yaml
mamba activate turtle_bot_env # or if it does not work try conda activate turtle_bot_env
```

Then the environment will change from `base` to `turtle_bot_env`.

![image.png](imgs/terminal.png)

### Task 2: Install ROS Noetic

Adapted from:

 https://wiki.ros.org/noetic/Installation/Ubuntu

Open the ***Terminal*** by click it in the application menu or shortcut **Ctrl + Alt + T**, then paste the following commands:

1. Setup your source.list
    
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
    
    It will ask for your password, it will **not** be shown on your screen, just type it and enter. As long as there is nothing shown (such as error message), it means the command has been run correctly
    

2. Set up your keys
    
    ```bash
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```
    

3. Installation

    ```bash
    sudo apt update
    ```

    It is recommended to install the Desktop-Full version:

    ```bash
    sudo apt install ros-noetic-desktop-full
    ```

Once its done, the ROS Noetic has been successfully installed

4. Environment setup
    
    Every time you want to use ROS from the terminal, you have to source this first:
    
    ```bash
    source /opt/ros/noetic/setup.bash
    ```
    
    This is necessary for **every** new terminal tab you open
    
    It will be convenient to automatically source the script by add it to `~/.bashrc` :
    
    ```bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    

5. Installing essential dependencies
    
    ```jsx
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```
    

6. After all, enter `roscore` to see if the ROS runs successfully

Reference:

 https://medium.com/@createwithabd/ros-noetic-installation-guide-on-ubuntu-20-04-388568d24bcf

### Task 3 Install VScode and Git

1. VScode
    - You can directly download the package to install Visual Studio Code from the official website:
    https://code.visualstudio.com/download
    Make sure you select the correct format of it. For Ubuntu OS, we need `.deb` file, and then select `x64` or `Arm64` based on your machine’s architecture

![image.png](imgs/image%2012.png)

- Then in terminal, use `cd` command to direct to the folder where you download the package to, and use the following command to install it.
    
    ```bash
     sudo dpkg -i [package]
    ```
    

![image.png](imgs/image%2013.png)

2. (Optional) Git
Normally, Git should have been installed to your Ubuntu system, run `git` to see if it is installed. If not, use the following command to install it.
    
    ```bash
    sudo apt install git-all
    ```
    

### Problem encountered or optional step

1. BitLocker

    BitLocker is a disk encryption utility built into some Windows machine that may be triggered during system partitioning or operations. If this happens you need to enter the BitLocker recovery key to re-enter Windows. We suggest you to simply turn it off before partitioning the disks.
    - Search ***Manage BitLocker*** from the search bar of Windows and open it. If you cannot find it, go to ***Settings → Control Panel → System and Security → BitLocker Drive Encryption***
    - Select ***Turn off BitLocker***

    (Optional) Enter the Recover key to unlock it
    In case you do not want to turn BitLocker off or the problem has been encountered, follow the instructions to find your recovery key. For example, login the same Microsoft account as your computer’s on the page https://account.microsoft.com/devices/recoverykey. And in the ***Device*** tab, you will find your recover key

    ![Untitled](imgs/Untitled%205.png)

    ![Untitled](imgs/Untitled%206.png)



2. Default boot
After dual boot is installed, the GRUB (GRand Unified Bootloader) will ask you the system you would like to boot and the default boot system will be Ubuntu. If you want to change the default selection to Windows, use the following instructions to modify the configuration file of GRUB
    
    ![Untitled](imgs/Untitled%207.png)
    
    - Open the terminal in Ubuntu
    - run following command
    
    ```bash
    sudo nano /etc/default/grub
    ```
    
    - Find this line
    
    ```bash
    GRUB_DEFAULT=0
    ```
    
    - Change it to the entry of Windows. For example, if Windows is the second (0, 1, 2) option in the GRUB menu, change it to:
    
    ```bash
    GRUB_DEFAULT=2
    ```
    
    - Save the file and exit the nano editor (press `Ctrl + O` to save, then `Ctrl + X` to exit).
    - Run the following command to update the GRUB configuration:
    
    ```bash
    sudo update-grub
    ```
    
    - Then reboot the computer
    
    ```bash
    sudo reboot
    ```
    

3. Low packages downloading speed
If you have two disks on your laptop, it is recommended to use the one with the higher read/write speed for installing Ubuntu. If you don't know which one is better, just use the one with Windows installed if the space is enough. Using a slower disk may lead to decreased performance when installing software and packages in Ubuntu.

4. ROS quick setup (Source https://github.com/fishros/fish_install)
We suggest you to use the official instructions in [Task2](https://www.notion.so/COMP0182-Real-world-Multi-agent-Systems-8876543ee87747a3aa30972c9f4631d9?pvs=21)  to install ROS, but if you find it difficult to understand, this tool can be used for a quick setup of ROS
- Open terminal in Ubuntu
- Run the following command and follow the instructions
    
    ```bash
    wget https://raw.githubusercontent.com/fishros/fish_install/main/install -O - | bash
    ```
    

5.  Optimize your hard drive for more partition space
In case you want more space in your laptop, check this tutorial for more information:
    
    [How to Dual Boot Windows 10 and Ubuntu – Linux Dual Booting Tutorial](https://www.freecodecamp.org/news/how-to-dual-boot-windows-10-and-ubuntu-linux-dual-booting-tutorial/)
    

TODO:

- [x]  secure boot disabled
- [x]  macos vm
- [x]  masos robostack?
- [ ]  windows with robostack
- [ ]  menu for different options
- [ ]  move secure boot part up
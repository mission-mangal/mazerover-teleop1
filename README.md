# **Installing VMware and Setting Up Ubuntu 22.04 on Windows**

This guide provides a **step-by-step** approach to installing **VMware Workstation Player** on Windows and setting up **Ubuntu 22.04** as a virtual machine. Every step is detailed to ensure a smooth installation process.

> ## **If You have already Ubuntu 22.04 alongside your windows or Using with Virtual Machine. Skip to Step 8.**

* * *

## **1\. System Requirements**

### **Minimum System Requirements**

- **Processor**: 64-bit x86 Intel or AMD CPU (Intel Core i3/i5/i7/i9 or AMD Ryzen)
- **RAM**: At least **4GB RAM** (Recommended: **8GB or more**)
- **Storage**: Minimum **20GB** of free disk space (Recommended: **50GB or more**)
- **Operating System**: Windows 10 (64-bit) or Windows 11 (64-bit)
- **Virtualization Support**: Ensure **Intel VT-x** or **AMD-V** is enabled in the BIOS

* * *

## **2\. Enabling Virtualization in BIOS**

### **How to Check If Virtualization is Enabled?**

1.  **Open Task Manager** (`Ctrl + Shift + Esc`).
2.  Navigate to the **Performance** tab.
3.  Click on **CPU** in the left panel.
4.  Look for "Virtualization":
    - If it says **"Enabled"**, you can proceed.
    - If it says **"Disabled"**, follow the steps below.

### **How to Enable Virtualization in BIOS?**

1.  **Restart your computer** and enter the BIOS.
    - Common keys to enter BIOS: `F2`, `F10`, `F12`, `DEL`, or `ESC`.
    - If unsure, check your PC manufacturer’s website for the correct key.
2.  Look for a setting called **Intel VT-x** (Intel CPUs) or **AMD-V** (AMD CPUs).
3.  **Enable it** and save the settings (`F10` to save and exit).
4.  Restart your system.

* * *

## **3\. Downloading VMware Workstation Player**

### **Steps to Download**

1.  **Go to VMware’s official website:**
    - [VMware Workstation Player Download Page](https://www.techspot.com/downloads/1969-vmware-player.html)
2.  Scroll down and select **Windows version**.
3.  Click **Download Now**.

* * *

## **4\. Installing VMware Workstation Player on Windows**

### **Step-by-Step Installation**

1.  **Locate the Downloaded File**
    - The file will be in your **Downloads** folder (`VMware-player-xx.x.x.exe`).
2.  **Run the Installer**
    - **Right-click** on the file and select **Run as Administrator**.
3.  **Begin Installation**
    - Click **Next** on the welcome screen.
4.  **Accept License Agreement**
    - Choose **I accept the terms in the license agreement** and click **Next**.
5.  **Choose Installation Directory**
    - Default: `C:\Program Files (x86)\VMware\VMware Player\`
    - Recommended: **Leave it as default** and click **Next**.
6.  **Choose Features**
    - Check **Enhanced Keyboard Driver** (for better VM experience).
    - Click **Next**.
7.  **User Experience Settings**
    - **Uncheck** the box for sending user experience data (optional).
    - Click **Next**.
8.  **Create a Desktop Shortcut**
    - Check **"Create a shortcut on the desktop"** (optional).
    - Click **Next**.
9.  **Start Installation**
    - Click **Install** and wait for the process to complete.
10. **Finish and Restart**

- Click **Finish** and restart your PC.

* * *

## **5\. Downloading Ubuntu 22.04 ISO**

### **Steps to Download Ubuntu**

1.  Go to the official Ubuntu website:
    - [Ubuntu 22.04 Download Page](https://releases.ubuntu.com/jammy/)
2.  Click on **Desktop Image - Ubuntu 22.04 LTS**.
3.  Click **Download**.
4.  The file will be named: `ubuntu-22.04-desktop-amd64.iso`.

* * *

## **6\. Creating a New Virtual Machine for Ubuntu**

### **Step-by-Step Guide**

1.  **Open VMware Workstation Player**
    - Click on **VMware Player** from the Start Menu or Desktop.
2.  **Create a New Virtual Machine**
    - Click **Create a New Virtual Machine**.
3.  **Select Installation Method**
    - Choose **"Installer disc image file (ISO)"**.
    - Click **Browse** and select `ubuntu-22.04-desktop-amd64.iso`.
    - Click **Next**.
4.  **Enter Virtual Machine Details**
    - **Full Name**: Enter any name (e.g., `UbuntuVM`).
    - **Username**: Enter a username.
    - **Password**: Set a strong password.
    - Click **Next**.
5.  **Specify Disk Size**
    - Recommended: **50GB** (Minimum: **20GB**).
    - Choose **Store virtual disk as a single file**.
    - Click **Next**.
6.  **Adjust CPU and RAM Settings**
    - Click **Customize Hardware**.
    - **Processor**: Set to **2 or more** (if your system supports it).
    - **RAM**: Allocate **at least 4GB** (Recommended: **8GB**).
    - **Display**: Enable **3D Acceleration** for better graphics.
    - Click **Close**, then **Finish**.

* * *

## **7\. Installing Ubuntu 22.04 on VMware**

### **Steps to Install Ubuntu**

1.  **Start the Virtual Machine**
    - Click **Play virtual machine**.
2.  **Select Language**
    - Choose **English** (or preferred language).
3.  **Click "Try Ubuntu" or "Install Ubuntu"**
    - Select **Install Ubuntu**.
4.  **Choose Keyboard Layout**
    - Default: **English (US)**.
    - Click **Continue**.
5.  **Select Installation Type**
    - Choose **Normal Installation**.
    - Check **Download updates while installing Ubuntu**.
    - Click **Continue**.
6.  **Partition Setup**
    - Choose **Erase disk and install Ubuntu**.
    - Click **Install Now** → **Continue**.
7.  **Set Time Zone**
    - Choose your location and click **Continue**.
8.  **Create User Account**
    - Enter your name, username, and password.
    - Click **Continue**.
9.  **Installation Begins**
    - Wait for the installation to complete (~10-20 minutes).
10. **Restart and Remove ISO**

- Click **Restart Now**.
- When prompted, press **Enter** to remove installation media.

* * *

## **8\. Installing ROS 2 on Ubuntu 22.04**

### **Steps to Install ROS 2**

1.  **Set up the Sources**
    
    ```bash
    locale  # check for UTF-8
    
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    locale  # verify settings
    
    
    sudo apt update && sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update
    ```
    
2.  **Add the ROS 2 Repository**
    
    ```bash
    sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    sudo apt-add-repository http://packages.ros.org/ros2/ubuntu
    sudo apt update
    ```
    
3.  **Install ROS 2 (Humble)**
    
    ```bash
    sudo apt install ros-humble-desktop
    ```
    
4.  **Source the Setup File**
    
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    

* * *

## **9\. Installing Essential Packages for ROS 2 Development**

* * *

### **1\. Install Additional ROS 2 Dependencies**

Install essential dependencies required for ROS 2 development:

```bash
sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### **Initialize rosdep**

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

* * *

### **2\. Install Gazebo for Simulation**

Gazebo is needed for robot simulation:

```bash
sudo apt install -y gazebo11 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

* * *

### **3\. Install RViz for Visualization**

RViz is required for visualizing robot models and sensor data:

```bash
sudo apt install -y ros-humble-rviz2 ros-humble-joint-state-publisher-gui
```

* * *

### **4\. Install Teleoperation Packages**

To control robots via keyboard:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

* * *

* * *

### **5\. Install Python3 Dependencies**

If your ROS 2 packages require additional Python dependencies:

```bash
sudo apt install -y python3-numpy python3-scipy python3-matplotlib
```

* * *

### **6\. Install Xacro**

Gazebo is needed for robot simulation:

```bash
sudo apt install ros-humble-xacro
```

* * *

### **7\. Install Git and Clone the Repository**

  Open a terminal in Ubuntu and install Git:
    
    ```bash
    sudo apt update && sudo apt install -y git
    ```
## **10\. Playing the Game**

### If You have pre-installed Ubuntu 22.04 alongside your windows or standalone (dual boot)

### Follow the Link

[https://github.com/mission-mangal/mazerover-teleop2](https://github.com/mission-mangal/mazerover-teleop2)

### If You have been using Ubuntu 22.04 Jammy with Virtual Machine

#### Open the Terminal

```bash
cd mazerover-teleop1
colcon build
source install/setup.bash
ros2 launch haruto_description visualisation_simulation.launch.py
```

#### Open Another Terminal

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

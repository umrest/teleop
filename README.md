# Robotic Exploration of Space Team - Teleoperation

Author: Jack Shelata  
Email: [jshelata@umich.edu](mailto:jshelata@umich.edu)  
Updated: April 18, 2019

Detailed documentation of these files can be found
[here](https://umrest.github.io/teleop/).

## Virtual Machine Setup

Last Updated: February 10, 2019 - Jack Shelata on MacBook Pro (MacOS 10.4.2)

In order to use [ROS](http://www.ros.org/about-ros/), you must be running a
ROS-compatible operating system on your computer. This guide will help you
download [VirtualBox](https://www.virtualbox.org/) and install a version of
[Linux](https://en.wikipedia.org/wiki/Linux) called
[Ubuntu](https://www.ubuntu.com/). The specific distribution of Linux that
we use is called **Ubuntu 16.04.5 LTS**. We use this version because it is
stable and able to be used with
[ROS Kinetic Kame](http://wiki.ros.org/kinetic) which is the version of ROS
that we use. *This should take about an hour to complete and requires an*
*internet connection.*

### General Setup

+ First, download and install VirtualBox for your machine:
  [VirtualBox Download](https://www.virtualbox.org/wiki/Downloads)
+ Download the Desktop, 64-bit versions of Ubuntu  
  Currently we are using 16.04.5:
  [Ubuntu Download](http://releases.ubuntu.com/16.04/ubuntu-16.04.5-desktop-amd64.iso)
+ To setup the Virtual Machine:
  + Open VirtualBox
  + In the top of the window, click **New**
  + Fill in the **Details** for the new machine
    + Enter a **Name** for the Virtual Machine
    + Either **Select** a new location for your VM to live or leave it
      as the **Default Location** (Make sure you have enough space to store
      the VM, it will take up nearly **40GB**)
    + Set the **Type** to **Linux**
    + Set the **Version** to **Ubuntu (64-bit)**
    + Click **Continue**
  + Specify the **Memory Size** for the new machine (RAM)
    + For the best performance, we recommend dragging the slider close to the
      end of the green.
    + Note that this will not affect your RAM when the VM is powered off
    + Click **Continue**
  + Set the **Hard Disk** Specifications (this is where all of the files you
    create on the VM will be stored)
    + Select **Create a virtual hard disk now** and click **Create**
    + Select **VDI**
    + Select **Dynamically Allocated** and click **Continue**
    + Name the folder **Ubuntu**
    + Change the size to **40GB** and click **Continue**
  + A Virtual Machine called **Ubuntu** should now appear on the left of the
    VirtualBox window
  + Start the Virtual Machine by **Double Clicking** on it or select it from
    the list and click **Start** at the top of the screen
  + A selection box will appear, **Click** the **Folder Icon** to the right
    of the dropdown list and select the **Ubuntu 16.04.5** .iso file that
    you downloaded earlier, then click **Start**
  + **Note**: The window may open very small. We will fix this later. If any
    of the buttons are cut off, use your mouse to click and drag the top of
    the window and drag to the left or right to see all of the options.
    (This is very annoying)
  + **Select** your desired language and click **Install Ubuntu** in the
    start up menu, click **Continue**
  + Select **Erase disk and install Ubuntu** and click **Install Now** (Don't
    worry, this will not delete any files outside of the Virtual Machine. The
    Machine can only touch the files inside of the 40GB space you gave it
    earlier)
  + Click **Continue** again
  + Type in your location **Detroit** and select
    **Detroit (Michigan, United States)**
  + Select your keyboard (Likely **Keyboard (US)** and **Keyboard (US)**) and
    click **Continue**
  + Fill in your user information for the user you will create on the virtual
    machine (this can be whatever you want) and click **Continue**
  + Wait for **Ubuntu** to be installed (this can take a while)
  + Click **Restart** and wait for the Virtual Machine to restart
    + If prompted to **Remove the installation medium and then press enter**
      just press enter
  + **Log In** using your password you set earlier
  + Congrats! You have an Ubuntu Virtual Machine

### Fix Small Screen Issue

+ With the Virtual Machine running, select **Devices** from the top your
  window (outside the Virtual Machine window) and click **Insert Guest**
  **Additions CD image...**
+ This will open a **Terminal** and run commands automatically
+ When the commands are finished running, the terminal will prompt
  you to press **Enter**
+ Now, when you resize the window, the desktop should adjust to fit
+ This setting should be saved permanently

### Increase Scale for Menu and Title Bars

+ To increase the size of the mouse, title, and menu bars, click the
  **Gear Icon** in the top right of the Ubuntu Desktop and select
  **System Settings...**
+ Click **Screen Display**
+ Slide the slider for **Scale for menu and title bars** to the right,
  click **Apply** and exit

### Turn On Bidirectional Clipboard

+ This will allow you to **Copy and Paste** things from your Operating
  System to the Virtual Machine
+ With the Virtual Machine running, select **Devices** from the top your
  window (outside the Virtual Machine window) and hover over **Shared**
  **Clipboard**.
+ Click **Bidirectional**
+ This setting should be saved permanently

### If You're Virtual Machine Runs Slow

If your VM runs very slow, you can try adjusting your settings to give it
access to more of your computer's resources. To do this:

+ With the Virtual Machine powered off, select the VM from the
  VirtualBox menu and click **Settings** at the top of the window
+ Click **System** to access system settings
+ Here you can adjust the amount of **RAM** the VM will have access to.
  If the VM runs slow, try sliding the slider closer to the edge of the
  green. We don't recommend going over the green line.
+ You can also give the VM more **CPU Cores** by clicking the **Processor**
  button and sliding the slider to the right. Again, We don't recommend
  crossing the green line
+ You can also select **Display** at the top of the window and increase the
  **Video Memory**. This slider does not have a limit on the green zone, but
  we don't recommend giving the VM access to all of your computer's video
  memory

## Setting Up Your Development Environment

Next, we will walk through some useful tools to add to your Virtual Machine
environment.

### Accessing/Installing Basic Tools

+ First, you will want to pin **Terminal** to your desktop launcher
  + Click the **Ubuntu Icon** in the top left corner of your screen
  + Search for **Terminal** and launch by clicking the **Terminal Icon**
  + This will add the **Terminal Icon** to your launcher along the left side
    of your desktop screen
  + Right-Click on the **Terminal Icon** and select **Lock to Launcher**
  + This will make the **Terminal** permanently available on the left side
    of your screen
+ You may also want to pin a **Text Editor** to the launcher for writing
  code. Feel free to download any **Text Editor** you like. This **Ubuntu**
  desktop comes with **Gedit** which is a simple, easy to use, text editor.
  You can search and pin it to your launcher the same way you pinned the
  **Terminal** application
+ You will also need to install **git** in order to clone this **GitHub**
  repository.
  + To install **git**, open up **Terminal** and run

  ```bash
  sudo apt-get install git
  ```

  + You can allow your VM to remember your git credentials by
  running the following:

  ```bash
  git config --global user.name "First_Name Last_Name"
  git config --global user.email "Your_Email@umich.edu"
  git config --global credential.helper cache
  git config --global credential.helper 'cache --timeout=3600'
  ```

### Installing ROS

**ROS** is the Robot Operating System. We use it to facilitate
communication between programs running on our Rover and Control Station
Computer. In a nutshell, **ROS** allows us to create a **Topic** on a
central message platform called the **ROS Master**. Then, a program can
**Publish** data to that topic and any program connected to the network
can **Subscribe** to the **Topic** and use the data constantly. This allows
us to easily send control data over the WiFi network from our control
computer to the Rover computer. We can also send data from one program
running on the Rover computer to another program on the same computer.

In order to build ROS nodes and run our existing software, you must install
**ROS**. Please follow
[this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install
**ROS** inside your virtual machine. We recommend choosing the
**Desktop-Full Install** when you get to section 1.4 in the tutorial. If
you have any questions, please [email me](mailto:jshelata@umich.edu).
Be sure to read through the entire tutorial as you execute the commands.
The last two commands in **Section 1.6** are not mandatory.
This will take a little while.

Once you are finished, please complete the **ROS tutorials**, in order to
get familiar with how **ROS** works. These will take a while, but are very
important. You can find the tutorials
[here](http://wiki.ros.org/ROS/Tutorials).

## Cloning this Repository

Once you have a good understanding of **ROS**. You can properly clone
this repository. This will be slightly different than cloning a normal
repository so be sure to follow all of these directions.

+ Create a catkin workspace

  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  ```

+ Next, clone the remote repository

  ```bash
  git clone https://github.com/umrest/teleop.git
  ```

+ Setup Bash (this is redundant if you've added this source
  to your `.bashrc` as suggested in the **ROS** installation tutorial):

  ```bash
  source /opt/ros/kinetic/setup.bash
  ```

+ Get run `catkin_make`:

  ```bash
  cd ~/catkin_ws/
  catkin_make
  ```

+ Whenever you begin working in the catkin workspace run:

```bash
cd ~/catkin_ws
source devel/setup.bash
```

This should add the repository to the catkin workspace properly. If you
encounter any build issues please shoot me an
[email](mailto:jshelata@umich.edu).

## Other Notes

**Router Password for Competition**: `Rusty2018`
**Nuk Password**: `rusty` or `Rusty`

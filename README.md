# DCSC_Robots
Repo of the Robotics Group at the DCSC, Faculty of 3ME, TU Delft

DCSCRobo
---------------

Welcome to the one and only <font color=ff00f0>DCSCRobo repository</font>.

DCSCRobo is a collection of the projects at the Distributed Robotics Lab of TU Delft's, Delft Center for Systems and Control

The Distributed Robotics Lab has many Robot Platforms that work on the Robot Operating System and a TelosB WSN dependent on TinyOS for Communication

The lab setup includes:

    Optitrack Motive System
    iRobot Create
    Spheros
    Hovercrafts
    TelosB WSN
  
##Setup
To download the repo, simply clone it:

    $ git clone https://github.com/zang3t5u/DCSC_Robots.git

Then initialize and update the submodules:

    $ git submodule init

    $ git submodule update

Install the tinyOS dependencies:

    [sudo] apt-get install automake
    [sudo] apt-get install nescc
    [sudo] apt-get install default-jdk
    [sudo] apt-get install msp430-gcc

    cd lib/tinyos-main/tools/
    ./Bootstrap
    ./configure
    make
    sudo make install

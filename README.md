RoboTower
=========

[RoboTower](http://airlab.ws.dei.polimi.it/index.php/RoboTower) is a Tower Defense Hi-CoRG (Higly Competitive RoboGame) based on Meccano's Spykee robot.

Aim of this project is to develop a strategic realtime Robogame, based on the idea of a "Tower defense" (a game in which the player has to defend an object, that is the "tower", from an attack pursued by the enemy). A mobile robot should try to ruin down the enemy tower in a given amount of time. The human player has to prevent this by acquiring obstacles and putting them on the game field, either at the beginning of the game, or during the game, by buying obstacles from a computer dispenser.

Usage
-----

The code runs on the Linux operating system.

The firmware for the STM32F4 Discovery Board added to the robot is into the directory named FirmwareSpykee, and is written for [ChibiOS](www.chibios.org).

You need to install the [ROS middleware](www.ros.org) to compile and execute the code in this repository. Furthermore, some part of the code need the OpenCV libraries to be installed on your system (this should be automatically done by ROS) and the ANN ROS package (you can download it [here](http://www.ros.org/wiki/ann). The GUI is written with the Qt4 libraries, and needs also the Qt Mobility package to play sounds. Mr. Brian needs `flex` and `bison` to generate the parsers for the configuration files.

To use the code, you need the Spykee robot together with some sonars and leds that have been added later to our robot. If you have a different hardware (robot and\or sonar interface card), you need to adapt the software.

Compiling
---------

To compile the PC-side code, run make from the main directory. To compile the firmware, run make inside the FirmareSpykee directory (remember to change the Makefile with the path of ChibiOS sources!)
* You should have configured ROS so that the directory in which you clone the repository is part of the ROS workspace.
* If compilation fails, try to remove the Cmake cache from the package directory.
Further information aboud the code, and the complete installation manual is available [here](http://airlab.ws.dei.polimi.it/index.php/RoboTower) in italian (go to the Discussion page and download the documentation)

Executing
---------

To start the software, issue

    roslaunch spykee.launch

in the main directory (the one in which you cloned the repo).

Copyright & contacts
--------------------

This project is distributed under the GNU GPL license, version 2.

(C) 2012 Marcello Pogliani, Davide Tateo

(C) 2012 Politecnico Di Milano

For further information, please contact davide.tateo90@gmail.com
 

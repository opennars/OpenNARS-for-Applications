# ONA ROS Package

This is ros package for integrating the NARS Reasoning System with the ROS framework.

### PREPARE

To use ONA-ROS package, you need to install and configure the ONA system in your machine. 

Please follow these simple rules for installing ONA. 

```bash
$ git clone https://github.com/opennars/OpenNARS-for-Applications.git
$ sh libbuild.sh
```

### INSTALLATION

```bash
$ cd ~/catkin_ws/src
$ git clone https://<uri>/repo.git
$ cd ~/catkin_ws
$ catkin_make
```
### USAGE

```bash
$ rosrun ona-ros ona-ros_node
```
------------------

This packages is released under MIT License. 

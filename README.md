# Installation

These instructions assume you are in ROS noetic and ubuntu 20.04.

To have this working you need to first of all install the ROS dependencies.

```
rosdep install --from-paths src --ignore-src -r -y
```

Then, you will need to install the udev rules for the realsense cameras.
You can do this by installing the udev rule package from the non-ROS apt repo.

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
sudo apt update
sudo apt install librealsense2-udev-rules
```

You should now reload the udev rules (or reboot):

```
sudo udevadm control --reload-rules
sudo udevadm trigger
```


Now it should be working.


## How to get librealsense tools
The ROS librealsense installation does not include the tools (realsense-viewer, rs-enumerate-devices...)

To have them you can clone the librealsense repo and build it, **WITHOUT INSTALLING IT**.

So:

```
https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build 
cd build
cmake ../
make
```

You can now find the binaries for the tools in the 'tools' folder


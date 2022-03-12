# exhibit-lidar
lidar scanning exhibit code and resources
Uses Intel Realsense Lidar L515 sensor


## Python Dependencies
- pyrealsense2  2.50.0.3812  
- pyglet        1.5.21
- numpy         1.22.1


## On the exhibit linux computer
Lidar program is stored in:
    /home/msclidar/Documents/lidar/lidar.py

The terminal command to run it is:
    python3 /home/msclidar/Documents/lidar/lidar.py

The program is set to auto-start.  In the "Activities" menu search for "startup applications" and in that window you will see and entry set to run this above command.

The desktop RS icon just runs the above terminal command.


## Optional utilities
Intel realsense comes with some pre-made binaries for testing the sensors.
[https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
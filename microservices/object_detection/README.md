# Purpose

This repository contains only example files that are used **exclusively** for the course DIT638 (Cyber Physical Systems and Systems of Systems) in spring 2019.

## Install OpenCV on Raspberry Pi 3 (rpi3)

_Refer to [this tutorial](https://www.pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/) for details._

1.SSH into the Raspboan OS of rpi3:

```shell
    $ ssh -p 2200 pi@192.168.8.1
```

2.Install necessary packages:

```Shell
    $ sudo apt-get update
    $ sudo apt-get upgrade
    $ sudo apt-get install -y \
        libtiff-dev libavcodec-dev libavformat-dev \
        libswscale-dev libv4l-dev libxvidcore-dev \
        libx264-dev libgtk-3-dev libcanberra-gtk* \
        libatlas-base-dev gfortran python3-dev \
        libjpeg-dev libpng-dev unzip
```

3.Expand the swapsize of the rpi3 to avoid potential failure (hanging) during compilation

```Shell
    $ sudo nano /etc/dphys-swapfile
        # Find the CONF_SWAPSIZE value (100 by default) and change it to 2048
        # Run the following commands to enable the change
    $ sudo /etc/init.d/dphys-swapfile stop
    $ sudo /etc/init.d/dphys-swapfile start
```

4.Download OpenCV source code from <https://opencv.org/releases.html> (make sure that [your rpi3 is connected to the Internet](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md) first):

```Shell
    $ wget https://github.com/opencv/opencv/archive/4.0.1.zip
    $ cd opencv-4.0.1/
    $ mkdir build && cd build
```

5.Compile the code (**this would take quite long time, ~ 30min-1h**)

```Shell
    $ cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D BUILD_EXAMPLES=OFF .. # Don't forget the two dots at the end
    $ make -j4 # -j4 specifies 4 cores for compilation. If you have compile errors or your Raspberry Pi hangs/freezes you can try without the -j4  switch which can eliminate race conditions.
    $ sudo make install
    $ sudo ldconfig # update the information of the necessary links to the most recent shared library, namely OpenCV
```

6.Change the swapsize value back to default (same as Step 3.)

7.Make symbolic link of the the header file folder:

```Shell
    $ sudo ln -s /usr/local/include/opencv4/opencv2 /usr/local/include/opencv2
```

8.The OpenCV library is now installed on your rpi3.

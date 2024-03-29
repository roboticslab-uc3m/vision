## Installation from Source Code

### Install the Software dependencies:

- [Install CMake 3.16+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-cmake.md/)
- [Install PCL with VTK >= 5](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-pcl.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-ycm.md/)
- [Install YARP 3.8+ and depthCamera](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-yarp.md#install-additional-yarp-device-depthcamera-ubuntu)
- [Install OpenCV](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-opencv.md/)

### Install the Software on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/vision.git  # Download vision software from the repository
cd vision && mkdir build && cd build && cmake ..  # Configure the vision software
make -j$(nproc)  # compile
sudo make install  # Install :-)
sudo ldconfig # For libTravisLib.so
```

You may want to (from the same build dir):
```bash
echo "export ROBOTICSLAB_VISION_DIR=`pwd`" >> ~/.bashrc
```

For additional VISION options use ccmake instead of cmake.

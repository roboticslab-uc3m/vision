## Installation from Source Code

### Install the Software dependencies:

- [Install CMake 3.5+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install PCL with VTK >= 5](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-pcl.md/)
- [Install OpenNI2 & NiTE2](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openni-nite.md/)
- [Install YCM 0.10+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 2.3.70+ and OpenNI2DeviceServer](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install OpenCV](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-opencv.md/)
- [Install color-debug](https://github.com/roboticslab-uc3m/color-debug)

### Install the Software on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/vision.git  # Download vision software from the repository
cd vision && mkdir build && cd build && cmake ..  # Configure the vision software
make -j$(nproc)  # compile
sudo make install  # Install :-)
```

You may want to (from the same build dir):
```bash
echo "export ROBOTICSLAB_VISION_DIR=`pwd`" >> ~/.bashrc
```

For additional VISION options use ccmake instead of cmake.

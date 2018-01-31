## Installation from Source Code

### Install the Software dependencies:

- [Install PCL with VTK >= 5](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-pcl.md)
- [Install OpenNI2 & NiTE2](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openni-nite.md)
- [Install Yarp and OpenNI2DeviceServer](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md)
- [Install KDL](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-kdl.md)
- [Install OpenCV](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-opencv.md)

Additionally, this project depends on YCM to download and build external packages. Although this process is intended to run automatically during the CMake configuration phase, you may still want to install YCM and said packages by yourself. In that respect, refer to [Install YCM](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md) and to the installation guides of any package listed below:

- [color-debug](https://github.com/roboticslab-uc3m/color-debug)

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


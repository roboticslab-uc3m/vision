## Installation from Source Code

### Install the Software dependencies:

- [Install PCL](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_pcl.md)
- [Install OpenNI2 & NiTE2](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_openni_nite.md)
- [Install Yarp and OpenNI2DeviceServer](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_yarp.md)
- [Install KDL](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_kdl.md)
- [Install OpenCV](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_opencv.md)
- [Install MBROLA VOICES](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_mbrola.md)
- [Install Speech Recognition](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_speech_recognition.md)

### Install the Software on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/vision.git  # Download vision software from the repository
cd vision; mkdir build; cd build; cmake ..  # Configure the vision software
make  # Compile
sudo make install  # Install :-)
```

You may want to (from the same build dir):
```bash
echo "export ROBOTICSLAB_VISION_DIR=`pwd`" >> ~/.bashrc
```

For additional VISION options use ccmake instead of cmake.


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
git clone https://github.com/roboticslab-uc3m/teo-head.git  # Download teo-head software from the repository
cd teo-head; mkdir build; cd build; cmake ..  # Configure the teo-head software
make  # Compile
sudo make install  # Install :-)
```

You may want to (from the same build dir):
```bash
echo "export TEO_HEAD_DIR=`pwd`" >> ~/.bashrc
```

For additional TEO options use ccmake instead of cmake.

Done! You are now probably interested in one of the following links:
  - teo_head_post_install
  - teo_head_environment_variables

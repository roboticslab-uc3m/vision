## Installation from Source Code (Ubuntu 15.04)

### Install the Software (Vision)

First install the depencencies:

```bash
# PCL, OpenNI and Xtion Pro Live OpenNI driver
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-dev pcl-tools libopenni-sensor-primesense0  # libpcl-dev & pcl-tools instead of libpcl-all-dev as of Dic/2015
```

```bash
# OpenNI2 and Xtion Pro Live OpenNI2 driver
sudo apt-get install git libusb-1.0-0-dev libudev-dev
sudo apt-get install openjdk-6-jdk  # if not using other java version
sudo apt-get install freeglut3-dev
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/OpenNI2.git  # Fork with working copy of Xtion Pro Live OpenNI2 driver.
cd OpenNI2 && make
sudo ln -s $PWD/Bin/x64-Release/libOpenNI2.so /usr/local/lib/  # $PWD should be /yourPathTo/OpenNI2
sudo ln -s $PWD/Bin/x64-Release/OpenNI2/ /usr/local/lib/  # $PWD should be /yourPathTo/OpenNI2
sudo ln -s $PWD/Include /usr/local/include/OpenNI2  # $PWD should be /yourPathTo/OpenNI2
sudo ldconfig
```

```bash
# NiTE2.2
cd  # go home
mkdir -p src; cd src  # make $HOME/src if it doesn't exist; then, enter it
wget https://sourceforge.net/projects/roboticslab/files/External/nite/NiTE-Linux-x64-2.2.tar.bz2
tar xvf NiTE-Linux-x64-2.2.tar.bz2
sudo ln -s $PWD/NiTE-Linux-x64-2.2/Redist/libNiTE2.so /usr/local/lib/  # $PWD should be /yourPathTo/NiTE-Linux-x64-2.2/..
sudo ln -s $PWD/NiTE-Linux-x64-2.2/Include /usr/local/include/NiTE-Linux-x64-2.2  # $PWD should be /yourPathTo/NiTE-Linux-x64-2.2/..
sudo ldconfig
```

```bash
# YARP
sudo apt-get install build-essential cmake cmake-curses-gui libace-dev
sudo apt-get install libgsl0-dev  # Needed for creating YARP lib_math
sudo apt-get install libgtkmm-2.4-dev  # Needed for YARP GUIs, e.g. yarpview
sudo apt-get install libjpeg8-dev   # Needed for mjpeg_carrier
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/robotology/yarp
cd yarp && mkdir build && cd build
cmake .. -DCREATE_SHARED_LIBRARY=OFF -DCREATE_LIB_MATH=ON -DCREATE_GUIS=ON -DCREATE_OPTIONAL_CARRIERS=ON -DENABLE_yarpcar_mjpeg_carrier=ON -DCREATE_DEVICE_LIBRARY_MODULES=ON -DENABLE_yarpmod_OpenNI2DeviceServer=ON -DOPENNI2_INCLUDE_LOCAL=/usr/local/include/OpenNI2/ -DOPENNI2_LIBRARY=/usr/local/lib/libOpenNI2.so -DNITE2_INCLUDE_LOCAL=/usr/local/include/NiTE-Linux-x64-2.2 -DNITE2_LIBRARY=/usr/local/lib/libNiTE2.so -DYARP_USE_QT5=OFF  # YARP_USE_QT5 can induce errors as of 02/2016. Force it off fixes #15.
make
sudo make install
sudo ldconfig
# you can now launch "yarpdev --device OpenNI2DeviceServer" from /YOUR_PATH_TO/NiTE-Linux-x64-2.2/Redist
```

```bash
# KDL
sudo apt-get install libeigen3-dev
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics/orocos_kdl; mkdir build; cd build
cmake ..
make -j3;  sudo make install; sudo ldconfig; cd  # go home
```

```bash
# OpenCV
sudo apt-get install libopencv-dev
```

```bash
# voice: Install MBROLA VOICES
sudo apt-get install espeak libespeak-dev
sudo apt-get install mbrola-en1
sudo apt-get install mbrola-es1
```

```bash
# Speech Recognition: 
sudo apt-get install libgstreamer1.0-dev
sudo apt-get install libgstreamer-plugins-base1.0-dev
sudo apt-get install autoconf
sudo apt-get install bison

# Installation sphinxbase
cd
cd repos
git clone https://github.com/cmusphinx/sphinxbase.git
cd sphinxbase
./autogen.sh
./configure
make -j4
make check # to be sure that all is ok
sudo make install

# Installation pocketsphinx
cd
cd repos
git clone https://github.com/cmusphinx/pocketsphinx
cd pocketsphinx
./autogen.sh
./configure
make -j4 clean all
make -j4 check
sudo make install

# We need to configure the path to look for shared libaries
echo "export LD_LIBRARY_PATH=/usr/local/lib" >> ~/.bashrc
echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig" >> ~/.bashrc
echo "export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0" >> ~/.bashrc

# To verify that GStreamer can find the plugin. You should get a large amount of output, ending with something like this:
# decoder             : The underlying decoder
#                        flags: readable
#                        Boxed pointer of type "PSDecoder"

gst-inspect-1.0 pocketsphinx

# to test that all is runnin ok
sudo modprobe snd_pcm_oss
pocketsphinx_continuous -inmic yes
```

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


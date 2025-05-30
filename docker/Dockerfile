ARG UBUNTU_TAG=latest
FROM ubuntu:${UBUNTU_TAG}

ARG CORES=1
ARG SWIG_TAG=4.2.1
ARG OPENCV_TAG
ARG GCC_TAG
ARG CLANG_TAG

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
        ${GCC_TAG} \
        ${CLANG_TAG} \
        cmake \
        wget \
        unzip \
        ccache \
        libpcl-dev \
        bison \
        googletest \
        python3-setuptools && \
    wget -O swig.zip https://github.com/swig/swig/archive/v${SWIG_TAG}.zip && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_TAG}.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_TAG}.zip && \
    unzip swig.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    cmake -S swig-${SWIG_TAG} -B swig-${SWIG_TAG}/build && \
    cmake --build swig-${SWIG_TAG}/build -- -j$CORES && \
    cmake --install swig-${SWIG_TAG}/build && \
    cmake -S opencv-${OPENCV_TAG} -B opencv-${OPENCV_TAG}/build \
        -DOPENCV_EXTRA_MODULES_PATH=opencv_contrib-${OPENCV_TAG}/modules \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DOPENCV_ENABLE_NONFREE=ON && \
    cmake --build opencv-${OPENCV_TAG}/build -- -j$CORES && \
    cmake --install opencv-${OPENCV_TAG}/build && \
    rm swig.zip opencv.zip opencv_contrib.zip && \
    rm -rf swig-${SWIG_TAG} opencv-${OPENCV_TAG} opencv_contrib-${OPENCV_TAG} && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

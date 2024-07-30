ARG UBUNTU_TAG
FROM ubuntu:${UBUNTU_TAG}

ARG OPENCV_TAG

RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && \
    apt-get install -y \
        g++ \
        clang \
        cmake \
        wget \
        unzip \
        ccache \
        libpcl-dev \
        swig \
        googletest && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_TAG}.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_TAG}.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    cmake -S opencv-${OPENCV_TAG} -B opencv-${OPENCV_TAG}/build \
        -DOPENCV_EXTRA_MODULES_PATH=opencv_contrib-${OPENCV_TAG}/modules \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DOPENCV_ENABLE_NONFREE=ON && \
    cmake --build opencv-${OPENCV_TAG}/build && \
    cmake --install opencv-${OPENCV_TAG}/build && \
    rm -rf opencv.zip opencv_contrib.zip opencv-${OPENCV_TAG} opencv_contrib-${OPENCV_TAG} && \
    apt-get clean

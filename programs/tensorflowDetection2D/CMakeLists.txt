

# Add cpp and hpp files
add_executable(tensorflowDetection2D main.cpp TensorflowDetection2D.hpp TensorflowDetection2D.cpp TensorflowSessionTest.h TensorflowSessionTest.cpp TensorflowDetector.hpp TensorflowDetector.cpp MainDetector.hpp MainDetector.cpp)
# YARP, OpenCV and TensorFlow_cc packages
find_package(YARP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TensorflowCC REQUIRED)
# OpenCV dirs
include_directories(${OpenCV_INCLUDE_DIRS})
# Link libraries
target_link_libraries(tensorflowDetection2D ${OpenCV_LIBS})
target_link_libraries(tensorflowDetection2D ${YARP_LIBRARIES})
target_link_libraries(tensorflowDetection2D TensorflowCC::Shared)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

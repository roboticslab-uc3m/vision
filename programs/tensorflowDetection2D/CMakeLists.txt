
if(NOT OpenCV_FOUND AND (NOT DEFINED ENABLE_tensorflowDetection2D OR ENABLE_tensorflowDetection2D))
    message(WARNING "OpenCV package not found, disabling tensorflowDetection2D program")
endif()

if(NOT TensorflowCC_FOUND AND (NOT DEFINED ENABLE_tensorflowDetection2D OR ENABLE_tensorflowDetection2D))
    message(WARNING "TensorflowCC package not found, disabling tensorflowDetection2D program")
endif()

cmake_dependent_option(ENABLE_tensorflowDetection2D "Enable/disable tensorflowDetection2D program" ON
                       "OpenCV_FOUND;TensorflowCC_FOUND" OFF)


if(ENABLE_tensorflowDetection2D)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    include_directories(${OpenCV_INCLUDE_DIRS})

    add_executable(tensorflowDetection2D main.cpp
                                   TensorflowDetection2D.hpp
                                   TensorflowDetection2D.cpp
                                   TensorflowDetector.hpp
                                   TensorflowDetector.cpp
                                   MainDetector.hpp
                                   MainDetector.cpp
                                   )

    target_link_libraries(tensorflowDetection2D YARP::YARP_OS
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          YARP::YARP_sig
                                          ${OpenCV_LIBS}
                                          TensorflowCC::Shared)


    install(TARGETS tensorflowDetection2D
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_tensorflowDetection2D OFF CACHE BOOL "Enable/disable tensorflowDetection2D program" FORCE)

endif()

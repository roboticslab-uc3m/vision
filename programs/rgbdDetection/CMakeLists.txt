option(ENABLE_rgbdDetection "Enable/disable rgbdDetection program" ON)

if(ENABLE_rgbdDetection)

    add_executable(rgbdDetection main.cpp
                                 RgbdDetection.hpp
                                 RgbdDetection.cpp)

    target_link_libraries(rgbdDetection YARP::YARP_os
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ROBOTICSLAB::YarpCropCallback
                                        ROBOTICSLAB::VisionInterfaces)

    if(TARGET rclcpp::rclcpp AND TARGET geometry_msgs::geometry_msgs)
        message(STATUS "Building rgbdDetection with ROS 2 support")
        target_compile_definitions(rgbdDetection PRIVATE HAVE_ROS)
        target_link_libraries(rgbdDetection rclcpp::rclcpp
                                            geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
    endif()

    install(TARGETS rgbdDetection)

else()

    set(ENABLE_rgbdDetection OFF CACHE BOOL "Enable/disable rgbdDetection program" FORCE)

endif()

option(ENABLE_rgbDetection "Enable/disable rgbDetection program" ON)

if(ENABLE_rgbDetection)

    add_executable(rgbDetection main.cpp
                                RgbDetection.hpp
                                RgbDetection.cpp)

    target_link_libraries(rgbDetection YARP::YARP_os
                                       YARP::YARP_init
                                       YARP::YARP_dev
                                       YARP::YARP_sig
                                       ROBOTICSLAB::VisionInterfaces
                                       ROBOTICSLAB::YarpCropCallback)

    if(TARGET opencv_imgproc)
        target_compile_definitions(rgbDetection PRIVATE HAVE_IMGPROC)
        target_link_libraries(rgbDetection opencv_imgproc)
    endif()

    install(TARGETS rgbDetection
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_rgbDetection OFF CACHE BOOL "Enable/disable rgbDetection program" FORCE)

endif()

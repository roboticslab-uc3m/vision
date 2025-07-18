# Create device interface target.
add_library(VisionInterfaces INTERFACE)

# Configure usage requirements.
target_link_libraries(VisionInterfaces INTERFACE YARP::YARP_os YARP::YARP_sig)

target_include_directories(VisionInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Register interface headers.
set_property(TARGET VisionInterfaces PROPERTY PUBLIC_HEADER IDetector.hpp)

# Register export set.
install(TARGETS VisionInterfaces
        EXPORT ROBOTICSLAB_VISION)

# Export variables to the global scope.
set_property(GLOBAL APPEND PROPERTY _exported_targets VisionInterfaces)
set_property(GLOBAL APPEND PROPERTY _exported_dependencies YARP_sig)

# Create alias target.
add_library(ROBOTICSLAB::VisionInterfaces ALIAS VisionInterfaces)

# FIXME: workaround for YARP 3.12.
if(YARP_VERSION VERSION_LESS 3.11.100)
    set(ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE OFF)
else()
    set(ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE ON)
endif()

option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Allow YARP to (re)build device param parsers"
       ${ALLOW_DEVICE_PARAM_PARSER_GENERATION_DEFAULT_VALUE})

# YARP devices.
add_subdirectory(ArucoDetector)
add_subdirectory(ColorRegionDetector)
add_subdirectory(DnnDetector)
add_subdirectory(HaarDetector)
add_subdirectory(QrDetector)

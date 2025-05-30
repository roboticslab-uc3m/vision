include(CheckCXXSymbolExists)
set(CMAKE_REQUIRED_INCLUDES "${OpenCV_INCLUDE_DIRS}")
set(CMAKE_REQUIRED_QUIET TRUE)
check_cxx_symbol_exists(OPENCV_ENABLE_NONFREE opencv2/opencv_modules.hpp _opencv_nonfree)

if(NOT DEFINED ENABLE_sceneReconstruction OR ENABLE_sceneReconstruction)
    if(NOT TARGET opencv_rgbd OR NOT _opencv_nonfree)
        message(WARNING "OpenCV rgbd module with non-free algorithms not found, disabling sceneReconstruction program")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling sceneReconstruction program")
    endif()
endif()

cmake_dependent_option(ENABLE_sceneReconstruction "Enable/disable sceneReconstruction program" ON
                       "TARGET opencv_rgbd;_opencv_nonfree;YARP_cv_FOUND;ENABLE_VisionIDL" OFF)

if(ENABLE_sceneReconstruction)

    add_executable(sceneReconstruction main.cpp
                                       SceneReconstruction.hpp
                                       SceneReconstruction.cpp
                                       KinectFusion.hpp
                                       KinectFusionImpl.hpp
                                       KinFu.cpp
                                       DynaFu.cpp
                                       LogComponent.hpp
                                       LogComponent.cpp)

    target_link_libraries(sceneReconstruction YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              YARP::YARP_cv
                                              opencv_rgbd
                                              ROBOTICSLAB::VisionIDL)

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.5.1)
        target_sources(sceneReconstruction PRIVATE KinFuLargeScale.cpp)
        target_compile_definitions(sceneReconstruction PRIVATE HAVE_KINFU_LS)
    endif()

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.5.5)
        target_sources(sceneReconstruction PRIVATE ColoredKinFu.cpp)
        target_compile_definitions(sceneReconstruction PRIVATE HAVE_COLORED_KINFU)
    endif()

    install(TARGETS sceneReconstruction)

else()

    set(ENABLE_sceneReconstruction OFF CACHE BOOL "Enable/disable sceneReconstruction program" FORCE)

endif()

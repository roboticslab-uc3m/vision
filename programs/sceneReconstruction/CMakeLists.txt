include(CheckCXXSymbolExists)
set(CMAKE_REQUIRED_INCLUDES "${OpenCV_INCLUDE_DIRS}")
set(CMAKE_REQUIRED_QUIET TRUE)
check_cxx_symbol_exists(OPENCV_ENABLE_NONFREE opencv2/opencv_modules.hpp _opencv_nonfree)

if(NOT DEFINED ENABLE_sceneReconstruction OR ENABLE_sceneReconstruction)
    if(NOT TARGET opencv_rgbd OR NOT OpenCV_VERSION VERSION_GREATER_EQUAL 4.0 OR NOT _opencv_nonfree)
        message(WARNING "OpenCV 4.0+/rgbd module with non-free algorithms not found, disabling sceneReconstruction program")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling sceneReconstruction program")
    endif()
endif()

cmake_dependent_option(ENABLE_sceneReconstruction "Enable/disable sceneReconstruction program" ON
                       "TARGET opencv_rgbd;OpenCV_VERSION VERSION_GREATER_EQUAL 4.0;_opencv_nonfree;YARP_cv_FOUND" OFF)

if(ENABLE_sceneReconstruction)

    # sceneReconstruction
    add_executable(sceneReconstruction main.cpp
                                       SceneReconstruction.hpp
                                       SceneReconstruction.cpp
                                       KinectFusion.hpp
                                       KinectFusionImpl.hpp
                                       KinFu.cpp)

    target_link_libraries(sceneReconstruction YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              YARP::YARP_cv
                                              opencv_rgbd)

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.2)
        target_sources(sceneReconstruction PRIVATE DynaFu.cpp)
        target_compile_definitions(sceneReconstruction PRIVATE HAVE_DYNAFU)
    endif()

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.5.1)
        target_sources(sceneReconstruction PRIVATE KinFuLargeScale.cpp)
        target_compile_definitions(sceneReconstruction PRIVATE HAVE_KINFU_LS)
    endif()

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.5.3)
        target_sources(sceneReconstruction PRIVATE ColoredKinFu.cpp)
        target_compile_definitions(sceneReconstruction PRIVATE HAVE_COLORED_KINFU)
    endif()

    install(TARGETS sceneReconstruction
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_sceneReconstruction OFF CACHE BOOL "Enable/disable sceneReconstruction program" FORCE)

endif()

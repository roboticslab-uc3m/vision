if(NOT DEFINED ENABLE_sceneReconstruction OR ENABLE_sceneReconstruction)
    if(NOT TARGET opencv_rgbd)
        message(WARNING "OpenCV/rgbd module not found, disabling sceneReconstruction program")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling sceneReconstruction program")
    endif()
endif()

cmake_dependent_option(ENABLE_sceneReconstruction "Enable/disable sceneReconstruction program" ON
                       "TARGET opencv_rgbd;YARP_cv_FOUND" OFF)

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
                                              opencv_rgbd
                                              ROBOTICSLAB::ColorDebug)

    if(OpenCV_VERSION VERSION_GREATER_EQUAL 4.2)
        target_sources(sceneReconstruction PRIVATE DynaFu.cpp)
        target_compile_definitions(sceneReconstruction PRIVATE HAVE_DYNAFU)
    endif()

    install(TARGETS sceneReconstruction
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_sceneReconstruction OFF CACHE BOOL "Enable/disable sceneReconstruction program" FORCE)

endif()

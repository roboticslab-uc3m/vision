if(NOT DEFINED ENABLE_sceneReconstruction OR ENABLE_sceneReconstruction)
    if(NOT OpenCV_FOUND OR NOT OpenCV_VERSION_MAJOR EQUAL 4)
        message(WARNING "OpenCV 4.x package not found, disabling sceneReconstruction program")
    elseif(NOT YARP_cv_FOUND)
        message(WARNING "YARP_cv package not found, disabling sceneReconstruction program")
    endif()
endif()

cmake_dependent_option(ENABLE_sceneReconstruction "Enable/disable sceneReconstruction program" ON
                       "OpenCV_FOUND;YARP_cv_FOUND" OFF)

if(ENABLE_sceneReconstruction)

    # sceneReconstruction
    add_executable(sceneReconstruction main.cpp
                                       SceneReconstruction.hpp
                                       SceneReconstruction.cpp)

    target_link_libraries(sceneReconstruction YARP::YARP_os
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              YARP::YARP_cv
                                              ROBOTICSLAB::ColorDebug)

    install(TARGETS sceneReconstruction
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_sceneReconstruction OFF CACHE BOOL "Enable/disable sceneReconstruction program" FORCE)

endif()

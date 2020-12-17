option(ENABLE_examples "Enable/disable C++ examples" OFF)

if(ENABLE_examples)
    if(TARGET ROBOTICSLAB::VisionInterfaces)
        add_subdirectory(exampleColorRegionDetector)
        add_subdirectory(exampleHaarDetector)
    endif()

    if(TARGET ROBOTICSLAB::YarpCloudUtils)
        add_subdirectory(exampleMeshFromCloud)
        add_subdirectory(exampleMeshFromLiveRGBD)
        add_subdirectory(exampleProcessCloud)
        add_subdirectory(exampleSceneReconstructionClient)
    endif()

    add_subdirectory(exampleRemoteGrabber)
    add_subdirectory(exampleRemoteRGBDSensor)
endif()

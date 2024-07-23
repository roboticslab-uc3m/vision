option(ENABLE_examples "Enable/disable C++ examples" OFF)

if(ENABLE_examples)
    if(TARGET ROBOTICSLAB::VisionInterfaces)
        add_subdirectory(exampleArucoDetector)
        add_subdirectory(exampleColorRegionDetector)
        add_subdirectory(exampleDnnDetector)
        add_subdirectory(exampleHaarDetector)
        add_subdirectory(exampleQrDetector)
    endif()

    if(TARGET ROBOTICSLAB::YarpCloudUtils)
        add_subdirectory(exampleMeshFromCloud)
        add_subdirectory(exampleMeshFromLiveRGBD)
        add_subdirectory(exampleProcessCloud)

        if(TARGET ROBOTICSLAB::VisionIDL)
            add_subdirectory(exampleSceneReconstructionClient)
        endif()
    endif()

    add_subdirectory(exampleRemoteGrabber)
    add_subdirectory(exampleRemoteRGBDSensor)
endif()

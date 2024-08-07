if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/googletest)
    include_directories(${GTestSources_INCLUDE_DIR})

    if(ENABLE_ArucoDetector)
        add_executable(testArucoDetector testArucoDetector.cpp)

        target_link_libraries(testArucoDetector YARP::YARP_os
                                                YARP::YARP_dev
                                                YARP::YARP_sig
                                                ROBOTICSLAB::VisionInterfaces
                                                gtest_main)

        gtest_discover_tests(testArucoDetector)
    endif()

    if(ENABLE_ColorRegionDetector)
        add_executable(testColorRegionDetector testColorRegionDetector.cpp)

        target_link_libraries(testColorRegionDetector YARP::YARP_os
                                                      YARP::YARP_dev
                                                      YARP::YARP_sig
                                                      ROBOTICSLAB::VisionInterfaces
                                                      gtest_main)

        gtest_discover_tests(testColorRegionDetector)
    endif()

    if(ENABLE_DnnDetector AND DOWNLOAD_MODELS)
        add_executable(testDnnDetector testDnnDetector.cpp)

        target_link_libraries(testDnnDetector YARP::YARP_os
                                              YARP::YARP_dev
                                              YARP::YARP_sig
                                              ROBOTICSLAB::VisionInterfaces
                                              gtest_main)

        gtest_discover_tests(testDnnDetector)
    endif()

    if(ENABLE_HaarDetector)
        add_executable(testHaarDetector testHaarDetector.cpp)

        target_link_libraries(testHaarDetector YARP::YARP_os
                                               YARP::YARP_dev
                                               YARP::YARP_sig
                                               ROBOTICSLAB::VisionInterfaces
                                               gtest_main)

        if(_HaarDetector_have_cv_face AND DOWNLOAD_MODELS)
            target_compile_definitions(testHaarDetector PRIVATE HAVE_CV_FACE)
        endif()

        gtest_discover_tests(testHaarDetector)
    endif()

    if(ENABLE_QrDetector)
        add_executable(testQrDetector testQrDetector.cpp)

        target_link_libraries(testQrDetector YARP::YARP_os
                                             YARP::YARP_dev
                                             YARP::YARP_sig
                                             ROBOTICSLAB::VisionInterfaces
                                             gtest_main)

        gtest_discover_tests(testQrDetector)
    endif()

    if(TARGET YarpCloudUtils)
        add_executable(testYarpCloudUtils testYarpCloudUtils.cpp)

        target_link_libraries(testYarpCloudUtils ROBOTICSLAB::YarpCloudUtils
                                                 gtest_main)

        target_compile_definitions(testYarpCloudUtils PRIVATE PLY_PATH="${CMAKE_CURRENT_BINARY_DIR}/")

        gtest_discover_tests(testYarpCloudUtils)
    endif()

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()

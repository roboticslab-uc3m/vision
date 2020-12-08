if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)

    enable_testing()

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

    include_directories(${GTestSources_INCLUDE_DIR})

    # testColorRegionDetector

    if(TARGET ColorRegionDetector)
        add_executable(testColorRegionDetector testColorRegionDetector.cpp)

        target_link_libraries(testColorRegionDetector YARP::YARP_os
                                                      YARP::YARP_dev
                                                      YARP::YARP_sig
                                                      ROBOTICSLAB::ColorDebug
                                                      VisionInterfaces
                                                      gtest_main)

        target_compile_features(testColorRegionDetector PUBLIC cxx_std_11)

        gtest_discover_tests(testColorRegionDetector)
    endif()

    if(TARGET HaarDetector)
        add_executable(testHaarDetector testHaarDetector.cpp)

        target_link_libraries(testHaarDetector YARP::YARP_os
                                               YARP::YARP_dev
                                               YARP::YARP_sig
                                               ROBOTICSLAB::ColorDebug
                                               VisionInterfaces
                                               gtest_main)

        target_compile_features(testHaarDetector PUBLIC cxx_std_11)

        gtest_discover_tests(testHaarDetector)
    endif()

    if(TARGET QRDetector)
        add_executable(testQRDetector testQRDetector.cpp)

        target_link_libraries(testQRDetector YARP::YARP_os
                                             YARP::YARP_dev
                                             YARP::YARP_sig
                                             ROBOTICSLAB::ColorDebug
                                             VisionInterfaces
                                             gtest_main)

        target_compile_features(testQRDetector PUBLIC cxx_std_11)

        gtest_discover_tests(testQRDetector)
    endif()

    if(TARGET YarpCloudUtils)
        add_executable(testYarpCloudUtils testYarpCloudUtils.cpp)

        target_link_libraries(testYarpCloudUtils YarpCloudUtils
                                                 gtest_main)

        target_compile_features(testYarpCloudUtils PUBLIC cxx_std_14)
        target_compile_definitions(testYarpCloudUtils PRIVATE PLY_PATH="${CMAKE_CURRENT_BINARY_DIR}/")

        gtest_discover_tests(testYarpCloudUtils)
    endif()

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()

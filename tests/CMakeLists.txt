if(NOT GTestSources_FOUND AND (NOT DEFINED ENABLE_tests OR ENABLE_tests))
    message(WARNING "GTestSources package not found, disabling tests")
endif()

cmake_dependent_option(ENABLE_tests "Enable/disable unit tests" ON
                       GTestSources_FOUND OFF)

if(ENABLE_tests)

    enable_testing()

    add_subdirectory(${GTestSources_SOURCE_DIR} ${CMAKE_BINARY_DIR}/gtest)

    include_directories(${GTestSources_INCLUDE_DIR}})

    # testColorRegionDetector

    if(TARGET ColorRegionDetector)
        add_executable(testColorRegionDetector testColorRegionDetector.cpp)

        target_link_libraries(testColorRegionDetector VisionInterfaces
                                                      gtest_main)

        target_compile_features(testColorRegionDetector PUBLIC cxx_std_11)

        gtest_discover_tests(testColorRegionDetector)
    endif()

else()

    set(ENABLE_tests OFF CACHE BOOL "Enable/disable unit tests" FORCE)

endif()

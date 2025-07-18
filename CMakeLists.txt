cmake_minimum_required(VERSION 3.19...3.28)

# 'C' required by target MPI::MPI_C via PCLConfig.cmake (affects Ubuntu 24.04).
project(ROBOTICSLAB_VISION LANGUAGES C CXX)

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
    # Possible values.
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    if(NOT CMAKE_BUILD_TYPE)
        # Encourage the user to specify build type.
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

# Pick up our cmake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.11 REQUIRED COMPONENTS os dev sig
                                OPTIONAL_COMPONENTS cv idl_tools pcl)

# Soft dependencies.
find_package(OpenCV 4.2 QUIET)
find_package(PCL 1.10 QUIET COMPONENTS common features filters search surface)
find_package(GTestSources 1.8 QUIET)
find_package(SWIG QUIET)
find_package(Doxygen QUIET)

# Always build YARP devices as MODULE libraries.
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "Force dynamic plugins")

# Configure installation paths for YARP resources.
yarp_configure_external_installation(roboticslab-vision WITH_PLUGINS)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Create targets if specific requirements are satisfied.
include(CMakeDependentOption)

# Acknowledge this is a CTest-friendly project.
enable_testing()

# Add main contents.
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(models)
add_subdirectory(tests)
add_subdirectory(share)
add_subdirectory(bindings)
add_subdirectory(doc)
add_subdirectory(examples/cpp)

# Retrieve global properties.
get_property(_exported_targets GLOBAL PROPERTY _exported_targets)
get_property(_exported_dependencies GLOBAL PROPERTY _exported_dependencies)

if(NOT DEFINED _exported_targets)
    set(_no_export NO_EXPORT)
endif()

if(DEFINED _exported_dependencies)
    list(REMOVE_DUPLICATES _exported_dependencies)
endif()

# Store the package in the user registry.
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

# Create and install config files.
include(InstallBasicPackageFiles)

install_basic_package_files(ROBOTICSLAB_VISION
                            VERSION 0.1.0
                            COMPATIBILITY AnyNewerVersion
                            ${_no_export}
                            NO_SET_AND_CHECK_MACRO
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            NAMESPACE ROBOTICSLAB::
                            DEPENDENCIES ${_exported_dependencies})

# Configure and create uninstall target.
include(AddUninstallTarget)

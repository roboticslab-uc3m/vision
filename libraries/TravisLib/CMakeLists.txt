if(ENABLE_TravisLib)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(TravisLib TravisLib.hpp TravisLib.cpp)

target_link_libraries(TravisLib ${OpenCV_LIBS})

install(TARGETS TravisLib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

set(ROBOTICSLAB_VISION_INCLUDE_DIRS ${ROBOTICSLAB_VISION_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_VISION_LIBRARIES ${ROBOTICSLAB_VISION_LIBRARIES} TravisLib CACHE INTERNAL "appended libraries")

endif()

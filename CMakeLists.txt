cmake_minimum_required(VERSION 3.12)
project(ob_gins_vlp)

set(CMAKE_CXX_STANDARD 14)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
endif ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE SOURCE
        src/ob_gins_vlp.cc
        src/fileio/fileloader.cc
        src/fileio/filesaver.cc
        src/preintegration/preintegration_base.cc
        src/preintegration/preintegration_normal.cc
        src/preintegration/preintegration_odo.cc)

include_directories(${PROJECT_SOURCE_DIR})
add_executable(${PROJECT_NAME} ${SOURCE})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# GTest
find_package(GTest REQUIRED)

# abseil
find_package(absl REQUIRED)
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format
        absl::time)

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
include_directories(${CERES_INCLUDE_DIRS})

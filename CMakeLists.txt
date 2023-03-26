cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Pangolin REQUIRED)
find_package(Glog REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    "/usr/include/eigen3"
    ${PROJECT_SOURCE_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

LIST(APPEND LINK_LIBS
    ${Pangolin_LIBRARIES}
    ${GLOG_LIBRARIES}
    pthread
)

add_executable(main  
    main.cpp  
    src/common.cpp
    src/pre_integration.cpp
    src/simulate_data_gen.cpp 
    src/visualizer.cpp)
TARGET_LINK_LIBRARIES (main ${LINK_LIBS})

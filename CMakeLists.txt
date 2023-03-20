cmake_minimum_required(VERSION 2.8)


find_package(Pangolin REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/include/eigen3")
include_directories(
    ${Pangolin_INCLUDE_DIRS}
)

LIST(APPEND LINK_LIBS
    ${Pangolin_LIBRARIES}
    pthread
)

add_executable(main  
    main.cpp  
    src/simulate_data_gen.cpp 
    src/visualizer.cpp)
TARGET_LINK_LIBRARIES (main ${LINK_LIBS})

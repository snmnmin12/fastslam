project(fastslam)
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PythonLibs 2.7)

set(s_fast src/main.cpp src/fastslam.cpp src/tools.cpp)
add_executable(fastslam ${s_fast})
target_include_directories(fastslam PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(fastslam ${PYTHON_LIBRARIES})


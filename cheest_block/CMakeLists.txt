cmake_minimum_required(VERSION 2.8.3)
project(conner_check)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS"${CMAKE_C_FLAGS}  -Wall -O3 -L/usr/local/lib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -L/usr/local/lib")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

find_package(OpenCV REQUIRED)
# include_directories(${OPENCV_DIR})

# target_sources(
#                 src/conner_check.cpp
# )
add_executable(conner_cpp src/conner_check.cpp)
target_link_libraries(conner_cpp ${OpenCV_LIBS})            

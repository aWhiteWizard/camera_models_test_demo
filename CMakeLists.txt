cmake_minimum_required(VERSION 2.8)
project(camera_undistortion)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -L/usr/local/lib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -L/usr/local/lib")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
find_package(PCL 1.2 REQUIRED)
   if(NOT PCL_FOUND)
      message(FATAL_ERROR "PCL > 1.8 not found.")
   endif()
find_package(VTK 9.1 REQUIRED)
# set(VTK_DLL_DIRS ${VTK_INSTALL_PERFIX}/bin)
# set(BIN_OUTPUT_DIR ${VTK_DLL_DIR}/)
find_package(OpenCV REQUIRED)

link_libraries(/usr/local/lib)

link_directories(${PCL_LIBRARY_DIRS})    
add_definitions(${PCL_DEFINITIONS})
include_directories()   
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ./include
    /usr/local/include
    /usr/local/include/vtk-9.1
)

add_executable(camera_undistortion src/camera_undistortion.cpp)
target_link_libraries(camera_undistortion ${OpenCV_LIBS} ${PCL_LIBRARIES})
install(TARGETS camera_undistortion RUNTIME DESTINATION bin) 
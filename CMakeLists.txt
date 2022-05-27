cmake_minimum_required(VERSION 2.8)
project(camera_undistortion)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -L/usr/local/lib")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -L/usr/local/lib  -lrealsense2")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -lrealsense2")

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
find_package(PCL 1.2 REQUIRED)
   if(NOT PCL_FOUND)
      message(FATAL_ERROR "PCL > 1.8 not found.")
   endif()
# find_package(EIGEN3 REQUIRED)
# if(NOT PCL_FOUND)
#    message(FATAL_ERROR "EIGEN3 not found.")
# endif()
find_package(VTK 9.1 REQUIRED)
# set(VTK_DLL_DIRS ${VTK_INSTALL_PERFIX}/bin)
# set(BIN_OUTPUT_DIR ${VTK_DLL_DIR}/)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})    
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ./include
    /usr/local/include
    /usr/local/include/vtk-9.1
)
set(DEPENDENCIES realsense2)


add_executable(camera_undistortion src/camera_undistortion.cpp)
target_link_libraries(camera_undistortion ${OpenCV_LIBS} ${PCL_LIBRARIES})
install(TARGETS camera_undistortion RUNTIME DESTINATION bin) 

add_executable(picture_sum src/picture_sum.cpp)
target_link_libraries(picture_sum ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(opencv_setero src/opencv_setero.cpp)
target_link_libraries(opencv_setero ${OpenCV_LIBS} ${PCL_LIBRARIES} ${realsense2_LIBRARY})

add_executable(opencv_orb_match src/opencv_orb_match.cpp)
target_link_libraries(opencv_orb_match ${OpenCV_LIBS} ${PCL_LIBRARIES} ${realsense2_LIBRARY})

add_executable(merge_with_coordinate src/merge_with_coordinate.cpp)
target_link_libraries(merge_with_coordinate ${OpenCV_LIBS} ${PCL_LIBRARIES} )
install(TARGETS merge_with_coordinate RUNTIME DESTINATION bin) 


add_executable(setero_calibr src/setero_calibr.cpp)
target_link_libraries(setero_calibr ${OpenCV_LIBS} ${PCL_LIBRARIES} )
install(TARGETS setero_calibr RUNTIME DESTINATION bin) 

add_executable(lab03 src/lab03.cpp)
target_link_libraries(lab03 ${OpenCV_LIBS} ${PCL_LIBRARIES} )
install(TARGETS lab03 RUNTIME DESTINATION bin) 

add_executable(global_splicing src/global_splicing.cpp)
target_link_libraries(global_splicing ${OpenCV_LIBS} ${PCL_LIBRARIES} )
install(TARGETS global_splicing RUNTIME DESTINATION bin) 

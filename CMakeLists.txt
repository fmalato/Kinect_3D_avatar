cmake_minimum_required(VERSION 3.13)
project(3D_avatar)

set(CMAKE_CXX_STANDARD 14)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

add_executable(3D_avatar main.cpp glad.c Shader.h stb_image.h Camera.h utils.h)
target_link_libraries(3D_avatar -lglew32 -lglfw3 -lopengl32 -lglu32 -lgdi32 -libusb)

find_package( KinectSDK2 REQUIRED )

# Settings Kinect SDK v2
if(KinectSDK2_FOUND)
    include_directories( ${KinectSDK2_INCLUDE_DIRS} )
    link_directories( ${KinectSDK2_LIBRARY_DIRS} )
    target_link_libraries(3D_avatar ${KinectSDK2_LIBRARIES} )
    add_custom_command( TARGET 3D_avatar POST_BUILD ${KinectSDK2_COMMANDS} )
endif()
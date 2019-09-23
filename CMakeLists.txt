cmake_minimum_required(VERSION 3.13)
project(3D_avatar)

set(CMAKE_CXX_STANDARD 14)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

include_directories("D:/Matlab/extern/include" "D:/Matlab/extern/lib/win64/microsoft/" "C:/MinGW-64Bit/mingw64/include/")
link_directories("D:/Matlab/extern/include" "D:/Matlab/extern/lib/win64/microsoft/")

add_executable(3D_avatar main.cpp glad.c Shader.h stb_image.h Camera.h utils.h Position.cpp Position.h Joint.cpp Joint.h Polygon.cpp Polygon.h)
target_link_libraries(3D_avatar -lglew32 -lglfw3 -lopengl32 -lglu32 -lgdi32 -lglut32win)




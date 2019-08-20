cmake_minimum_required(VERSION 3.13)
project(3D_avatar)

set(CMAKE_CXX_STANDARD 14)

add_executable(3D_avatar main.cpp glad.c Shader.h stb_image.h Camera.h utils.h)
target_link_libraries(3D_avatar -lglew32 -lglfw3 -lopengl32 -lglu32 -lgdi32)
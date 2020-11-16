cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 14)

#set(PCL_DIR "/home/brend/BCCancer/pcl/build")
#set(PCL_DIR "/usr/local/pcl-1.8/pcl/") #there is nothing here ???
set(PCL_DIR ".usr/local/include/pcl-1.9/pcl")
#i dont know what the differences are for any of these ?????

project(COLLISION_CHECKER)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_executable(${PROJECT_NAME} CollisionChecker.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)

find_package(LIBIGL REQUIRED QUIET)

# Add your project files
file(GLOB SRCFILES *.cpp)
add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

#dont think i need both add_executables

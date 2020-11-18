cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(COLLISION_CHECKER)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF) # Disable GNU extensions (e.g., std=gnu++14).


# Eigen.
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# PCL.
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# LibIGL.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_WITH_OPENGL      "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW"   ON)
find_package(LIBIGL REQUIRED QUIET)

# Build the executable.
file(GLOB SRCFILES src/*.cpp)
# This should set everything in src/ to the SRCFILES
add_executable(collision_checker ${SRCFILES})
target_link_libraries(collision_checker igl::core igl::opengl_glfw)
target_link_libraries(collision_checker ${PCL_LIBRARIES})


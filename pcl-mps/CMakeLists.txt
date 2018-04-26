cmake_minimum_required(VERSION 2.8.11)

project(mps)

find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})


# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_C_FLAGS_DEBUG MTd)


add_executable  (mps src/mps.cpp)

target_link_libraries (mps ${PCL_LIBRARIES} ${VTK_LIBRARIES})
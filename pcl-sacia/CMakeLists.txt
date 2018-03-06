cmake_minimum_required(VERSION 2.8.11)

project(PointCloudRegistration)

find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES src/main.cpp
					  src/features.cpp
					  src/filters.cpp
					  src/sac_ia.cpp
					  src/visualization.cpp)
set  (project_HEADERS src/common.hpp
	                  src/features.h
					  src/filters.h
					  src/sac_ia.h
					  src/visualization.h)
#set  (project_FORMS   src/mainwindow.ui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_FLAGS_DEBUG MTd)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
#find_package(Qt5Widgets)

#QT5_WRAP_UI(project_UIS mainwindow.ui)

add_executable  (pcr    WIN32  ${project_SOURCES}
                                      ${project_HEADERS})

target_link_libraries (pcr ${PCL_LIBRARIES} ${VTK_LIBRARIES})
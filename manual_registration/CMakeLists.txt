cmake_minimum_required(VERSION 2.8.11)

project(manual_registration)

find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES src/manual_registration.cpp)
set  (project_HEADERS src/manual_registration.h)
set  (project_FORMS   src/manual_registration.ui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_C_FLAGS_DEBUG MTd)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

QT5_WRAP_UI(project_UIS ${project_FORMS})

# Tell CMake to create the helloworld executable

add_executable  (manual_registration WIN32 ${project_SOURCES} ${project_HEADERS} ${project_FORMS} ${project_HEADERS_MOC})
# Use the Widgets module from Qt 5.

target_link_libraries (manual_registration Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
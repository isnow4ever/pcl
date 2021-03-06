cmake_minimum_required(VERSION 2.8.11)

project(auto_registration)

find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES src/main.cpp src/mainwindow.cpp src/ICPReg.cpp)
set  (project_HEADERS src/mainwindow.h src/ICPReg.h)
set  (project_FORMS   src/mainwindow.ui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_C_FLAGS_DEBUG MTd)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

QT5_WRAP_UI(project_UIS mainwindow.ui)

add_executable  (auto_registration    WIN32  ${project_SOURCES}
                                      ${project_HEADERS}
									  ${project_FORMS}
                                      ${project_HEADERS_MOC})

target_link_libraries (auto_registration Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
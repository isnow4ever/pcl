cmake_minimum_required(VERSION 2.8.11)

project(qvtkwidget)

find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES src/main.cpp src/ProjectMainWindow.cpp)
set  (project_HEADERS src/ProjectMainWindow.h)
set  (project_FORMS   src/ProjectMainWindow.ui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_C_FLAGS_DEBUG MTd)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

QT5_WRAP_UI(example_UIS ${project_FORMS})

# Tell CMake to create the helloworld executable

add_executable  (qvtkviewer WIN32 ${project_SOURCES}
                                      ${project_HEADERS}
									  ${project_FORMS}
                                      ${project_HEADERS_MOC})
# Use the Widgets module from Qt 5.

target_link_libraries (qvtkviewer Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
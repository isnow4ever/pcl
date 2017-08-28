cmake_minimum_required(VERSION 2.8.11)

project(qtviewer)

find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES src/main.cpp src/pclviewer.cpp)
set  (project_HEADERS src/pclviewer.h)
set  (project_FORMS   src/pclviewer.ui)
#set  (VTK_LIBRARIES   vtkRenderingOpenGL2 vtkGUISupportQt vtkInteractionStyle vtkRenderingFreeType)

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
#add_executable(qt_widgets WIN32 src/main.cpp src/mainwindow.cpp)

#include         (${QT_USE_FILE})
#add_definitions (${QT_DEFINITIONS})

add_executable  (pcl_visualizer WIN32 ${project_SOURCES}
                                      ${project_HEADERS}
									  ${project_FORMS}
                                      ${project_HEADERS_MOC})

target_link_libraries (pcl_visualizer Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# Use the Widgets module from Qt 5.
#target_link_libraries(qt_widgets Qt5::Widgets)
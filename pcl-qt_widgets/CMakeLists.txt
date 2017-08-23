cmake_minimum_required(VERSION 2.8.11)

project(qt_widgets)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_C_FLAGS_DEBUG MTd)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

QT5_WRAP_UI(example_UIS mainwindow.ui)

# Tell CMake to create the helloworld executable
add_executable(qt_widgets WIN32 src/main.cpp src/mainwindow.cpp)

# Use the Widgets module from Qt 5.
target_link_libraries(qt_widgets Qt5::Widgets)
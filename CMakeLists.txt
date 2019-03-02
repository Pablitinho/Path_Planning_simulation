cmake_minimum_required(VERSION 3.8.2)

project(Trajectory)

# Find includes in the build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Turn on automatic invocation of the MOC & UIC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# There may be a way to tell up front if Qt5 is going to be found, but I haven't found
# a foolproof way to do it yet, so settle for the default error message for now.
#if(NOT CMAKE_PREFIX_PATH AND NOT Qt5Widgets_DIR)
#    message(WARNING "CMAKE_PREFIX_PATH is not defined, so find_package may not work. Set the CMAKE_PREFIX_PATH "
#            "environment variable to the install prefix of Qt 5, either on the command line as "
#            "-DCMAKE_PREFIX_PATH=\"path/to/Qt5/lib/cmake\" or with set(CMAKE_PREFIX_PATH path/to/Qt5/lib/cmake)")
#endif(NOT CMAKE_PREFIX_PATH AND NOT Qt5Widgets_DIR)

#Eigen 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Add a compiler flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Make this a GUI application on Windows
if(WIN32)
  set(CMAKE_WIN32_EXECUTABLE ON)
endif()

# Find the QtWidgets library
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt5Widgets)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Svg REQUIRED)

file(GLOB SRC_CODE "src/*.cpp")
file(GLOB HEADER_CODE "src/*.h")
file(GLOB UI_CODE "src/*.ui")
# QWT library
set(QWT_LIBRARIES_PATH "C:/Qwt-6.1.4/lib")
set(QWT_INCLUDE_DIR "C:/Qwt-6.1.4/include")

include_directories(${QWT_INCLUDE_DIR} "src/")

# Generate rules for building source files from the resources
# Not at the moment

# Tell CMake to create the helloworld executable
add_executable(${PROJECT_NAME} ${SRC_CODE} ${HEADER_CODE} ${UI_CODE})

# Add the Qt5 Widgets for linking
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen debug ${QWT_LIBRARIES_PATH}/qwtd.lib optimized ${QWT_LIBRARIES_PATH}/qwt.lib Qt5::Widgets Qt5::Svg Qt5::PrintSupport)

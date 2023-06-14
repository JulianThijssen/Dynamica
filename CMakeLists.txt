# Specifies the CMake version we require
cmake_minimum_required(VERSION 3.9)

# Include the INSTALL target in the build step
set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Set executable directory
set(BINARY_DIR ${CMAKE_SOURCE_DIR}/Binary)

project(ProjectiveDynamics LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# Set our include folders as the place to look for library includes
#include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/stbimage/include/)
#include_directories(${CMAKE_SOURCE_DIR}/Source)

add_subdirectory(Library)
add_subdirectory(Application)

#target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/ThirdParty/GDT/lib/$<CONFIG>/GDT.lib)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BINARY_DIR}/$<CONFIG>/)

# Set VS working directory so the resources folder can be found
#set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# Install binaries
#install(FILES ThirdParty/glfw/bin/$<CONFIG>/glfw3.dll DESTINATION ${BINARY_DIR}/$<CONFIG>/)

#set startup project to be the main executable instead of ALL_BUILD
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT "${PROJECT_NAME}")

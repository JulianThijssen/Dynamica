set(LIBRARY_SOURCES
    Source/ProjectiveDynamics.h
    Source/ProjectiveDynamics.cpp
    Source/NewtonRaphson.h
    Source/NewtonRaphson.cpp
)

add_library(${PROJECT_NAME}_lib SHARED
    ${LIBRARY_SOURCES}
)

source_group(Library FILES ${LIBRARY_SOURCES})

target_include_directories(${PROJECT_NAME}_lib PRIVATE ThirdParty/Eigen/include)

#--------------------------------------------------------------------
# INSTALL
#--------------------------------------------------------------------
# Install artifacts to output directory
install(FILES ${CMAKE_BINARY_DIR}/Library/$<CONFIG>/${PROJECT_NAME}_lib.dll DESTINATION ${BINARY_DIR}/$<CONFIG>/)
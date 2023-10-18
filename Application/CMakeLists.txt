set(APPLICATION_SOURCES
    Source/Application.cpp
    Source/ObjLoader.h
    Source/Scene.h
    Source/Mesh.h
    Source/CellExporter.h
    Source/CellExporter.cpp
    Source/Recorder.h
    Source/Recorder.cpp
)

add_executable(${PROJECT_NAME}
    ${APPLICATION_SOURCES}
)

source_group(Application FILES ${APPLICATION_SOURCES})

# FIXME, Replace by consuming public headers from Library
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Library/Source)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Library/ThirdParty/Eigen/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Application/ThirdParty/ShapeOp/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Application/ThirdParty/Assimp/include)
target_include_directories(${PROJECT_NAME} PRIVATE ThirdParty/GDT/include)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/ShapeOp/lib/$<CONFIG>/ShapeOp.0.1.0.lib")
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/Assimp/lib/Release/assimp.lib")
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/GDT/lib/glfw3.lib")
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/GDT/lib/$<CONFIG>/GDT.lib")

# Install binary dependencies to output folder
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Resources"
    DESTINATION ${CMAKE_INSTALL_PREFIX}/$<CONFIG>
)

# Set VS working directory to output folder
if(MSVC)
    set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/$<CONFIG>")
endif()

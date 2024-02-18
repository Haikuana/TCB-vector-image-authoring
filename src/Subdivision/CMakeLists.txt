cmake_policy(SET CMP0077 NEW)

set(APP_NAME SubdivisionMesh)
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC OFF )

aux_source_directories(SOURCES "Source Files\\include"     .\\include)
aux_source_directories(mainfiles "Source Files\\include"     .\\include)

#copy files
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/mat_dep_fitcolor/sparse_quad_prog_color.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/mat_dep_fitcolor/sparse_quad_prog_color.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)

#create files in VS
source_group("Main" FILES ${mainfiles})
add_library(${APP_NAME} STATIC ${SOURCES})

message(STATUS "approxiGui source path: ${ImageApproxiPath}")
set(SubdivisionPath 
${ImageApproxiPath} 
${CMAKE_CURRENT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src/3dparty/mat_dep_fitcolor)

target_include_directories(${APP_NAME} PUBLIC ${SubdivisionPath})
target_link_directories(${APP_NAME} PUBLIC ${SubdivisionPath})

target_link_libraries(${APP_NAME} PUBLIC ImageApproxi)
target_link_libraries(${APP_NAME} PUBLIC sparse_quad_prog_color)

install_runtime_targets(${APP_NAME})

SET_TARGET_PROPERTIES(${APP_NAME} PROPERTIES FOLDER "ImageApproximation")

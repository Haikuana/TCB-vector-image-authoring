cmake_policy(SET CMP0077 NEW)

set(APP_NAME ImageApproxi)
set( CMAKE_INCLUDE_CURRENT_DIR ON )

aux_source_directories(SOURCES "Source Files\\include\\Assistant"     			include\\Assistant)
aux_source_directories(Assistantfiles "Source Files\\include\\Assistant"     	include\\Assistant)
aux_source_directories(SOURCES "Source Files\\include\\Corefuncs"     			include\\Corefuncs)
aux_source_directories(Corefiles "Source Files\\include\\Corefuncs"     		include\\Corefuncs)
aux_source_directories(SOURCES "Source Files\\include\\ImageFeatureEdge"     	include\\ImageFeatureEdge)
aux_source_directories(Featurefiles "Source Files\\include\\ImageFeatureEdge"   include\\ImageFeatureEdge)
aux_source_directories(SOURCES "Source Files\\include\\OptimizeKnotTri"     	include\\OptimizeKnotTri)
aux_source_directories(KnotTrifiles "Source Files\\include\\OptimizeKnotTri"    include\\OptimizeKnotTri)

#copy files
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/ANN/ANN.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/ANN/ANN.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/mat_dep_quadprog/sparse_quad_prog.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/mat_dep_quadprog/sparse_quad_prog.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/OpenMesh/OpenMeshCored.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/OpenMesh/OpenMeshCore.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/QGLViewer/QGLViewerd2.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
file(COPY ${PROJECT_SOURCE_DIR}/src/3dparty/QGLViewer/QGLViewer2.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)

#matlab
find_package(Matlab)#REQUIRED ENG_LIBRARY MX_LIBRARY MAT_LIBRARY
IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")
ELSE(MATLAB_FOUND)
    MESSAGE("MATLAB not found...nothing will be built.")
ENDIF(MATLAB_FOUND)

#opencv
find_package(OpenCV REQUIRED)
# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

#create files in VS
source_group("Axuiliary" FILES ${Assistantfiles})
source_group("Cores" FILES ${Corefiles})
source_group("FeatureEdge" FILES ${Featurefiles})
source_group("OptKnotTri" FILES ${KnotTrifiles})

add_library(${APP_NAME} STATIC ${SOURCES})

#for openmesh
if (WIN32)
	target_compile_definitions(${APP_NAME} PUBLIC _USE_MATH_DEFINES NOMINMAX)
endif ()

set(ImageApproxiPath 
${PROJECT_SOURCE_DIR}/src/ImageApproxi/include
${PROJECT_SOURCE_DIR}/src/3dparty
${PROJECT_SOURCE_DIR}/src/3dparty/ANN
${PROJECT_SOURCE_DIR}/src/3dparty/mat_dep_quadprog
${PROJECT_SOURCE_DIR}/src/3dparty/QGLViewer
${PROJECT_SOURCE_DIR}/src/3dparty/OpenMesh
CACHE INTERNAL "files location"
)

target_include_directories(${APP_NAME} PUBLIC ${ImageApproxiPath})
target_link_directories(${APP_NAME} PUBLIC ${ImageApproxiPath})

target_link_libraries(${APP_NAME} ${OpenCV_LIBS})
target_link_libraries(${APP_NAME} opengl32 glu32)
target_link_libraries(${APP_NAME} Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml)
target_link_libraries(${APP_NAME} CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(${APP_NAME} debug ANNd optimized ANN)
target_link_libraries(${APP_NAME} debug OpenMeshCored optimized OpenMeshCore)
target_link_libraries(${APP_NAME} debug QGLViewerd2 optimized QGLViewer2)
target_link_libraries(${APP_NAME} sparse_quad_prog )

if(MATLAB_FOUND)
	target_include_directories(${APP_NAME} PUBLIC ${Matlab_INCLUDE_DIRS})
	target_link_directories(${APP_NAME} PUBLIC "${Matlab_ROOT_DIR}/extern/lib/win64/microsoft")
	target_link_libraries(${APP_NAME} mclmcrrt)
endif()

if(OpenMP_CXX_FOUND)
    target_link_libraries(${APP_NAME} OpenMP::OpenMP_CXX)
endif()

install_runtime_targets(${APP_NAME})

SET_TARGET_PROPERTIES(${APP_NAME} PROPERTIES FOLDER "ImageApproximation")

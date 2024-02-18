cmake_minimum_required(VERSION 3.3...3.12 FATAL_ERROR)
foreach(p
    CMP0071 # 3.10: Let AUTOMOC and AUTOUIC process GENERATED files
    )
  if(POLICY ${p})
    cmake_policy(SET ${p} NEW)
  endif()
endforeach()

function(aux_source_directories var folder)
    set(sources)
    foreach(dir ${ARGN})
        file(GLOB _sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${dir}/*.[ch]" "${dir}/*.[ch]pp")
        list(APPEND sources ${_sources})
    endforeach()
    
    if( NOT folder STREQUAL "")
        source_group(${folder} FILES ${sources})
    endif()

    #message("\nDEBUG: aux_source_directories: current_source_dir=${CMAKE_CURRENT_SOURCE_DIR} dirs=${ARGN}):\n${sources}\n")
    set(${var} ${${var}} ${sources} PARENT_SCOPE)
endfunction()

function(install_runtime_targets)
    install(
        TARGETS ${ARGN}
        COMPONENT runtime
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endfunction()

# Generate position independent code by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(ImageApproxi)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

string(REPLACE ${CMAKE_SOURCE_DIR} "" RELATIVE_OUTPUT_DIR ${CMAKE_BINARY_DIR})
if(WIN32)
        set(MSVC_CONFIG \$\(Configuration\))
        set(RELATIVE_BIN_DIR ${RELATIVE_OUTPUT_DIR}/bin/${MSVC_CONFIG}/)
        set(RELATIVE_LIB_DIR ${RELATIVE_OUTPUT_DIR}/lib/${MSVC_CONFIG}/)
else()
        set(RELATIVE_BIN_DIR ${RELATIVE_OUTPUT_DIR}/bin/)
        set(RELATIVE_LIB_DIR ${RELATIVE_OUTPUT_DIR}/lib/)
endif()

#opengl
include(${CMAKE_CURRENT_SOURCE_DIR}/opengl.cmake)

#cgal
find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

#qt
find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
find_package(Qt5 COMPONENTS Core REQUIRED QUIET)
find_package(Qt5 COMPONENTS OpenGL REQUIRED QUIET)
find_package(Qt5 COMPONENTS Xml REQUIRED QUIET)

#openmp
find_package(OpenMP)
IF(OpenMP_CXX_FOUND)
    message(STATUS "OPENMP Found.")
ELSE(OpenMP_CXX_FOUND)
    MESSAGE("OPENMP not found.")
ENDIF(OpenMP_CXX_FOUND)

include_directories( ${CMAKE_CURRENT_BINARY_DIR}  ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_LIB_DIR})
message(STATUS "LIB path: ${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_LIB_DIR}")

add_definitions(-w)

add_subdirectory(src/ImageApproxi)
add_subdirectory(src/Subdivision)
add_subdirectory(src/EditGui)


cmake_policy(SET CMP0077 NEW)

set(APP_NAME EditingGui)
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC OFF )

aux_source_directories(SOURCES "Source Files\\include"     			include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Resources DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

#qt
set( UIs ${CMAKE_CURRENT_SOURCE_DIR}/include/mainwindow.ui )
set( QRCs ${CMAKE_CURRENT_SOURCE_DIR}/include/mainwindow.qrc)
set( ExeIcon ${CMAKE_CURRENT_SOURCE_DIR}/include/resource.rc)
qt5_wrap_ui(UI_Srcs ${UIs})
qt5_add_resources(QRC_Srcs ${QRCs} )
set( MOC_Hdrs 
${CMAKE_CURRENT_SOURCE_DIR}/include/editing.h 
${CMAKE_CURRENT_SOURCE_DIR}/include/action.h 
${CMAKE_CURRENT_SOURCE_DIR}/include/action_definition.h 
${CMAKE_CURRENT_SOURCE_DIR}/include/action_list.h 
${CMAKE_CURRENT_SOURCE_DIR}/include/mainwindow.h
${CMAKE_CURRENT_SOURCE_DIR}/include/vecImage_window.h
${CMAKE_CURRENT_SOURCE_DIR}/include/operation_window.h
${CMAKE_CURRENT_SOURCE_DIR}/include/reference_window.h
${CMAKE_CURRENT_SOURCE_DIR}/include/viewer.h
${CMAKE_CURRENT_SOURCE_DIR}/include/basicViewer.h
)
QT5_WRAP_CPP(MOCS ${MOC_Hdrs})

#create files in VS
source_group("Resources" FILES ${UIs} ${QRCs})
source_group("Generated" FILES ${UI_Srcs} ${MOCS} ${QRC_Srcs})
source_group("Source" FILES ${SOURCES})

add_executable(${APP_NAME} ${SOURCES} ${QRC_Srcs} ${MOCS} ${UI_Srcs} ${ExeIcon})

set(EditGuiPath ${SubdivisionPath} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${APP_NAME} PUBLIC ${EditGuiPath})
target_link_libraries(${APP_NAME} PUBLIC SubdivisionMesh)

install_runtime_targets(${APP_NAME})

SET_TARGET_PROPERTIES(${APP_NAME} PROPERTIES FOLDER "ImageApproximation")

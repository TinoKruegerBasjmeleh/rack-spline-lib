cmake_minimum_required(VERSION 3.1)
project(rack-spline)

include(ExternalProject)  
SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

set(SISL_PROJECT_NAME sisl_lib)  
set(SISL_PROJECT_URL https://github.com/SINTEF-Geometry/SISL.git)  
set(SISL_PROJECT_TAG 72394ff1ae86d50a5ee7c910f5cefe7b7093811c) # or the version you want  

#set optional examples 
set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples folder")

ExternalProject_Add(  
  ${SISL_PROJECT_NAME}  
  GIT_REPOSITORY ${SISL_PROJECT_URL}  
  GIT_TAG ${SISL_PROJECT_TAG}  
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND ""
  )  


set(SOURCES
      sislnurbsgen.cpp
   )


add_library(${PROJECT_NAME} ${SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${SISL_PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
  ${CMAKE_INSTALL_PREFIX}/include/rack
  )

target_link_libraries(${PROJECT_NAME}
                      ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}-build/sisl
                      )

set_target_properties(${PROJECT_NAME} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF )

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif(BUILD_EXAMPLES)

install(FILES ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include/sisl.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(FILES ${CMAKE_SOURCE_DIR}//sislnurbsgen.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(FILES ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}-build/libsisl.a DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(FILES ${CMAKE_BINARY_DIR}/librack-spline.a DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

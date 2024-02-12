
set(EXAMPLE "create_splines")
set(SOURCES_EXAMPLE
     create_simple_splines.cpp
)

message("${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}-build/libsisl.a")

add_executable(${EXAMPLE} ${SOURCES_EXAMPLE})
#add_dependencies(${EXAMPLE} ${PROJECT_NAME})
target_include_directories(${EXAMPLE} PUBLIC
                           ${CMAKE_SOURCE_DIR}
                           ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                           ${CMAKE_INSTALL_PREFIX}/include/rack
                         )
target_link_libraries(${EXAMPLE}
                      "${CMAKE_BINARY_DIR}/librack-spline.a"
                      "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                      )

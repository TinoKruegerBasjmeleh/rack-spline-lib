
set(EXAMPLE "create_splines")
set(SOURCES_EXAMPLE
     create_simple_splines.cpp
)
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



set(INTERP "create_interpolation")
set(SOURCES_INTERP
                       create_interpolation_spline.cpp
                  )
add_executable(${INTERP} ${SOURCES_INTERP})
target_include_directories(${INTERP} PUBLIC
                                             ${CMAKE_SOURCE_DIR}
                                             ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                                             ${CMAKE_INSTALL_PREFIX}/include/rack
                                           )
target_link_libraries(${INTERP}
                                        "${CMAKE_BINARY_DIR}/librack-spline.a"
                                        "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                                        )



set(OFFSET "create_offset")
set(SOURCES_OFFSET
        create_offset_spline.cpp
         )
add_executable(${OFFSET} ${SOURCES_OFFSET})
target_include_directories(${OFFSET} PUBLIC
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
             ${CMAKE_INSTALL_PREFIX}/include/rack
                                                                               )
target_link_libraries(${OFFSET}
                                                                            "${CMAKE_BINARY_DIR}/librack-spline.a"
                                                                            "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                                                                            )



set(BLENDED "create_blended")
set(SOURCES_BLENDED
        create_blended_spline.cpp
         )
add_executable(${BLENDED} ${SOURCES_BLENDED})
target_include_directories(${BLENDED} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
    ${CMAKE_INSTALL_PREFIX}/include/rack
     )
    target_link_libraries(${BLENDED}
    "${CMAKE_BINARY_DIR}/librack-spline.a"
    "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
        )


set(TIMEMEASURE "measure_time")
set(SOURCES_TIMEMEASURE
        time_measure_spline.cpp
         )
add_executable(${TIMEMEASURE} ${SOURCES_TIMEMEASURE})
target_include_directories(${TIMEMEASURE} PUBLIC
                               ${CMAKE_SOURCE_DIR}
                               ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                               ${CMAKE_INSTALL_PREFIX}/include/rack
                             )
target_link_libraries(${TIMEMEASURE}
                          "${CMAKE_BINARY_DIR}/librack-spline.a"
                          "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a")
                          
                          
set(S_SHAPE "create_s_spline")
set(SOURCES_S_SHAPE
        create_s_spline.cpp
         )
add_executable(${S_SHAPE} ${SOURCES_S_SHAPE})
target_include_directories(${S_SHAPE} PUBLIC
                         ${CMAKE_SOURCE_DIR}
                         ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                         ${CMAKE_INSTALL_PREFIX}/include/rack
                       )
target_link_libraries(${S_SHAPE}
                                            "${CMAKE_BINARY_DIR}/librack-spline.a"
                                            "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                                            )

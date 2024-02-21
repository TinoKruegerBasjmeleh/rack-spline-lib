#------ create a simple curve
#------ add new function to generate a spline based on given knots

set(EXAMPLE "create_splines")
set(SOURCES_EXAMPLE
     create_simple_splines.cpp
   )

add_executable(${EXAMPLE} ${SOURCES_EXAMPLE})
target_include_directories(${EXAMPLE} PUBLIC
                           ${CMAKE_SOURCE_DIR}
                           ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                           ${CMAKE_INSTALL_PREFIX}/include/rack
                         )
target_link_libraries(${EXAMPLE}
                      "${CMAKE_BINARY_DIR}/librack-spline.a"
                      "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                      )

set_target_properties(${EXAMPLE} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF )

#------ create a interpolation curve
#------ add new function to generate a interpolation spline based on given points

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

set_target_properties(${INTERP} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF )

#------ create a offset curve
#------ add new function to generate a offset spline based on given spline

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

set_target_properties(${OFFSET} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF )

#------ blended curve
#------ add new function to generate a blended spline based two given splines

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


set_target_properties(${BLENDED} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF )

#------ measure time
#------ add new function to generate a spline  and measure duration for generation

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
                      "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                     )

set_target_properties(${TIMEMEASURE} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF)

#------ S-shape spline
#------  add new function to generate a s-shaped spline based on a given knots

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

set_target_properties(${S_SHAPE} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF)

#------ path based spline
#------ add new function to generate a spline based on a given path 

set(PATH_BASED_SPLINE "path_based_splines")
set(SOURCES_PATH_BASED_SPLINE
     path_based_spline.cpp
   )

add_executable(${PATH_BASED_SPLINE} ${SOURCES_PATH_BASED_SPLINE})

target_include_directories(${PATH_BASED_SPLINE} PUBLIC
                           ${CMAKE_SOURCE_DIR}
                           ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                           ${CMAKE_INSTALL_PREFIX}/include/rack
                         )

target_link_libraries(${PATH_BASED_SPLINE}
                      "${CMAKE_BINARY_DIR}/librack-spline.a"
                      "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                      )

set_target_properties(${PATH_BASED_SPLINE} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF)

#------ Calculate the cord length of a spline
#------ add new function and example to calculate the cord length of a given spline

set(CORD_LENGTH "cord_length")
set(SOURCES_CORD_LENGTH
     calculate_spline_length.cpp
   )

add_executable(${CORD_LENGTH} ${SOURCES_CORD_LENGTH})

target_include_directories(${CORD_LENGTH} PUBLIC
                           ${CMAKE_SOURCE_DIR}
                           ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                           ${CMAKE_INSTALL_PREFIX}/include/rack
                         )

target_link_libraries(${CORD_LENGTH}
                      "${CMAKE_BINARY_DIR}/librack-spline.a"
                      "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                      )

set_target_properties(${CORD_LENGTH} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF)

#------ blended full curve
#------ add new function to generate a blended spline based two given splines

set(BLENDED_FULL "create_blended_full")
set(SOURCES_BLENDED_FULL
      create_blended_full_spline.cpp
   )
add_executable(${BLENDED_FULL} ${SOURCES_BLENDED_FULL})
target_include_directories(${BLENDED_FULL} PUBLIC
                           ${CMAKE_SOURCE_DIR}
                           ${CMAKE_BINARY_DIR}/${SISL_PROJECT_NAME}-prefix/src/${SISL_PROJECT_NAME}/include
                           ${CMAKE_INSTALL_PREFIX}/include/rack
                            )
target_link_libraries(${BLENDED_FULL}
                      "${CMAKE_BINARY_DIR}/librack-spline.a"
                      "${CMAKE_BINARY_DIR}/sisl_lib-prefix/src/sisl_lib-build/libsisl.a"
                      )


set_target_properties(${BLENDED_FULL} PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF )

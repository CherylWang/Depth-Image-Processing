SET(APPICATION_NAME face_experiment)

SET(MODULE_DEPENDENCIES
    )

include(includes.cmake)

foreach(DEPENDENT ${MODULE_DEPENDENCIES})
  include(${PROJECT_SOURCE_DIR}/${DEPENDENT}/includes.cmake)

  set(INCLUDE_DIRS
    ${INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/${DEPENDENT}/include
    )
endforeach(DEPENDENT)

set(INCLUDE_DIRS
    ${INCLUDE_DIRS}
    )

include_directories(${INCLUDE_DIRS})

set(INCS
    )

set(SRCS
    ${APPICATION_NAME}.cpp
    )

set(LIBS
    ${MODULE_DEPENDENCIES}
    )

add_executable(${APPICATION_NAME} ${SRCS} ${INCS})
target_link_libraries(${APPICATION_NAME} ${LIBS})

install(TARGETS ${APPICATION_NAME} DESTINATION ${BIN_INSTALL_DIR})

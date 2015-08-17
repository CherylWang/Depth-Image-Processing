SET(APPICATION_NAME face_modeling)

SET(MODULE_DEPENDENCIES
    cameras
    common
    filters
    io
    point_cloud
    projects
    registration
    sampling
    segmentation
    surface
    visualization
    )

include(includes.cmake)

foreach(DEPENDENT ${MODULE_DEPENDENCIES})
  include(${PROJECT_SOURCE_DIR}/${DEPENDENT}/includes.cmake)

  set(INCLUDE_DIRS
    ${INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/${DEPENDENT}/include
    )
endforeach(DEPENDENT)

#add pcl
Find_PACKAGE(PCL REQUIRED)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# OpenCV
Find_PACKAGE(OpenCV REQUIRED)


set(INCLUDE_DIRS
    ${INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )




include_directories(${INCLUDE_DIRS})

set(INCS
    )

set(SRCS
    ${APPICATION_NAME}.cpp
    )

set(LIBS
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${MODULE_DEPENDENCIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_executable(${APPICATION_NAME} ${SRCS} ${INCS})
target_link_libraries(${APPICATION_NAME} ${LIBS})

install(TARGETS ${APPICATION_NAME} DESTINATION ${BIN_INSTALL_DIR})

if(WIN32)
  include(install.cmake)
endif()

ADD_LIBRARY(types_object_slam ${G2O_LIB_TYPE}
  types_object_slam.h
  types_object_slam.cpp
)

SET_TARGET_PROPERTIES(types_object_slam PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_object_slam)

TARGET_LINK_LIBRARIES(types_object_slam core types_slam3d)

INSTALL(TARGETS types_object_slam
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/object_slam)

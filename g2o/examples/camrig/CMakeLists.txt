if(CHOLMOD_INCLUDE_DIR OR CSPARSE_INCLUDE_DIR)
  add_executable(camrig_demo camrig_demo.cpp)
  target_link_libraries(camrig_demo core types_icp types_sba)
  set_target_properties(camrig_demo PROPERTIES OUTPUT_NAME camrig_demo${EXE_POSTFIX})
endif()

# Link CHOLMOD if available, CSparse otherwise
if(CHOLMOD_FOUND)
  include_directories(${CHOLMOD_INCLUDE_DIR})
  target_link_libraries(camrig_demo solver_cholmod)
elseif(CSPARSE_FOUND)
  include_directories(${CSPARSE_INCLUDE_DIR})
  target_link_libraries(camrig_demo solver_csparse)
endif()

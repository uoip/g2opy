include_directories(${PROJECT_SOURCE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})
include_directories(${CSPARSE_INCLUDE_DIR})


# pybind11 (version 2.13.6), vendored under EXTERNAL/pybind11.
# It is not tracked in git, so a fresh clone has to fetch it first.
IF(NOT EXISTS ${PROJECT_SOURCE_DIR}/EXTERNAL/pybind11/CMakeLists.txt)
  MESSAGE(FATAL_ERROR "EXTERNAL/pybind11 is missing. Run script/fetch_pybind11.sh, "
                      "then configure again.")
ENDIF()

# Only the sources needed to build are vendored, so pybind11's own CMakeLists.txt
# is brought in as a subdirectory to define the pybind11::* targets.
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/EXTERNAL/pybind11 ${CMAKE_CURRENT_BINARY_DIR}/pybind11)


pybind11_add_module(g2o g2o.cpp)
target_link_libraries(g2o PRIVATE 
    core
    solver_cholmod
    solver_csparse
    solver_eigen
    solver_dense
    solver_pcg
    solver_slam2d_linear
    solver_structure_only
    types_data
    types_icp
    types_sba
    types_sclam2d
    types_sim3
    types_slam2d
    types_slam2d_addons
    types_slam3d
    types_slam3d_addons
    contrib
)
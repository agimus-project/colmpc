cmake_minimum_required(VERSION 3.10)

# Project properties
set(PROJECT_NAMESPACE memory-of-motion)
set(PROJECT_NAME sobec)
set(PROJECT_DESCRIPTION "Sandbox for optimal control explicitly for bipeds")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(BUILD_PY2CPP_INTERFACE "Build the py2cpp library" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(GENERATE_PYTHON_STUBS
       "Generate the Python stubs associated to the Python library" OFF)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/stubs.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_compile_options(-Wno-error)

apply_default_apple_configuration()

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 2.7.10 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

add_project_dependency(ndcurves REQUIRED)
add_project_dependency(crocoddyl REQUIRED)
add_project_dependency(yaml-cpp REQUIRED)

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/flexibility_compensation.hpp
    include/${PROJECT_NAME}/python.hpp
    include/${PROJECT_NAME}/fwd.hpp
    include/${PROJECT_NAME}/crocomplements/residual-com-velocity.hpp
    include/${PROJECT_NAME}/crocomplements/residual-cop.hpp
    include/${PROJECT_NAME}/crocomplements/residual-feet-collision.hpp
    include/${PROJECT_NAME}/crocomplements/residual-vel-collision.hpp
    include/${PROJECT_NAME}/crocomplements/residual-fly-high.hpp
    include/${PROJECT_NAME}/crocomplements/residual-fly-angle.hpp
    include/${PROJECT_NAME}/crocomplements/residual-dcm-position.hpp
    include/${PROJECT_NAME}/crocomplements/residual-2D-surface.hpp
    include/${PROJECT_NAME}/crocomplements/activation-quad-ref.hpp
    include/${PROJECT_NAME}/crocomplements/activation-weighted-log.hpp
    include/${PROJECT_NAME}/crocomplements/lowpassfilter/state.hpp
    include/${PROJECT_NAME}/crocomplements/lowpassfilter/state.hxx
    include/${PROJECT_NAME}/crocomplements/lowpassfilter/action.hpp
    include/${PROJECT_NAME}/crocomplements/lowpassfilter/action.hxx
    include/${PROJECT_NAME}/crocomplements/contact/contact6d.hpp
    include/${PROJECT_NAME}/crocomplements/contact/contact3d.hpp
    include/${PROJECT_NAME}/crocomplements/contact/contact1d.hpp
    include/${PROJECT_NAME}/crocomplements/contact/multiple-contacts.hpp
    include/${PROJECT_NAME}/crocomplements/contact/contact-fwddyn.hpp
    include/${PROJECT_NAME}/crocomplements/contact/contact-force.hpp
    include/${PROJECT_NAME}/crocomplements/residual-com-velocity.hxx
    include/${PROJECT_NAME}/crocomplements/residual-cop.hxx
    include/${PROJECT_NAME}/crocomplements/residual-feet-collision.hxx
    include/${PROJECT_NAME}/crocomplements/residual-vel-collision.hxx
    include/${PROJECT_NAME}/crocomplements/contact/contact6d.hxx
    include/${PROJECT_NAME}/crocomplements/contact/contact3d.hxx
    include/${PROJECT_NAME}/crocomplements/contact/contact1d.hxx
    include/${PROJECT_NAME}/crocomplements/contact/multiple-contacts.hxx
    include/${PROJECT_NAME}/crocomplements/contact/contact-fwddyn.hxx
    include/${PROJECT_NAME}/crocomplements/contact/contact-force.hxx
    include/${PROJECT_NAME}/crocomplements/residual-fly-high.hxx
    include/${PROJECT_NAME}/crocomplements/residual-fly-angle.hxx
    include/${PROJECT_NAME}/crocomplements/residual-dcm-position.hxx
    include/${PROJECT_NAME}/crocomplements/residual-2D-surface.hxx
    include/${PROJECT_NAME}/walk-without-think/ocp.hpp
    include/${PROJECT_NAME}/walk-without-think/mpc.hpp
    include/${PROJECT_NAME}/walk-without-think/mpc.hxx
    include/${PROJECT_NAME}/walk-with-traj/designer.hpp
    include/${PROJECT_NAME}/walk-with-traj/model_factory.hpp
    include/${PROJECT_NAME}/walk-with-traj/horizon_manager.hpp
    # include/${PROJECT_NAME}/walk-with-traj/ocp.hpp
    include/${PROJECT_NAME}/walk-with-traj/wbc.hpp
    include/${PROJECT_NAME}/walk-with-traj/wbc_horizon.hpp
    include/${PROJECT_NAME}/walk-with-traj/foot_trajectory.hpp)

set(${PROJECT_NAME}_SOURCES
    src/walk-with-traj/designer.cpp
    src/walk-with-traj/model_factory.cpp
    src/walk-with-traj/horizon_manager.cpp
    # src/walk-with-traj/ocp.cpp
    src/walk-with-traj/wbc.cpp
    src/walk-with-traj/wbc_horizon.cpp
    src/walk-with-traj/foot_trajectory.cpp
    src/flexibility_compensation.cpp
    src/walk-without-think/ocp.cpp
    src/walk-without-think/ocp-running-models.cpp
    src/walk-without-think/ocp-terminal-model.cpp
    src/walk-without-think/ocp-params.cpp
    src/walk-without-think/ocp-robot.cpp
    src/walk-without-think/ocp-ref-forces.cpp
    src/walk-without-think/mpc-params.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC crocoddyl::crocoddyl
                                             ndcurves::ndcurves yaml-cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX
                                                 INSTALL_RPATH "\$ORIGIN")

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  if(BUILD_PY2CPP_INTERFACE)
    add_library(${PROJECT_NAME}_py2cpp SHARED
                src/py2cpp.cpp include/${PROJECT_NAME}/py2cpp.hpp)
    if(SUFFIX_SO_VERSION)
      set_target_properties(${PROJECT_NAME}_py2cpp
                            PROPERTIES SOVERSION ${PROJECT_VERSION})
    endif()
    install(FILES include/${PROJECT_NAME}/py2cpp.hpp
            DESTINATION include/${PROJECT_NAME})
    install(
      TARGETS ${PROJECT_NAME}_py2cpp
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION lib)
    target_link_libraries(${PROJECT_NAME}_py2cpp PUBLIC crocoddyl::crocoddyl)
    target_link_libraries(${PROJECT_NAME}_py2cpp PRIVATE ${PYTHON_LIBRARIES})
    target_include_directories(${PROJECT_NAME}_py2cpp
                               PRIVATE ${PYTHON_INCLUDE_DIRS})
    target_link_boost_python(${PROJECT_NAME}_py2cpp PRIVATE)
  endif()

  add_subdirectory(python)
endif()

# Unit tests
if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  add_subdirectory(tests)
endif()

# Benchmark
add_subdirectory(benchmark)

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

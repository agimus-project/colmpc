set(${PY_NAME}_SOURCES
    main.cpp
    std-containers.cpp
    residual-distance-collision.cpp
)

add_library(${PY_NAME}_pywrap SHARED ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME}_pywrap PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
set_target_properties(
  ${PY_NAME}_pywrap
  PROPERTIES PREFIX ""
  OUTPUT_NAME ${PY_NAME}
             SUFFIX ${PYTHON_EXT_SUFFIX})
if(UNIX)
  get_relative_rpath(${${PY_NAME}_INSTALL_DIR} ${PY_NAME}_INSTALL_RPATH)
  set_target_properties(${PY_NAME}_pywrap
                        PROPERTIES INSTALL_RPATH "${${PY_NAME}_INSTALL_RPATH}")
endif()
target_link_boost_python(${PY_NAME}_pywrap PRIVATE)
install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

if(GENERATE_PYTHON_STUBS)
  load_stubgen()
  generate_stubs(${CMAKE_CURRENT_BINARY_DIR} ${PROJECT_NAME} ${PYTHON_SITELIB}
                 ${PY_NAME}_pywrap)
endif(GENERATE_PYTHON_STUBS)

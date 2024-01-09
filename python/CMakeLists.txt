set(${PY_NAME}_SOURCES
    main.cpp
    std-containers.cpp
    residual-distance-collision.cpp
)

add_library(${PY_NAME}_pywrap SHARED ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME}_pywrap PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
target_include_directories(${PY_NAME}_pywrap SYSTEM
                           PRIVATE ${PYTHON_INCLUDE_DIR})
set_target_properties(
  ${PY_NAME}_pywrap
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
             PREFIX ""
             SUFFIX ${PYTHON_EXT_SUFFIX})
if(UNIX)
  get_relative_rpath(${${PY_NAME}_INSTALL_DIR} ${PY_NAME}_INSTALL_RPATH)
  set_target_properties(${PY_NAME}_pywrap
                        PROPERTIES INSTALL_RPATH "${${PY_NAME}_INSTALL_RPATH}")
endif()
target_link_boost_python(${PY_NAME}_pywrap PRIVATE)
install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

# set(${PY_NAME}_PYTHON
#     __init__.py pinbullet.py viewer_multiple.py talos_collections.py
#     repr_ocp.py logs.py
#     # talos_low.py
# )

if(GENERATE_PYTHON_STUBS)
  load_stubgen()
  generate_stubs(${CMAKE_CURRENT_BINARY_DIR} ${PROJECT_NAME} ${PYTHON_SITELIB}
                 ${PY_NAME}_pywrap)
endif(GENERATE_PYTHON_STUBS)

foreach(python ${${PY_NAME}_PYTHON})
  python_install_on_site(${PY_NAME} ${python})
endforeach()

# set(${PY_WALK_NAME}_PYTHON
#     __init__.py
#     ocp.py
#     weight_share.py
#     robot_wrapper.py
#     config_mpc.py
#     miscdisp.py
#     params.py
#     yaml_params.py
#     plotter.py
#     save_traj.py)

# foreach(python ${${PY_WALK_NAME}_PYTHON})
#   python_install_on_site(${PY_NAME}/walk_without_think ${python})
# endforeach()

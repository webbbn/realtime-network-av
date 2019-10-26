
# Find the python library.
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH} ${PROJECT_SOURCE_DIR}/modules/srt/srtcore ${PROJECT_SOURCE_DIR}/modules/srt/common ${PROJECT_BINARY_DIR}/modules/srt ${PROJECT_SOURCE_DIR}/python)

if (USE_CYTHON)

  # Compile with cython
  include(UseCython)

  # Build the py_srt cython interface
  cython_add_module(py_srt)
  link_directories(${PROJECT_BINARY_DIR})
  target_link_libraries(py_srt srt_shared ${PYTHON_LIBRARY} ${Extra_Libraries})

  # Build the py_v4l2 cython interface
  cython_add_module(py_v4l2)
  target_link_libraries(py_v4l2  ${PYTHON_LIBRARY} ${Extra_Libraries} v4l2)

  # Install the cython targets.
  install(TARGETS py_srt py_v4l2 DESTINATION ${PYTHON_INSTALL_DIR})

endif ()

# Install the python executable scripts
install(PROGRAMS fc_comm.py pycamera_tx.py fpvng_controller.py DESTINATION bin)

# Install the python libraries
install(FILES
  format_as_table.py
  network.py
  camera.py
  telemetry.py
  MultiWii.py
  transmitter.py
  udp_relay.py
  DESTINATION ${PYTHON_INSTALL_DIR})

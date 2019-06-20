
# Find the python library.
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH} ${PROJECT_SOURCE_DIR}/modules/srt/srtcore ${PROJECT_SOURCE_DIR}/modules/srt/common ${PROJECT_BINARY_DIR}/modules/srt ${PROJECT_SOURCE_DIR}/python)

if (USE_CYTHON)

  # Compile with cython
  include(UseCython)

  # If the pyx file is a C++ file, we should specify that here.
  set_source_files_properties(fec.pyx PROPERTIES CYTHON_IS_CXX TRUE)

  cython_add_module(py_srt)
  link_directories(${PROJECT_BINARY_DIR})
  target_link_libraries(py_srt srt_shared ${PYTHON_LIBRARY} ${Extra_Libraries})

  cython_add_module(py_v4l2)
  target_link_libraries(py_v4l2  ${PYTHON_LIBRARY} ${Extra_Libraries} v4l2)

  cython_add_module(fec fec.pyx ${PROJECT_SOURCE_DIR}/src/fec.c)
  target_link_libraries(fec ${PYTHON_LIBRARY})

  install(TARGETS py_srt py_v4l2 fec DESTINATION python)

endif ()

set(PYFILES fc_comm.py pycamera_tx.py)

install(PROGRAMS fc_comm.py pycamera_tx.py wifi_info.py DESTINATION bin)
install(FILES format_as_table.py DESTINATION python)

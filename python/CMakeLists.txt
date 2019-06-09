
# Compile with cython
include(UseCython)

# Find the python library.
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH} ${PROJECT_SOURCE_DIR}/modules/srt/srtcore ${PROJECT_SOURCE_DIR}/modules/srt/common ${PROJECT_BINARY_DIR}/modules/srt ${PROJECT_SOURCE_DIR}/python)

link_directories(${PROJECT_BINARY_DIR})
cython_add_module(py_srt)
target_link_libraries(py_srt srt ${PYTHON_LIBRARY} ${Extra_Libraries})

cython_add_module(py_v4l2)
target_link_libraries(py_v4l2  ${PYTHON_LIBRARY} ${Extra_Libraries} v4l2)

set(PYFILES fc_comm.py pycamera_tx.py)

install(TARGETS py_srt py_v4l2 DESTINATION python)
install(PROGRAMS fc_comm.py pycamera_tx.py wifi_info.py DESTINATION bin)
install(FILES format_as_table.py DESTINATION python)

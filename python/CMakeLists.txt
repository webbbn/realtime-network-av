
# Compile with cython
include(UseCython)

# Find the python library.
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH} ${PROJECT_SOURCE_DIR}/modules/srt/srtcore ${PROJECT_BINARY_DIR}/modules/srt)

link_directories(${PROJECT_BINARY_DIR})
cython_add_module(py_srt)
target_link_libraries(py_srt srt ${PYTHON_LIBRARY})

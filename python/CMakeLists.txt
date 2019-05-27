include(UseCython)

cython_add_module(py_srt)
target_link_libraries(py_srt srt)

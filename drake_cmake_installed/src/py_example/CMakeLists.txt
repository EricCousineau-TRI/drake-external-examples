pybind11_add_module(py_example py_example.cc)
target_link_libraries(py_example PUBLIC pybind11::module)

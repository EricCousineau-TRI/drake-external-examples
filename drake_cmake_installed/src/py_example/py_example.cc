#include <pybind11/pybind11.h>

int incr(int x) {
    return x + 1;
}

namespace py = pybind11;

PYBIND11_MODULE(py_example, m) {
    m.doc() = "Example module using pybind11 from Drake.";
    m.def("incr", &incr);
}

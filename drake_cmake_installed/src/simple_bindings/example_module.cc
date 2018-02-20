#include <pybind11/pybind11.h>

#include <drake/systems/leaf_system.h>

namespace py = pybind11;

using drake::systems;
using systems::BasicVector;
using systems::Context;
using systems::LeafSystem;

namespace shambhala {
namespace {

using T = double;

/// Adds a constant to an input.
class SimpleAdder : public LeafSystem<T> {
 public:
  SimpleAdder(T add) {
    this->DeclareInputPort(systems::kVectorValued, 1);
    this->DeclareVectorOutputPort(
        [add](const Context<T>& context,
              drake::systems::BasicVector<T>* output) {
          auto u = this->EvalEigenVectorInput(context, 0);
          output->get_mutable_value() += u + add;
        });
  }
};

PYBIND11_MODULE(example_module, m) {
  m.doc() = "Example module interfacing with pydrake and Drake C++";

  py::module::import("pydrake.framework.systems");

  py::class_<SimpleAdder, LeafSystem<T>>(m, "SimpleAdder")
      .def(py::init<T>(), py::arg("add"));
}

}  // namespace shambhala

#include "simple_adder.h"

namespace shambhala {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::kVectorValued;

template <typename T>
SimpleAdder<T>::SimpleAdder(T add)
      : add_(add) {
  this->DeclareInputPort(kVectorValued, 1);
  this->DeclareVectorOutputPort(
      BasicVector<T>(1), &SimpleAdder::CalcOutput);
}

template <typename T>
void SimpleAdder<T>::CalcOutput(
    const Context<T>& context, BasicVector<T>* output) const {
  auto u = this->EvalEigenVectorInput(context, 0);
  auto&& y = output->get_mutable_value();
  y.array() = u.array() + add_;
}

}  // namespace shambhala

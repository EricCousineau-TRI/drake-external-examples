/**
 * @file
 * Provides an example of creating a simple Drake C++ system that will later be
 * bound.
 */

#include <drake/systems/framework/leaf_system.h>

namespace shambhala {

/// Adds a constant to an input.
template <typename T>
class SimpleAdder : public drake::systems::LeafSystem<T> {
 public:
  explicit SimpleAdder(T add);

 private:
  void CalcOutput(
      const drake::systems::Context<T>& context,
      drake::systems::BasicVector<T>* output) const;

  const T add_{};
};

}  // namespace shambhala

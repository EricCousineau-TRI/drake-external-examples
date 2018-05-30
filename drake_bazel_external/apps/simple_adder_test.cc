#include <iostream>

#include <drake/common/text_logging.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/signal_logger.h>

#include "simple_adder.h"

using drake::systems::Simulator;
using drake::systems::DiagramBuilder;
using drake::systems::ConstantVectorSource;
using drake::systems::SignalLogger;

namespace shambhala {
namespace {

int DoMain() {
  DiagramBuilder<double> builder;
  auto source = builder.AddSystem<ConstantVectorSource<double>>(
      Eigen::VectorXd::Constant(1, 10.));
  auto adder = builder.AddSystem<SimpleAdder<double>>(100.);
  builder.Connect(source->get_output_port(), adder->get_input_port(0));
  auto logger = builder.AddSystem<SignalLogger<double>>(1);
  builder.Connect(adder->get_output_port(0), logger->get_input_port());
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  simulator.StepTo(1);

  auto x = logger->data();
  Eigen::VectorXd x_expected = Eigen::Vector2d(110., 110.);
  std::cout << "Output values: " << x << std::endl;
  DRAKE_DEMAND(x.isApprox(x_expected));

  return 0;
}

}  // namespace
}  // namespace shambhala

int main() {
  return shambhala::DoMain();
}

#include <drake/common/text_logging.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/signal_logger.h>

#include "simple_adder.h"

using drake::systems::Simulator:
using drake::systems::DiagramBuilder;
using drake::systems::ConstantVectorSource;
using drake::systems::SignalLogger;

int main() {
  DiagramBuilder builder;

  auto source = builder.AddSystem<ConstantVectorSource>(
      Eigen::Vector1d::Constant(10.));
  auto adder = builder.AddSystem<SimpleAdder>(100.);
  builder.Connect(source->get_output_port(0), adder->get_input_port(0));
  auto logger = builder.AddSystem<SignalLogger>(1);
  builder.Connect(adder->get_output_port(0), logger->get_input_port(0));

  auto diagram = diagram.Build();

  Simulator simulator(*diagram);
  simulator.StepTo(1);

  auto x = logger->data();
  drake::log()->info("Output values: {}", x.transpose());
  return 0;
}

// Simple Discrete Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple discrete time system,
// simulates it from a given initial condition, and checks the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include <chrono>
#include <thread>
#include "drake/common/proto/call_python.h"
#include <iostream>
#include <Eigen/Core>

namespace drake {
namespace systems {
namespace {

// Simple Discrete Time System
//   x_{n+1} = x_n³
//         y = x
class SimpleDiscreteTimeSystem : public LeafSystem<double> {
 public:
  SimpleDiscreteTimeSystem() {
    DeclarePeriodicDiscreteUpdateEvent(1.0, 0.0,
                                       &SimpleDiscreteTimeSystem::Update);
    auto state_index = DeclareDiscreteState(1);  // One state variable.
    DeclareStateOutputPort("y", state_index);
  }

 private:
  // x_{n+1} = x_n³
  void Update(const Context<double>& context,
              DiscreteValues<double>* next_state) const {
    const double x_n = context.get_discrete_state()[0];
    (*next_state)[0] = std::pow(x_n, 3.0);
  }
};

int main() {
  // Create the simple system.
  double dt = 1.0;
  double T = 10.0;
  int N = static_cast<int>(T/dt + 1.0);
  SimpleDiscreteTimeSystem system;

  // Create the simulator.
  Simulator<double> simulator(system);

  // Set the initial conditions x₀.
  DiscreteValues<double>& state =
      simulator.get_mutable_context().get_mutable_discrete_state();
  state[0] = 0.99;

  simulator.Initialize();
  const auto& context = simulator.get_context();
  Eigen::VectorXd time_steps;
  time_steps.resize(N);
  Eigen::VectorXd state_values;
  state_values.resize(N);
  time_steps[0] = 0;
  state_values[0] =  state[0];
  int i = 0;

  while (context.get_time() < T) { // Simulate until 10 seconds.
    simulator.AdvanceTo(context.get_time() + dt); // Advance by 0.1 seconds.
    i++;
    time_steps[i] = context.get_time();
    state_values[i] = state[0];
  }

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(state[0] < 1.0e-4);

  common::CallPython("figure", 1);
  common::CallPython("clf");
  common::CallPython("plot", time_steps, state_values, "r");

  return 0;
}

}  // namespace
}  // namespace systems
}  // namespace drake

int main() {
  return drake::systems::main();
}

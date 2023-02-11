//
// Created by luohx on 23-2-10.
//

#pragma

#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

namespace legged {
using namespace ocs2;

class QpSolver : public solverBase {
public:
  QpSolver();

  ~QpSolver() override;

  void reset() override;

  void reset() override;

  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }

  const ProblemMetrics& getSolutionMetrics() const override { return problemMetrics_; }

  size_t getNumIterations() const override { return totalNumIterations_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { return ocpDefinitions_.front(); }

  const PerformanceIndex& getPerformanceIndeces() const override { return getIterationsLog().back(); };

  const std::vector<PerformanceIndex>& getIterationsLog() const override;

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override;

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override {
    throw std::runtime_error("[SqpSolver] getHamiltonian() not available yet.");
  }

  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    throw std::runtime_error("[SqpSolver] getStateInputEqualityConstraintLagrangian() not available yet.");
  }

  MultiplierCollection getIntermediateDualSolution(scalar_t time) const override {
    throw std::runtime_error("[SqpSolver] getIntermediateDualSolution() not available yet.");
  }

private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override {
    if (externalControllerPtr == nullptr) {
      runImpl(initTime, initState, finalTime);
    } else {
      throw std::runtime_error("[SqpSolver::run] This solver does not support external controller!");
    }
  }

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) override {
    // Copy all except the controller
    primalSolution_.timeTrajectory_ = primalSolution.timeTrajectory_;
    primalSolution_.stateTrajectory_ = primalSolution.stateTrajectory_;
    primalSolution_.inputTrajectory_ = primalSolution.inputTrajectory_;
    primalSolution_.postEventIndices_ = primalSolution.postEventIndices_;
    primalSolution_.modeSchedule_ = primalSolution.modeSchedule_;
    runImpl(initTime, initState, finalTime);
  }


};

} // namespace legged
//
// Created by luohx on 23-2-10.
//

#pragma

#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_core/initialization/Initializer.h>

#include <qpOASES.hpp>

namespace legged {

using namespace ocs2;

class ConvexMpc final : public MPC_BASE {
public:
  ConvexMpc(mpc::Settings mpcSetting, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
  : MPC_BASE(std::move(mpcSetting)) {
  }

  ~ConvexMpc() override = default;

  SolverBase* getSolverPtr() override { return solverPtr_.get(); }
  const SolverBase* getSolverPtr() const override { return solverPtr_.get(); }

protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    if (settings().coldStart_) {
      solverPtr_->reset();
    }
    solverPtr_->run(initTime, initState, finalTime);
  }

private:
  std::unique_ptr<SolverBase> solverPtr_;
};

}
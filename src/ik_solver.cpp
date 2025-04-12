#include "ik_solver.hpp"

namespace rix {
namespace rdf {

IKSolver::IKSolver(Tree &tree) : tree(tree) {}

rix::msg::sensor::JS IKSolver::solve(const std::string &link_name, const Eigen::Affine3d &goal,
                                     const rix::msg::sensor::JS &initial_guess, double step_scale, double tolerance,
                                     uint32_t max_iterations, bool &converged) {
    /**
     * TODO: Implement the inverse kinematics solver. This function should 
     * return the joint state that converged to the goal pose. The function
     * should also set the converged flag to true if the solution converged
     * within the specified tolerance and maximum iterations.
     */
    return {};
}

std::vector<rix::msg::sensor::JS> IKSolver::solve_with_trajectory(const std::string &link_name,
                                                                  const Eigen::Affine3d &goal,
                                                                  const rix::msg::sensor::JS &initial_guess,
                                                                  double step_scale, double tolerance,
                                                                  uint32_t max_iterations, bool &converged) {
    /**
     * TODO: Implement the inverse kinematics solver with trajectory. This 
     * function should return a vector of joint states that represent the
     * configuration of the robot at each step of the gradient descent
     * algorithm. The first element of the vector should be the initial guess, 
     * and the last element should be the final configuration that converged to 
     * the goal pose. The function should also set the converged flag to true
     * if the solution converged within the specified tolerance and maximum
     * iterations.
     */
    return {};
}

}  // namespace rdf
}  // namespace rix
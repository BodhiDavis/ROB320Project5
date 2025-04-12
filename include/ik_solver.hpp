#pragma once

#include "rix/msg/geometry/Pose.hpp"
#include "rix/rdf/fk_solver.hpp"
#include "rix/rdf/tree.hpp"

namespace rix {
namespace rdf {

class IKSolver {
   public:
    IKSolver(Tree &tree);
    
    rix::msg::sensor::JS solve(const std::string &link_name, const Eigen::Affine3d &goal_pose,
                               const rix::msg::sensor::JS &initial_guess, double step_scale, double tolerance,
                               uint32_t max_iterations, bool &converged);

    std::vector<rix::msg::sensor::JS> solve_with_trajectory(const std::string &link_name,
                                                            const Eigen::Affine3d &goal_pose,
                                                            const rix::msg::sensor::JS &initial_guess,
                                                            double step_scale, double tolerance,
                                                            uint32_t max_iterations, bool &converged);

   private:
    Tree &tree;

    /**
     * TODO: Declare any private member variables or methods you may need.
     * 
     * Hint: You may want to declare a private method that computes a single 
     * iteration of the gradient descent algorithm. This way, both of the solve
     * functions can call the same method to perform the gradient descent step,
     * but one can track the trajectory while the other does not.
     */
};

}  // namespace rdf
}  // namespace rix
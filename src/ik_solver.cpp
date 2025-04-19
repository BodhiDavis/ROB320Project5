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
       rix::msg::sensor::JS current_js = initial_guess;
       converged = false;
       FKSolver fk(tree);
       
       for (uint32_t iter = 0; iter < max_iterations; ++iter) {
            tree.set_state(current_js);
            Eigen::Affine3d current_pose = fk.solve(link_name);
            
            // position error
            double position_error = (goal.translation() - current_pose.translation()).norm();

            // orientation error
            Eigen::Matrix3d rotation_error = goal.rotation() * current_pose.rotation().transpose();
            Eigen::AngleAxisd aa(rotation_error);
            double orientation_error = aa.angle();
            
            double total_error = position_error + orientation_error;
            
            // std::cout << "Iteration: " << iter << ", Total Error: " << total_error<< " Tolerance: " << tolerance << std::endl;
            if (total_error <= tolerance) {
                converged = true;
                break;
            }
            
            current_js = gradient_descent_step(link_name, goal, current_js, step_scale);
       }
       
       return current_js;
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
     
    std::vector<rix::msg::sensor::JS> trajectory;
    trajectory.push_back(initial_guess);
    rix::msg::sensor::JS current_js = initial_guess;
    converged = false;
    FKSolver fk(tree);
    
    for (uint32_t iter = 0; iter < max_iterations; ++iter) {
        tree.set_state(current_js);
        
        Eigen::Affine3d current_pose = fk.solve(link_name);
        
        // position error
        double position_error = (goal.translation() - current_pose.translation()).norm();

        // orientation error
        Eigen::Matrix3d rotation_error = goal.rotation() * current_pose.rotation().transpose();
        Eigen::AngleAxisd aa(rotation_error);
        double orientation_error = aa.angle();
        
        double total_error = position_error + orientation_error;
        
        // // Check for convergence
        // std::cout << "Trajectory Iteration: " << iter << ", Total Error: " << total_error 
        //         << " Tolerance: " << tolerance << std::endl;
                
        if (total_error <= tolerance) {
            converged = true;
            break;
        }
        
        current_js = gradient_descent_step(link_name, goal, current_js, step_scale);
        
        trajectory.push_back(current_js);
    }

    return trajectory;
}

}  // namespace rdf
}  // namespace rix
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
      
   //TODO create a method to compute the Jacobian matrix for a given robot configuration
   Eigen::MatrixXd get_jacobian(const std::string &end_effector) {
      // Returns the jacobian of a speicified link of the robot arm. 

      std::vector<std::string> joint_names = tree.get_joint_names();
      int num_joints = joint_names.size();
      
      // Initialize Jacobian 
      Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, num_joints);
      
      // Get end effector/link position
      FKSolver fk(tree);
      Eigen::Affine3d ee_pose = fk.solve(end_effector);
      Eigen::Vector3d P_w = ee_pose.translation();
      
      // For each joint, calculate its contribution to the Jacobian
      for(int index = 0; index < num_joints; index++) {
         std::string joint_name = joint_names[index];
         auto current_joint = tree.get_joint(joint_name);
         auto current_joint_transform_to_world = fk.solve(current_joint.get_parent()) * current_joint.get_origin();
         
         auto r_i = P_w - current_joint_transform_to_world.translation();
         Eigen::Vector3d joint_axis_world_frame = current_joint_transform_to_world.linear() * current_joint.get_axis();
         
         if(current_joint.get_type() == rix::rdf::JointType::FIXED) {
             jacobian.block<3,1>(0, index) = Eigen::Vector3d::Zero();
             jacobian.block<3,1>(3, index) = Eigen::Vector3d::Zero();
         }

         else if(current_joint.get_type() == rix::rdf::JointType::PRISMATIC)
         {
             // for prismatic joints
             Eigen::Vector3d linear_vel = joint_axis_world_frame;
             Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
             
             jacobian.block<3,1>(0, index) = linear_vel;
             jacobian.block<3,1>(3, index) = angular_vel;
            
         }

         else{
            // for revolute/continuous joints
            Eigen::Vector3d linear_vel = joint_axis_world_frame.cross(r_i);
            Eigen::Vector3d angular_vel = joint_axis_world_frame;

            jacobian.block<3,1>(0, index) = linear_vel;
            jacobian.block<3,1>(3, index) = angular_vel;
         }
      }
      
      return jacobian;

   }

   //TODO create a method to compute the psuedoinverse of the jacobian
   Eigen::MatrixXd get_psuedo(Eigen::MatrixXd &jacobian) {
      // Returns the psuedoinverse of the jacobian matrix with damping
      Eigen::MatrixXd jacobian_pseudo;

      if(6 < jacobian.cols()) {
          // Jacobian is wide, more than 6 DOFs
          // right pseudo-inverse
          jacobian_pseudo = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
      }

      else if(6 == jacobian.cols()) {
          // Jacobian is square
          jacobian_pseudo = jacobian.inverse();
      }

      else {
          // Jacobian is tall, less than 6 DOFs
          // left pseudo-inverse
          jacobian_pseudo = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();
      }

      return jacobian_pseudo;
  }

   //TODO create a method to perform gradient descent for iterative optimization
   rix::msg::sensor::JS gradient_descent_step(
                                             const std::string &link_name,
                                             const Eigen::Affine3d &goal_pose,
                                             const rix::msg::sensor::JS &current_js,
                                             double step_scale)
  {
      FKSolver fk(tree);
      Eigen::Affine3d current_pose = fk.solve(link_name);
      Eigen::VectorXd error = Eigen::VectorXd::Zero(6);
      error.head<3>() = goal_pose.translation() - current_pose.translation();
      Eigen::Matrix3d rotation_error = goal_pose.rotation() * current_pose.rotation().transpose();
      Eigen::AngleAxisd aa(rotation_error);
      error.tail<3>() = aa.axis() * aa.angle();
      
      // Calculate Jacobian and its pseudoinverse
      Eigen::MatrixXd J = get_jacobian(link_name);
      Eigen::MatrixXd J_pinv = get_psuedo(J);
      
      // Compute delta_q
      Eigen::VectorXd delta_q = step_scale * J_pinv * error;      

      rix::msg::sensor::JS new_js = current_js;
      
      // Apply updates based on matching joint names
      std::vector<std::string> joint_names = tree.get_joint_names();
      for (size_t i = 0; i < new_js.joint_states.size(); i++) {
          std::string joint_name = new_js.joint_states[i].name;
          
          // Find this joint's index in the Jacobian columns
          auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
          if (it != joint_names.end()) {
              int j = std::distance(joint_names.begin(), it);
              new_js.joint_states[i].position += delta_q[j];
          }
      }
      
      return new_js;
  }

};

}  // namespace rdf
}  // namespace rix
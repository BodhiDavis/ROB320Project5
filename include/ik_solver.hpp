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
   Eigen::MatrixXd get_jacobian(){
      // Returns the jacobian of the endeffector of the robot arm. 
      // We will consider the forward kinematics of the endeffector and that each joint 
      // represents a degree of freedom(assumes revolute joints)
      
      //initialize the jacobian      

      //STEPS TO SOLVE JACOBIAN

      // 1.) GET THE VELOCITY VECTOR OF THE END EFFECTOR
      // 1A.) IN ORDER TO DO THIS, GO THROUGH ALL JOINTS IN THE KINEMATIC CHAIN AND CHECK IF THEY ARE REVOLUTE OR PRISMATIC
      // 1B.) IF A JOINT IS REVOLUTE, NOTE THEIR ANGULAR VELOCITY, IF THEY ARE PRISMATIC, NOTE THE LINEAR VELOCITY

      // FKSolver fk(tree);
      // Eigen::Affine3d forward_kinematics = fk.solve(tree.get_end_effectors()[0]);
      // auto position_vector = forward_kinematics.translation();

      // std::vector<std::string> joint_names = tree.get_joint_names();
      // int num_joints = joint_names.size();
      // Eigen::Matrix<Eigen::Vector3d, 1, Eigen::Dynamic> jacobian = Eigen::Matrix<Eigen::Vector3d, 1, Eigen::Dynamic>::Zero(1, num_joints);

      // int index = 0;
      // for(std::string &joint_name : joint_names)
      // {
      //    auto current_joint = tree.get_joint(joint_name).;
      //    auto current_axis = current_joint.get_axis();
      //    auto current_origin = current_joint.get_origin();
      //    // REVIEW - check that this is the correct way to do the equation in slide.
      //    if(current_joint.get_type() == rix::rdf::JointType::REVOLUTE)
      //    {
      //       Eigen::Vector3d lhs = current_axis - current_origin.linear();
      //       Eigen::Vector3d rhs = forward_kinematics.linear() - current_origin.linear();

      //       // take the derivative by taking the cross product of the angular velocity and the vector.
      //       // REVIEW - why are we taking the derivative of the the forward kinematics wrt time instead of taking the partial derivative wrt the joints????
      //       auto Jvi = lhs.cross(rhs);
      //       jacobian(0,index) = Jvi;
      //    }
         
      // }

      // return jacobian;


      // Get number of joints for sizing the Jacobian
      std::vector<std::string> joint_names = tree.get_joint_names();
      int num_joints = joint_names.size();
      
      // Initialize Jacobian matrix (6 rows for position/orientation, columns for each joint)
      Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, num_joints);
      
      // Get end effector position
      FKSolver fk(tree);
      std::string end_effector = tree.get_end_effectors()[0];
      Eigen::Affine3d ee_pose = fk.solve(end_effector);
      Eigen::Vector3d ee_position = ee_pose.translation();
      
      // For each joint, calculate its contribution to the Jacobian
      for(int index = 0; index < num_joints; index++) {
         std::string joint_name = joint_names[index];
         auto current_joint = tree.get_joint(joint_name);
         
         // Get joint position and rotation axis
         Eigen::Vector3d joint_axis = current_joint.get_axis();
         Eigen::Vector3d joint_position = current_joint.get_origin().translation();
         
         if(current_joint.get_type() == rix::rdf::JointType::REVOLUTE)
         {
            // For revolute joints:
            // Linear velocity component: z_i × (p_end - p_i)
            Eigen::Vector3d position_diff = ee_position - joint_position;
            Eigen::Vector3d linear_vel = joint_axis.cross(position_diff);
            
            // Angular velocity component: z_i (rotation axis)
            Eigen::Vector3d angular_vel = joint_axis;
            
            // Fill in the Jacobian column for this joint
            jacobian.block<3,1>(0, index) = linear_vel;   // Top 3 rows: linear velocity
            jacobian.block<3,1>(3, index) = angular_vel;  // Bottom 3 rows: angular velocity
         }

         else{
              // for prismatic joints
            
            // Angular velocity component: z_i (rotation axis)
            Eigen::Vector3d linear_vel = joint_axis;
            Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
            
            // Fill in the Jacobian column for this joint
            jacobian.block<3,1>(0, index) = linear_vel;   // Top 3 rows: linear velocity
            jacobian.block<3,1>(3, index) = angular_vel;  // Bottom 3 rows: angular velocity
         }
      }
      
      return jacobian;// Get number of joints for sizing the Jacobian
      std::vector<std::string> joint_names = tree.get_joint_names();
      int num_joints = joint_names.size();
      
      // Initialize Jacobian matrix (6 rows for position/orientation, columns for each joint)
      Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, num_joints);
      
      // Get end effector position
      FKSolver fk(tree);
      std::string end_effector = tree.get_end_effectors()[0];
      Eigen::Affine3d ee_pose = fk.solve(end_effector);
      Eigen::Vector3d ee_position = ee_pose.translation();
      
      // For each joint, calculate its contribution to the Jacobian
      for(int index = 0; index < num_joints; index++) {
         std::string joint_name = joint_names[index];
         auto current_joint = tree.get_joint(joint_name);
         
         // Get joint position and rotation axis
         Eigen::Vector3d joint_axis = current_joint.get_axis();
         Eigen::Vector3d joint_position = current_joint.get_origin().translation();
         
         // For revolute joints:
         // Linear velocity component: z_i × (p_end - p_i)
         Eigen::Vector3d position_diff = ee_position - joint_position;
         Eigen::Vector3d linear_vel = joint_axis.cross(position_diff);
         
         // Angular velocity component: z_i (rotation axis)
         Eigen::Vector3d angular_vel = joint_axis;
         
         // Fill in the Jacobian column for this joint
         jacobian.block<3,1>(0, index) = linear_vel;   // Top 3 rows: linear velocity
         jacobian.block<3,1>(3, index) = angular_vel;  // Bottom 3 rows: angular velocity
      }
      
      return jacobian;

   }

   //TODO create a method to compute the psuedoinverse of the jacobian

   //TODO create a method to perform gradient descent for iterative optimization
};

}  // namespace rdf
}  // namespace rix
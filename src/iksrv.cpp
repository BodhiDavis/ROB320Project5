#include <functional>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ik_solver.hpp"
#include "rix/core/node.hpp"
#include "rix/core/service.hpp"
#include "rix/util/cl_parser.hpp"
#include "rix/util/log.hpp"

/**
 * TODO: Include any necessary headers here.
 */

using CLParser = rix::util::CLParser;
using Log = rix::util::Log;
#include "rix/msg/iksrv/IKRequest.hpp"
#include "rix/msg/iksrv/IKResponse.hpp"

/**
 * TODO: Declare any additional aliases or using directives here.
 */
using Node = rix::core::Node;
using Service = rix::core::Service;
using JS = rix::msg::sensor::JS;
using json = nlohmann::json;
using request = rix::msg::iksrv::IKRequest;
using response = rix::msg::iksrv::IKResponse;


/**
 * TODO: Declare any necessary classes or functions here.
 */
class IKService {
    public:
        IKService(const std::string& jrdf_file) {
            // Load the robot description from JRDF
            std::ifstream file(jrdf_file);
            if (!file.is_open()) {
                throw std::runtime_error("Failed to open JRDF file: " + jrdf_file);
            }
            
            json js;
            file >> js;
            
            // Create the robot tree from JSON
            tree = std::make_unique<rix::rdf::Tree>(js);
            
            // Create the IK solver
            solver = std::make_unique<rix::rdf::IKSolver>(*tree);
            
            
            // Register the IK service            
            service = Node::advertise_service<request, response>("ik", 
                [this](const request& req, response& res) {
                    return handle_ik_request(req, res);
                });
                
            if (!service.ok()) {
                throw std::runtime_error("Failed to advertise IK service");
            }
        }
        
        // Start the service node
        void run() {
            Node::spin(true);
        }
        
    private:
        std::unique_ptr<rix::rdf::Tree> tree;
        std::unique_ptr<rix::rdf::IKSolver> solver;
        Service service;
        std::mutex solver_mutex; 
        
        bool handle_ik_request(const request& req, response& res) {
            std::lock_guard<std::mutex> lock(solver_mutex);
            try {
                Log::info << "Received IK request for end effector: " << req.end_effector << std::endl;
                
                Eigen::Affine3d goal_pose = Eigen::Affine3d::Identity();
                goal_pose.translation() = Eigen::Vector3d(
                    req.goal.translation.x, 
                    req.goal.translation.y, 
                    req.goal.translation.z
                );

                Eigen::Quaterniond q(
                    req.goal.rotation.w,
                    req.goal.rotation.x, 
                    req.goal.rotation.y, 
                    req.goal.rotation.z
                );
                
                goal_pose.linear() = q.toRotationMatrix();
                res.debug = req.debug;
                
                bool converged = false;
                
                if (req.debug) {
                    //TODO If debug is true, use solve_with_trajectory
                    std::vector<JS> trajectory = solver->solve_with_trajectory(
                        req.end_effector,
                        goal_pose,
                        req.initial_guess,
                        req.step_scale,
                        req.tolerance,
                        req.max_iterations,
                        converged
                    );
                    
                    if (!trajectory.empty()) {
                        res.result = trajectory.back();
                        res.steps = trajectory;
                    }
                } 
                
                else {
                    //TODO If debug is false, use solve()
                    res.result = solver->solve(
                        req.end_effector,
                        goal_pose,
                        req.initial_guess,
                        req.step_scale,
                        req.tolerance,
                        req.max_iterations,
                        converged
                    );
                }
                
                // Set convergence status in response
                res.converged = converged;
                return true;
                
            } 
            
            catch (const std::exception& e) {
                Log::error << "IK solver error: " << e.what() << std::endl;
                return false;
            }
        }
    };
 
int main(int argc, char **argv) {
    CLParser parser("iksrv", "Inverse Kinematics Service");
    parser.add_arg(CLParser::Arg("jrdf", "Path to JRDF file", '1'));
    parser.add_opt(CLParser::Opt("ip", "i", "RIX Hub IP Address", "127.0.0.1", "", '1'));
    parser.parse(argc, argv);

    std::string jrdf_file = parser.get_arg("jrdf").front();
    std::string hub_ip = parser.get_opt("ip").front();

    /**
     * TODO: Implement the inverse kinematics service.
     */
        
    // Initialize RIX node
    if (!Node::init("ik_service", hub_ip, RIX_HUB_PORT)) {
        Log::error << "Failed to initialize RIX node" << std::endl;
        return 1;
    }

    try {
        // Create and start the IK service
        IKService service_node(jrdf_file);
        service_node.run();
    } catch (const std::exception& e) {
        Log::error << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
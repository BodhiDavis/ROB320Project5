#include "tests.hpp"

#include "rix/core/node.hpp"
#include "rix/core/service_client.hpp"
#include "rix/msg/iksrv/IKRequest.hpp"
#include "rix/msg/iksrv/IKResponse.hpp"

using Node = rix::core::Node;
using ServiceClient = rix::core::ServiceClient;
using IKRequest = rix::msg::iksrv::IKRequest;
using IKResponse = rix::msg::iksrv::IKResponse;

bool test_solve(ServiceClient &client, double p1, double p2, double p3) {
    JS init;
    init.joint_states.resize(3);
    init.joint_states[0].name = "shoulder";
    init.joint_states[0].position = 0;
    init.joint_states[0].velocity = 0;
    init.joint_states[0].effort = 0;
    init.joint_states[1].name = "elbow";
    init.joint_states[1].position = 0;
    init.joint_states[1].velocity = 0;
    init.joint_states[1].effort = 0;
    init.joint_states[2].name = "wrist";
    init.joint_states[2].position = 0;
    init.joint_states[2].velocity = 0;
    init.joint_states[2].effort = 0;

    JS q = init;
    q.joint_states[0].position = p1;
    q.joint_states[1].position = p2;
    q.joint_states[2].position = p3;

    Tree tree(Json::parse(ROBOT));
    tree.set_state(q);
    FKSolver fk(tree);
    Eigen::Affine3d goal = fk.solve("tool");

    IKRequest ik_request;
    ik_request.end_effector = "tool";
    ik_request.goal = rix::rdf::eigen_to_msg(goal);
    ik_request.initial_guess = init;
    ik_request.step_scale = 0.5;
    ik_request.tolerance = 1e-4;
    ik_request.max_iterations = 1000;
    ik_request.debug = false;
    IKResponse ik_response;
    if (!client.call(ik_request, ik_response)) {
        Log::error << "Failed to call IK service" << std::endl;
        return false;
    }
    if (!ik_response.converged) {
        Log::error << "IK solver did not converge" << std::endl;
        return false;
    }

    tree.set_state(ik_response.result);
    Eigen::Affine3d end_effector_transform = fk.solve("tool");
    double error;
    Eigen::Vector3d goal_translation = goal.translation();
    Eigen::Vector3d end_effector_translation = end_effector_transform.translation();

#ifdef CHECK_ORIENTATION
    Eigen::Vector3d goal_orientation = goal.rotation().eulerAngles(0, 1, 2);
    Eigen::Vector3d end_effector_orientation = end_effector_transform.rotation().eulerAngles(0, 1, 2);

    Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
    dp.head<3>() = goal_translation - end_effector_translation;
    dp.tail<3>() = goal_orientation - end_effector_orientation;
    error = dp.norm();
#else
    error = (goal_translation - end_effector_translation).norm();
#endif

    if (error > TOL) {
        Log::error << "IK solver error greater than tolerance: " << error << " > " << TOL << std::endl;
        return false;
    }
    return true;
}

bool test_solve_with_trajectory(ServiceClient &client, double p1, double p2, double p3) {
    Tree tree(Json::parse(ROBOT));

    JS init;
    init.joint_states.resize(3);
    init.joint_states[0].name = "shoulder";
    init.joint_states[0].position = 0;
    init.joint_states[0].velocity = 0;
    init.joint_states[0].effort = 0;
    init.joint_states[1].name = "elbow";
    init.joint_states[1].position = 0;
    init.joint_states[1].velocity = 0;
    init.joint_states[1].effort = 0;
    init.joint_states[2].name = "wrist";
    init.joint_states[2].position = 0;
    init.joint_states[2].velocity = 0;
    init.joint_states[2].effort = 0;

    JS q = init;
    q.joint_states[0].position = p1;
    q.joint_states[1].position = p2;
    q.joint_states[2].position = p3;

    tree.set_state(q);
    FKSolver fk(tree);
    Eigen::Affine3d goal = fk.solve("tool");
    IKRequest ik_request;
    ik_request.end_effector = "tool";
    ik_request.goal = rix::rdf::eigen_to_msg(goal);
    ik_request.initial_guess = init;
    ik_request.step_scale = 0.5;
    ik_request.tolerance = 1e-4;
    ik_request.max_iterations = 1000;
    ik_request.debug = true;
    IKResponse ik_response;
    if (!client.call(ik_request, ik_response)) {
        Log::error << "Failed to call IK service" << std::endl;
        return false;
    }
    if (!ik_response.converged) {
        Log::error << "IK solver did not converge" << std::endl;
        return false;
    }

    tree.set_state(ik_response.result);
    Eigen::Affine3d end_effector_transform = fk.solve("tool");
    double error;
    Eigen::Vector3d goal_translation = goal.translation();
    Eigen::Vector3d end_effector_translation = end_effector_transform.translation();
#ifdef CHECK_ORIENTATION
    Eigen::Vector3d goal_orientation = goal.rotation().eulerAngles(0, 1, 2);
    Eigen::Vector3d end_effector_orientation = end_effector_transform.rotation().eulerAngles(0, 1, 2);

    Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
    dp.head<3>() = goal_translation - end_effector_translation;
    dp.tail<3>() = goal_orientation - end_effector_orientation;
    error = dp.norm();
#else
    error = (goal_translation - end_effector_translation).norm();
#endif

    if (error > TOL) {
        Log::error << "IK solver error greater than tolerance: " << error << " > " << TOL << std::endl;
        return false;
    }
    if (!validate_ikresponse_steps(ik_response.steps, fk, tree, "tool", goal, TOL)) {
        return false;
    }
    return true;
}

int main() {
    if (!Node::init("test_iksrv", "127.0.0.1", RIX_HUB_PORT)) {
        Log::error << "Failed to initialize node" << std::endl;
        return 1;
    }

    rix::core::ServiceClient client;
    client = Node::service_client<IKRequest, IKResponse>("ik");
    if (!client.ok()) {
        Log::error << "Failed to initialize service client" << std::endl;
        return 1;
    }

    // TEST 1
    if (!test_solve(client, M_PI / 2, M_PI / 2, 0)) {
        Log::error << "Test 1 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, M_PI / 2, M_PI / 2, 0)) {
        Log::error << "Test 1 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 2
    if (!test_solve(client, 0, M_PI / 2, 0)) {
        Log::error << "Test 2 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, 0, M_PI / 2, 0)) {
        Log::error << "Test 2 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 3
    if (!test_solve(client, M_PI / 2, 0, 0)) {
        Log::error << "Test 3 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, M_PI / 2, 0, 0)) {
        Log::error << "Test 3 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 4
    if (!test_solve(client, -M_PI / 2, M_PI / 2, 0)) {
        Log::error << "Test 4 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, -M_PI / 2, M_PI / 2, 0)) {
        Log::error << "Test 4 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 5
    if (!test_solve(client, M_PI / 2, -M_PI / 2, 0)) {
        Log::error << "Test 5 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, M_PI / 2, -M_PI / 2, 0)) {
        Log::error << "Test 5 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 6
    if (!test_solve(client, 0, -M_PI / 2, 0)) {
        Log::error << "Test 6 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, 0, -M_PI / 2, 0)) {
        Log::error << "Test 6 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 7
    if (!test_solve(client, -M_PI / 2, 0, 0)) {
        Log::error << "Test 7 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, -M_PI / 2, 0, 0)) {
        Log::error << "Test 7 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 8
    if (!test_solve(client, -M_PI / 2, -M_PI / 2, 0)) {
        Log::error << "Test 8 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, -M_PI / 2, -M_PI / 2, 0)) {
        Log::error << "Test 8 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 9
    if (!test_solve(client, M_PI / 2, M_PI / 2, 0.5)) {
        Log::error << "Test 9 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, M_PI / 2, M_PI / 2, 0.5)) {
        Log::error << "Test 9 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 10
    if (!test_solve(client, 0, M_PI / 2, 0.5)) {
        Log::error << "Test 10 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, 0, M_PI / 2, 0.5)) {
        Log::error << "Test 10 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 11
    if (!test_solve(client, M_PI / 2, 0, 0.5)) {
        Log::error << "Test 11 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, M_PI / 2, 0, 0.5)) {
        Log::error << "Test 11 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 12
    if (!test_solve(client, -M_PI / 2, M_PI / 2, 0.5)) {
        Log::error << "Test 12 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, -M_PI / 2, M_PI / 2, 0.5)) {
        Log::error << "Test 12 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 13
    if (!test_solve(client, M_PI / 2, -M_PI / 2, 0.5)) {
        Log::error << "Test 13 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, M_PI / 2, -M_PI / 2, 0.5)) {
        Log::error << "Test 13 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 14
    if (!test_solve(client, 0, -M_PI / 2, 0.5)) {
        Log::error << "Test 14 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, 0, -M_PI / 2, 0.5)) {
        Log::error << "Test 14 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 15
    if (!test_solve(client, -M_PI / 2, 0, 0.5)) {
        Log::error << "Test 15 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, -M_PI / 2, 0, 0.5)) {
        Log::error << "Test 15 solve with trajectory failed."  << std::endl;
        return 1;
    }

    // TEST 16
    if (!test_solve(client, -M_PI / 2, -M_PI / 2, 0.5)) {
        Log::error << "Test 16 solve failed."  << std::endl;
        return 1;
    }
    if (!test_solve_with_trajectory(client, -M_PI / 2, -M_PI / 2, 0.5)) {
        Log::error << "Test 16 solve with trajectory failed."  << std::endl;
        return 1;
    }

    std::cout << "all tests passed!\n";

    return 0;
}
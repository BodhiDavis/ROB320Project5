#include <gtest/gtest.h>

#include "tests.hpp"

void test_solve(double p1, double p2, double p3) {
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

    IKSolver ik(tree);
    bool converged = false;
    JS result = ik.solve("tool", goal, init, 0.5, 1e-4, 1000, converged);
    ASSERT_TRUE(converged);

    tree.set_state(result);
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

    EXPECT_LT(error, TOL);
}

void test_solve_with_trajectory(double p1, double p2, double p3) {
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

    IKSolver ik(tree);
    bool converged = false;
    std::vector<JS> result = ik.solve_with_trajectory("tool", goal, init, 0.5, 1e-4, 1000, converged);
    ASSERT_TRUE(converged);

    tree.set_state(result.back());
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

    EXPECT_LT(error, TOL);
    EXPECT_EQ(validate_ikresponse_steps(result, fk, tree, "tool", goal, TOL), true);
}

TEST(IKSolver, Solve1) { test_solve(M_PI / 2, M_PI / 2, 0); }

TEST(IKSolver, Solve2) { test_solve(M_PI / 2, 0, 0); }

TEST(IKSolver, Solve3) { test_solve(M_PI / 2, M_PI / 2, 0); }

TEST(IKSolver, Solve4) { test_solve(-M_PI / 2, M_PI / 2, 0); }

TEST(IKSolver, Solve5) { test_solve(M_PI / 2, -M_PI / 2, 0); }

TEST(IKSolver, Solve6) { test_solve(-M_PI / 2, 0, 0); }

TEST(IKSolver, Solve7) { test_solve(0, -M_PI / 2, 0); }

TEST(IKSolver, Solve8) { test_solve(-M_PI / 2, -M_PI / 2, 0); }

TEST(IKSolver, Solve9) { test_solve(M_PI / 2, M_PI / 2, 0.5); }

TEST(IKSolver, Solve10) { test_solve(M_PI / 2, 0, 0.5); }

TEST(IKSolver, Solve11) { test_solve(M_PI / 2, M_PI / 2, 0.5); }

TEST(IKSolver, Solve12) { test_solve(-M_PI / 2, M_PI / 2, 0.5); }

TEST(IKSolver, Solve13) { test_solve(M_PI / 2, -M_PI / 2, 0.5); }

TEST(IKSolver, Solve14) { test_solve(-M_PI / 2, 0, 0.5); }

TEST(IKSolver, Solve15) { test_solve(0, -M_PI / 2, 0.5); }

TEST(IKSolver, Solve16) { test_solve(-M_PI / 2, -M_PI / 2, 0.5); }

TEST(IKSolver, SolveWithTrajectory1) { test_solve_with_trajectory(0, M_PI / 2, 0); }

TEST(IKSolver, SolveWithTrajectory2) { test_solve_with_trajectory(M_PI / 2, 0, 0); }

TEST(IKSolver, SolveWithTrajectory3) { test_solve_with_trajectory(M_PI / 2, M_PI / 2, 0); }

TEST(IKSolver, SolveWithTrajectory4) { test_solve_with_trajectory(-M_PI / 2, M_PI / 2, 0); }

TEST(IKSolver, SolveWithTrajectory5) { test_solve_with_trajectory(M_PI / 2, -M_PI / 2, 0); }

TEST(IKSolver, SolveWithTrajectory6) { test_solve_with_trajectory(-M_PI / 2, 0, 0); }

TEST(IKSolver, SolveWithTrajectory7) { test_solve_with_trajectory(0, -M_PI / 2, 0); }

TEST(IKSolver, SolveWithTrajectory8) { test_solve_with_trajectory(-M_PI / 2, -M_PI / 2, 0); }

TEST(IKSolver, SolveWithTrajectory9) { test_solve_with_trajectory(0, M_PI / 2, 0.5); }

TEST(IKSolver, SolveWithTrajectory10) { test_solve_with_trajectory(M_PI / 2, 0, 0.5); }

TEST(IKSolver, SolveWithTrajectory11) { test_solve_with_trajectory(M_PI / 2, M_PI / 2, 0.5); }

TEST(IKSolver, SolveWithTrajectory12) { test_solve_with_trajectory(-M_PI / 2, M_PI / 2, 0.5); }

TEST(IKSolver, SolveWithTrajectory13) { test_solve_with_trajectory(M_PI / 2, -M_PI / 2, 0.5); }

TEST(IKSolver, SolveWithTrajectory14) { test_solve_with_trajectory(-M_PI / 2, 0, 0.5); }

TEST(IKSolver, SolveWithTrajectory15) { test_solve_with_trajectory(0, -M_PI / 2, 0.5); }

TEST(IKSolver, SolveWithTrajectory16) { test_solve_with_trajectory(-M_PI / 2, -M_PI / 2, 0.5); }
#pragma once

#include <Eigen/Geometry>

#include "ik_solver.hpp"
#include "rix/util/log.hpp"
#include "nlohmann/json.hpp"
#include "rix/rdf/fk_solver.hpp"
#include "rix/rdf/tree.hpp"

using namespace rix::rdf;
using namespace nlohmann;

using JS = rix::msg::sensor::JS;
using FKSolver = rix::rdf::FKSolver;
using Tree = rix::rdf::Tree;
using Log = rix::util::Log;

#define TOL 1e-2

// #define CHECK_ORIENTATION 1

const char *ROBOT = R"(
{
  "name": "RRP",
  "links": [
    {
      "name": "base",
      "visual": {
        "geometry": {
          "type": "box",
          "size": [0.5, 0.5, 1.0]
        },
        "origin": {
          "xyz": [0, 0, 0.5],
          "rpy": [0, 0, 0]
        }
      }
    },
    {
      "name": "upper_arm",
      "visual": {
        "geometry": {
          "type": "box",
          "size": [1.0, 0.25, 0.25]
        },
        "origin": {
          "xyz": [0.5, 0, 0],
          "rpy": [0, 0, 0]
        }
      }
    },
    {
      "name": "lower_arm",
      "visual": {
        "geometry": {
          "type": "box",
          "size": [1.0, 0.25, 0.25]
        },
        "origin": {
          "xyz": [0.5, 0, 0],
          "rpy": [0, 0, 0]
        }
      }
    },
    {
      "name": "tool",
      "visual": {
        "geometry": {
          "type": "box",
          "size": [0.1, 0.1, 0.25]
        },
        "origin": {
          "xyz": [0, 0, 0.125],
          "rpy": [0, 0, 0]
        }
      }
    }
  ],
  "joints": [
    {
      "name": "shoulder",
      "type": "continuous",
      "parent": "base",
      "child": "upper_arm",
      "origin": {
        "xyz": [0, 0, 0.5],
        "rpy": [0, 0, 0]
      },
      "axis": [0, 0, 1]
    },
    {
      "name": "elbow",
      "type": "continuous",
      "parent": "upper_arm",
      "child": "lower_arm",
      "origin": {
        "xyz": [0.5, 0, 0],
        "rpy": [0, 0, 0]
      },
      "axis": [0, 0, 1]
    },
    {
      "name": "wrist",
      "type": "prismatic",
      "parent": "lower_arm",
      "child": "tool",
      "origin": {
        "xyz": [0.5, 0, 0],
        "rpy": [0, 0, 0]
      },
      "axis": [0, 0, -1],
      "limits": {
        "lower": 0,
        "upper": 1
      }
    }
  ]
}
)";

bool validate_ikresponse_steps(const std::vector<JS> &steps, const FKSolver &fk, Tree &tree, const std::string &end_effector, const Eigen::Affine3d &goal, double tol) {
    if (steps.size() < 2) {
        return false;
    }
    for (size_t i = 0; i < steps.size() - 1; i++) {
        if (steps[i].joint_states.size() != tree.get_num_joints()) {
            return false;
        }
        tree.set_state(steps[i]);
        Eigen::Affine3d Ta = fk.solve(end_effector);
        tree.set_state(steps[i + 1]);
        Eigen::Affine3d Tb = fk.solve(end_effector);

        double error_a, error_b;
#ifdef CHECK_ORIENTATION
        Eigen::Vector3d orientation_a = Ta.rotation().eulerAngles(0, 1, 2);
        Eigen::Vector3d translation_a = Ta.translation();
        Eigen::VectorXd dp_a = Eigen::VectorXd::Zero(6);
        dp_a.head<3>() = translation_a - goal.translation();
        dp_a.tail<3>() = orientation_a - goal.rotation().eulerAngles(0, 1, 2);
        error_a = dp_a.norm();

        Eigen::Vector3d orientation_b = Tb.rotation().eulerAngles(0, 1, 2);
        Eigen::Vector3d translation_b = Tb.translation();
        Eigen::VectorXd dp_b = Eigen::VectorXd::Zero(6);
        dp_b.head<3>() = translation_b - goal.translation();
        dp_b.tail<3>() = orientation_b - goal.rotation().eulerAngles(0, 1, 2);
        error_b = dp_b.norm();
#else
        Eigen::Vector3d translation_a = Ta.translation();
        Eigen::Vector3d translation_b = Tb.translation();
        error_a = (translation_a - goal.translation()).norm();
        error_b = (translation_b - goal.translation()).norm();
#endif
        if (error_b > error_a) {
            Log::error << "IK response is not monotonic" << std::endl;
            Log::error << "Step " << i << ": error_a = " << error_a << ", error_b = " << error_b << std::endl;
            return false;
        }
    }
    return true;
}
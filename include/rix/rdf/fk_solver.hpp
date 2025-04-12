#pragma once

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <stack>
#include <string>
#include <vector>

#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"
#include "rix/rdf/joint.hpp"
#include "rix/rdf/link.hpp"
#include "rix/rdf/tree.hpp"
#include "rix/rdf/util.hpp"
#include "rix/util/timing.hpp"

namespace rix {
namespace rdf {

class FKSolver {
   public:
    FKSolver(const Tree &tree);
    Eigen::Affine3d solve(const std::string &link_name) const;
    Eigen::Affine3d solve(const std::string &link_name, const std::string &reference_link) const;
    static Eigen::Affine3d solve(const rix::msg::geometry::TF &tf, const std::string &link_name);
    static Eigen::Affine3d solve(const rix::msg::geometry::TF &tf, const std::string &link_name,
                                 const std::string &reference_link);
    rix::msg::geometry::TF global_tf() const;
    static rix::msg::geometry::TF global_tf(const rix::msg::geometry::TF &tf);

   private:
    const Tree &tree;
};

}  // namespace rdf
}  // namespace rix
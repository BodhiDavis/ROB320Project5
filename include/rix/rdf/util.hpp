#pragma once

#include <Eigen/Geometry>
#include "rix/msg/geometry/TF.hpp"

namespace rix {
namespace rdf {

rix::msg::geometry::Transform eigen_to_msg(const Eigen::Affine3d &T);
Eigen::Affine3d msg_to_eigen(const rix::msg::geometry::Transform &t);

}
}
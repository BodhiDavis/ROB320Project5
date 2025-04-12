#pragma once

#include <Eigen/Geometry>
#include <iostream>

namespace rix {
namespace rdf {

enum JointType { FIXED, CONTINUOUS, REVOLUTE, PRISMATIC };

class Joint {
   public:
    Joint(const Eigen::Vector3d &axis = Eigen::Vector3d::Zero(),
          const Eigen::Affine3d &origin = Eigen::Affine3d::Identity(), const JointType &type = FIXED,
          const std::string &parent = "", const std::string &child = "", double upper_bound = 0.0,
          double lower_bound = 0.0);
    Joint(const Joint &j);
    Joint &operator=(Joint j);

    bool is_in_bounds(double q) const;
    double clamp(double q) const;

    std::string get_parent() const;
    std::string get_child() const;
    JointType get_type() const;
    double get_lower_bound() const;
    double get_upper_bound() const;
    const Eigen::Vector3d &get_axis() const;
    const Eigen::Affine3d &get_origin() const;
    double get_state() const;
    Eigen::Affine3d get_transform() const;

    void set_state(double q);

   private:
    double q;
    double lower_bound;
    double upper_bound;
    std::string parent;
    std::string child;
    Eigen::Vector3d axis;
    Eigen::Affine3d origin;
    JointType type;
};

}  // namespace rdf
}  // namespace rix
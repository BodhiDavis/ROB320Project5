#pragma once

#include <Eigen/Geometry>

namespace rix {
namespace rdf {

class Link {
   public:
    Link(const Eigen::Affine3d &visual = Eigen::Affine3d::Identity(), const std::string &parent = "",
         const std::vector<std::string> &children = std::vector<std::string>());
    Link(const Link &other);
    Link &operator=(Link other);

    std::string get_parent() const;
    std::vector<std::string> get_children() const;
    bool is_root() const;
    bool is_end_effector() const;

    const Eigen::Affine3d &get_origin() const;

   private:
    Eigen::Affine3d origin;
    std::string parent;
    std::vector<std::string> children;
};

}  // namespace rdf
}  // namespace rix
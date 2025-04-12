#pragma once

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "rix/util/timing.hpp"
#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"
#include "rix/msg/sensor/JS.hpp"
#include "rix/rdf/joint.hpp"
#include "rix/rdf/link.hpp"
#include "rix/rdf/util.hpp"

namespace rix {
namespace rdf {

using Json = nlohmann::json;

class Tree {
   public:
    Tree(const Json &json);

    size_t get_num_joints() const;
    size_t get_num_links() const;

    std::vector<std::string> get_joint_names() const;
    std::vector<std::string> get_link_names() const;

    bool has_joint(const std::string &name) const;
    bool has_link(const std::string &name) const;
    Joint &get_joint(const std::string &name);
    Link &get_link(const std::string &name);
    const Joint &get_joint(const std::string &name) const;
    const Link &get_link(const std::string &name) const;

    std::string get_root() const;
    std::vector<std::string> get_end_effectors() const;

    rix::msg::geometry::TF TF();
    rix::msg::sensor::JS JS();
    void set_state(const rix::msg::sensor::JS &js);

   private:
    std::string root;
    std::unordered_map<std::string, Joint> joints;
    std::unordered_map<std::string, Link> links;
};

}  // namespace rdf
}  // namespace rix
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

/**
 * TODO: Declare any additional aliases or using directives here.
 */

/**
 * TODO: Declare any necessary classes or functions here.
 */

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
    return 0;
}
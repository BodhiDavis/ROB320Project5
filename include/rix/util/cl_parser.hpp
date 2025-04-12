#pragma once

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <unordered_map>
#include <set>
#include <vector>
#include <string>

namespace rix {
namespace util {

/**
 * @brief A command-line parser class.
 */
class CLParser {
   public:
    /**
     * @brief A struct representing an option.
     */
    struct Opt {
        std::string name;         ///< long name (--<name>)
        std::string short_name;   ///< short name (single character) (-<short_name>)
        std::string description;  ///< description of the option
        std::string default_val;  ///< value if not specified
        std::string const_val;    ///< value if specified without argument (flag)
        char nargs;               ///< number of arguments (0, 1, ?: 0 or 1, *: 0 or more, +: 1 or more)

        Opt() = default;
        Opt(std::string name, std::string short_name, std::string description, std::string default_val,
            std::string const_val, char nargs);
        bool operator<(const Opt &other) const;
    };

    /**
     * @brief A struct representing an argument.
     */
    struct Arg {
        std::string name;         ///< name of the argument
        std::string description;  ///< description of the argument
        char nargs;               ///< number of arguments (1, +: 1 or more)

        Arg() = default;
        Arg(std::string name, std::string description, char nargs);
        bool operator<(const Arg &other) const;
    };

    /**
     * @brief Constructs a CLParser object.
     * @param name The name of the program.
     * @param description The description of the program.
     */
    CLParser(const std::string &name, const std::string &description);

    /**
     * @brief Adds an option to the parser.
     * @param option The option to add.
     */
    void add_opt(const Opt &option);

    /**
     * @brief Adds an argument to the parser.
     * @param argument The argument to add.
     */
    void add_arg(const Arg &argument);

    /**
     * @brief Parses the command-line arguments.
     * @param argc The number of arguments.
     * @param argv The array of arguments.
     */
    void parse(int argc, char **argv);

    /**
     * @brief Gets the values of an option.
     * @param name The name of the option.
     * @return A vector of values for the option.
     */
    std::vector<std::string> get_opt(const std::string &name);

    /**
     * @brief Gets the values of an argument.
     * @param name The name of the argument.
     * @return A vector of values for the argument.
     */
    std::vector<std::string> get_arg(const std::string &name);

    /**
     * @brief Gets the help message.
     * @return The help message as a string.
     */
    std::string get_help();

    /**
     * @brief Gets the usage message.
     * @return The usage message as a string.
     */
    std::string get_usage();

   private:
    std::string name;  ///< The name of the program.
    std::string description;  ///< The description of the program.

    std::unordered_map<std::string, std::string> short_to_long;  ///< short name to long name mapping
    std::unordered_map<std::string, std::pair<Opt, std::vector<std::string>>> options;  ///< options map

    std::vector<std::string> arg_names;  ///< argument names
    std::unordered_map<std::string, std::pair<Arg, std::vector<std::string>>> arguments;  ///< arguments map
};

}  // namespace util
}  // namespace rix
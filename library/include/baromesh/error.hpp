#ifndef BAROBO_ERROR_HPP
#define BAROBO_ERROR_HPP

#include <stdexcept>

namespace barobo {

struct Error : std::runtime_error {
    explicit Error (std::string s) : std::runtime_error(s) {}
};

}

#endif
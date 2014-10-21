#ifndef BAROMESH_ERROR_HPP
#define BAROMESH_ERROR_HPP

#include <stdexcept>

namespace barobo {

class Error : public std::runtime_error {
public:
	explicit Error (std::string what) : std::runtime_error(what) {}
};

} // namespace barobo

#endif
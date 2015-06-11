#ifndef BAROMESH_LOG_HPP
#define BAROMESH_LOG_HPP

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include <boost/optional.hpp>

#include <string>

namespace baromesh {
namespace log {

boost::program_options::options_description optionsDescription (boost::optional<std::string> logFileName);
void initialize (std::string appName, const boost::program_options::variables_map& conf);

} // namespace log
} // namespace baromesh

#endif
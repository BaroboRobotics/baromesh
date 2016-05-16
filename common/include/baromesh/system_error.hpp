#ifndef BAROMESH_SYSTEM_ERROR_HPP
#define BAROMESH_SYSTEM_ERROR_HPP

#include <baromesh/status.hpp>

#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <string>

namespace baromesh {

class ErrorCategory : public boost::system::error_category {
public:
    virtual const char* name () const BOOST_NOEXCEPT override;
    virtual std::string message (int ev) const BOOST_NOEXCEPT override;
};

const boost::system::error_category& errorCategory ();
boost::system::error_code make_error_code (Status status);
boost::system::error_condition make_error_condition (Status status);

} // namespace baromesh

namespace boost {
namespace system {

template <>
struct is_error_code_enum< ::baromesh::Status> : public std::true_type { };

} // namespace system
} // namespace boost

#endif

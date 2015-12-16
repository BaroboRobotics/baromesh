#ifndef USBCDC_DEVICES_HPP
#define USBCDC_DEVICES_HPP

#include <boost/range/any_range.hpp>

#include <string>

namespace usbcdc {

class Device {
public:
    Device () = default;
    Device (const std::string& path, const std::string& productString)
        : mPath(path), mProductString(productString)
    {}
    const std::string& path () const { return mPath; }
    const std::string& productString () const { return mProductString; }
private:
    std::string mPath;
    std::string mProductString;
};

// The third parameter to any_range (Reference) must be const to work around a
// Boost bug: https://svn.boost.org/trac/boost/ticket/10493
// Non-const makes it segfault.
using DeviceRange = boost::any_range<Device, boost::single_pass_traversal_tag, const Device, ptrdiff_t>;

DeviceRange devices ();

} // namespace usbcdc

#endif

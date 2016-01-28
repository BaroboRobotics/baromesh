#include <baromesh/dongledevicepath.hpp>
#include <baromesh/system_error.hpp>

#include <usbcdc/devices.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <set>
#include <string>

// List of valid Barobo dongle product strings.
// Update this list as necessary.
static const std::set<std::string>& usbDongleProductStrings () {
    static const std::set<std::string> s {
        "Mobot USB-Serial Adapter",
        "Linkbot USB-Serial Adapter",
        "Barobo USB-Serial Adapter"
    };
    return s;
}

static bool isBaroboDongle (const usbcdc::Device& d) {
#ifdef __MACH__
    for (auto& expectedProduct : usbDongleProductStrings()) {
        // TODO: test if productString is truly null-terminated properly on
        // OS X 10.10. Until this is tested, we'll keep the old method of
        // comparing product strings: true if the expected product string
        // is a prefix of the device's product string.
        if (expectedProduct.size() <= d.productString().size()
            && expectedProduct.end() == std::mismatch(
                expectedProduct.begin(),
                expectedProduct.end(),
                d.productString().begin()).first) {
            return true;
        }
    }
    return false;
#else
    return !!usbDongleProductStrings().count(d.productString());
#endif
}


namespace baromesh {

std::string dongleDevicePath () {
    boost::system::error_code ec;
    auto path = dongleDevicePath(ec);
    if (ec) {
        throw boost::system::system_error(ec);
    }
    return path;
}

std::string dongleDevicePath (boost::system::error_code& ec) {
    boost::log::sources::logger lg;
    ec = baromesh::Status::DONGLE_NOT_FOUND;
    try {
        for (auto d : usbcdc::devices()) {
            if (isBaroboDongle(d)) {
                ec = baromesh::Status::OK;
                return d.path();
            }
        }
    }
    catch (boost::system::system_error& e) {
        ec = e.code();
    }
    catch (std::exception& e) {
        BOOST_LOG(lg) << "Exception getting dongle device path: " << e.what();
    }
    return {};
}

} // namespace baromesh

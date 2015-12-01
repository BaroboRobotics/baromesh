#include "baromesh/dongledevicepath.hpp"

#include "baromesh/system_error.hpp"

struct usb_dongle_id {
  const char *product;
};

/* List of valid Barobo dongle product strings. The platform-specific
 * dongleDevicePath() functions should depend on this data for finding the
 * dongle. Update this list as necessary. */
static const usb_dongle_id g_barobo_usb_dongle_ids[] = {
    { "Mobot USB-Serial Adapter" },
    { "Linkbot USB-Serial Adapter" },
    { "Barobo USB-Serial Adapter" },
};

static int dongleDevicePathImpl(char *, size_t);

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
    // Get the dongle device path, i.e.: /dev/ttyACM0, \\.\COM3, etc.
    char path[64];
    auto status = dongleDevicePathImpl(path, sizeof(path));
    if (-1 == status) {
      ec = Status::DONGLE_NOT_FOUND;
      return {};
    }
    ec = Status::OK;
    return std::string(path);
}

} // namespace baromesh


/* For convenience. */
#define NUM_BAROBO_USB_DONGLE_IDS \
  (sizeof(g_barobo_usb_dongle_ids) / sizeof(g_barobo_usb_dongle_ids[0]))

#ifdef _WIN32
#include "dongledevicepath/win32.cpp"
#elif defined(__linux__)
#include "dongledevicepath/linux.cpp"
#elif defined(__APPLE__) && defined(__MACH__)
#include "dongledevicepath/osx.cpp"
#else
#error No dongledevicepath.cpp available for this platform.
#endif

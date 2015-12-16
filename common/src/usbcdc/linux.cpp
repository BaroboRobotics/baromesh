#include "devices.hpp"

#include <boost/algorithm/string/predicate.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/iterator_range.hpp>

#include <boost/system/error_code.hpp>

#include <exception>
#include <iostream>
#include <string>

#include <cstdlib>

#include <unistd.h>

namespace fs = boost::filesystem;
using namespace boost::adaptors;

namespace usbcdc {

static fs::path sysDevices () {
    auto sysEnv = std::getenv("SYSFS_PATH");
    auto sd = fs::path{sysEnv ? sysEnv : "/sys"} / "devices";
    if (!fs::exists(sd) || !fs::is_directory(sd)) {
        throw std::runtime_error("No sysfs");
    }
    return sd;
}

static boost::iterator_range<fs::recursive_directory_iterator>
traverseDir (const fs::path& root) {
    return boost::make_iterator_range(
        fs::recursive_directory_iterator{root},
        fs::recursive_directory_iterator{});
};

// For use with Boost.Range's filtered adaptor
struct BySubsystem {
    const std::string mTarget;
    BySubsystem () = default;
    explicit BySubsystem (const std::string& target) : mTarget(target) {}
    // True if p has a child subsystem symlink matching target.
    bool operator() (const fs::path& p) const {
        if (fs::is_directory(p) && !fs::is_symlink(p)) {
            auto ec = boost::system::error_code{};
            auto subsystem = fs::read_symlink(p / "subsystem", ec);
            return !ec && mTarget == subsystem.filename();
        }
        return false;
    };
};

// http://www.usb.org/developers/defined_class
enum class UsbClass : uint8_t {
    cdc = 0x02
};

struct ByUsbInterfaceClass {
    const UsbClass mTarget;
    ByUsbInterfaceClass () = default;
    explicit ByUsbInterfaceClass (UsbClass target) : mTarget(target) {}
    bool operator() (const fs::path& p) const {
        if (fs::is_directory(p) && !fs::is_symlink(p)) {
            auto icPath = p / "bInterfaceClass";
            if (fs::exists(icPath)) {
                std::string icString;
                fs::ifstream icStream{icPath};
                if (std::getline(icStream, icString)) {
                    size_t p = 0;
                    auto ic = UsbClass(std::stoul(icString, &p, 16));
                    return p && mTarget == ic;
                }
            }
        }
        return false;
    }
};

// For use with Boost.Range transformed adaptor
static Device toDevice (const fs::path& p) {
    auto productPath = p.parent_path() / "product";
    if (fs::exists(productPath)) {
        std::string productString;
        fs::ifstream productStream{productPath};
        if (std::getline(productStream, productString)) {
            for (const auto& tty : traverseDir(p / "tty")
                                   | filtered(BySubsystem{"tty"})) {
                auto uePath = tty / "uevent";
                if (fs::exists(uePath)) {
                    std::string path;
                    fs::ifstream ueStream{uePath};
                    const auto key = std::string("DEVNAME=");
                    while (std::getline(ueStream, path)) {
                        if (boost::algorithm::starts_with(path, key)) {
                            path.replace(0, key.length(), "/dev/");
                            return Device{path, productString};
                        }
                    }
                }
            }
        }
    }
    return Device{};
}

static bool deviceIsValid (const Device& d) {
    return d.path().size() && d.productString().size();
}

DeviceRange devices () {
    return traverseDir(sysDevices())
        | filtered(BySubsystem{"usb"})
        | filtered(ByUsbInterfaceClass{UsbClass::cdc})
        | transformed(toDevice)
        | filtered(deviceIsValid)
        ;
}

} // namespace usbcdc

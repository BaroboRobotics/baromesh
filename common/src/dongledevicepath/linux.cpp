#include "baromesh/system_error.hpp"

#include <boost/algorithm/string/predicate.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range.hpp>

#include <boost/system/error_code.hpp>

#include <exception>
#include <iostream>
#include <string>

#include <cstdlib>

#include <unistd.h>

namespace fs = boost::filesystem;
using namespace boost::adaptors;

class Device {
public:
    Device () = default;
    Device (std::string path, std::string productString, fs::path sysfsUsbPath, fs::path sysfsTtyPath)
        : mPath(path), mProductString(productString), mSysfsUsbPath(sysfsUsbPath), mSysfsTtyPath(sysfsTtyPath)
    {}

    const std::string& path () const {
        return mPath;
    }

    const std::string& productString () const {
        return mProductString;
    }

    const fs::path& sysfsUsbPath () const { return mSysfsUsbPath; }
    const fs::path& sysfsTtyPath () const { return mSysfsTtyPath; }

private:
    std::string mPath;
    std::string mProductString;
    fs::path mSysfsUsbPath;
    fs::path mSysfsTtyPath;
};

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
    std::string mTarget;
    BySubsystem () = default;
    explicit BySubsystem (const std::string& target) : mTarget(target) {}
    // True if p has a child subsystem symlink matching target.
    bool operator() (const fs::path& p) const {
        auto ec = boost::system::error_code{};
        auto subsystem = fs::read_symlink(p / "subsystem", ec);
        return !ec && subsystem.filename() == mTarget;
    };
};

// For use with Boost.Range transformed adaptor
static Device toDevice (const fs::path& p) {
    auto productPath = p / "product";
    if (fs::exists(productPath)) {
        std::string productString;
        fs::ifstream productStream{productPath};
        std::getline(productStream, productString);
        for (const auto& tty : traverseDir(p)
                               | filtered(BySubsystem{"tty"})) {
            auto uePath = tty / "uevent";
            if (fs::exists(uePath)) {
                std::string path;
                fs::ifstream ueStream{uePath};
                const auto key = std::string("DEVNAME=");
                while (std::getline(ueStream, path)) {
                    if (boost::algorithm::starts_with(path, key)) {
                        path.replace(0, key.length(), "/dev/");
                        return Device{path, productString, p, tty};
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

// TODO get rid of this when we switch to C++14 and use "auto devices () { }"
// instead.
using DeviceRange
    = boost::filtered_range<
        decltype(&deviceIsValid),
        const boost::transformed_range<
            decltype(&toDevice),
            const boost::filtered_range<
                BySubsystem,
                const decltype(traverseDir(sysDevices()))
            >
        >
    >;

static DeviceRange devices () {
    return traverseDir(sysDevices())
        | filtered(BySubsystem{"usb"})
        | transformed(toDevice)
        | filtered(deviceIsValid)
        ;
}

std::string dongleDevicePathImpl (boost::system::error_code& ec) {
    boost::log::sources::logger lg;
    ec = baromesh::Status::DONGLE_NOT_FOUND;
    try {
        for (auto d : devices()) {
            BOOST_LOG(lg) << "Detected " << d.productString() << " at " << d.path()
                          << "(" << d.sysfsUsbPath() << " ; " << d.sysfsTtyPath() << ")";
        }
        for (auto device : devices()) {
            if (usbDongleProductStrings().count(device.productString())) {
                ec = baromesh::Status::OK;
                return device.path();
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

#include <boost/algorithm/string/predicate.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <boost/iterator/filter_iterator.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/range/iterator_range.hpp>

#include <boost/system/error_code.hpp>

#include <exception>
#include <iostream>
#include <string>

#include <cstdlib>

#include <unistd.h>

namespace fs = boost::filesystem;

bool findTtyPath (std::string& output,
        std::string expectedManufacturer,
        std::string expectedProduct) {
    using namespace std::placeholders;

    std::string sysfs{"/sys"};
    auto sysfsPtr = std::getenv("SYSFS_PATH");
    if (sysfsPtr) {
        sysfs = sysfsPtr;
    }

    auto sysDevices = fs::path{sysfs + "/devices"};

    if (fs::exists(sysDevices) && fs::is_directory(sysDevices)) {
        // True if a path is a subsystem symlink matching ss.
        auto isSubsystemSymlink = [] (const fs::path& ss, const fs::path& p) {
            return p.filename() == "subsystem" &&
                   fs::read_symlink(p).filename() == ss;
        };

        // Make an iterator which filters directory entries by isSubsystemSymlink.
        auto makeSubsystemFilter = [=] (fs::path ss, fs::recursive_directory_iterator iter) {
            return boost::make_filter_iterator(
                std::bind(isSubsystemSymlink, ss, _1),
                iter,
                fs::recursive_directory_iterator{});
        };

        // Make an object we can for (x : y) over.
        auto subsystems = [=] (const fs::path& ss, const fs::path& root) {
            return boost::make_iterator_range(
                makeSubsystemFilter(ss, fs::recursive_directory_iterator{root}),
                makeSubsystemFilter(ss, fs::recursive_directory_iterator{}));
        };

        // For each USB device on the system ...
        for (fs::path usbSs : subsystems("usb", sysDevices)) {
            auto manufacturerPath = usbSs.parent_path() / "manufacturer";
            auto productPath = usbSs.parent_path() / "product";

            // that has manufacturer and product entries ...
            if (fs::exists(manufacturerPath) && fs::exists(productPath)) {
                std::string manufacturer;
                fs::ifstream manufacturerStream{manufacturerPath};
                std::getline(manufacturerStream, manufacturer);

                std::string product;
                fs::ifstream productStream{productPath};
                std::getline(productStream, product);

                // which matches the dongle's expected strings ...
                if (manufacturer == expectedManufacturer && product == expectedProduct) {
                    // find its associated TTY device path.
                    for (fs::path ttySs : subsystems("tty", usbSs.parent_path())) {
                        auto uePath = ttySs.parent_path() / "uevent";
                        if (fs::exists(uePath)) {
                            fs::ifstream ueStream{uePath};
                            while (std::getline(ueStream, output)) {
                                auto key = std::string{"DEVNAME="};
                                if (boost::algorithm::starts_with(output, key)) {
                                    output.replace(0, key.length(), "/dev/");
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

int dongleDevicePathImpl (char *buf, size_t len) {
    boost::log::sources::logger log;
    std::string path;
    for (auto i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
        if (findTtyPath(path,
                g_barobo_usb_dongle_ids[i].manufacturer,
                g_barobo_usb_dongle_ids[i].product)) {
            if (!access(path.c_str(), R_OK | W_OK)) {
                auto nWritten = snprintf(buf, len, "%s", path.c_str());
                if (nWritten >= len) {
                    BOOST_LOG(log) << "Buffer overflow copying device path: "
                                   << buf << " (... and "
                                   << path.length() - len + 1
                                   << " more characters)";
                }
                else {
                    return 0;
                }
            }
            else {
                auto ec = boost::system::error_code{errno, boost::system::generic_category()};
                BOOST_LOG(log) << "Error accessing dongle at " << path << ": " << ec.message();
            }
        }
    }
    return -1;
}

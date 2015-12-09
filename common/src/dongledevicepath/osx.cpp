#include "osx_uniqueioobject.hpp"

#include "baromesh/system_error.hpp"

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>

#include "CF++.h"

#include <IOKit/IOKitLib.h>
#include <IOKit/IOBSD.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOReturn.h>

#include <sys/sysctl.h>

#include <algorithm>

#include <string.h>
#include <unistd.h>

static unsigned darwinVersionMajor () {
    static auto v = [] {
        char osRelease[256];
        size_t osReleaseSize = sizeof(osRelease);
        auto rc = sysctlbyname("kern.osrelease", osRelease, &osReleaseSize, nullptr, 0);
        if (rc) {
            throw std::runtime_error("Error getting Darwin version string with sysctlbyname");
        }
        assert(osReleaseSize <= sizeof(osRelease));
        auto darwinVersionString = std::string(osRelease, osReleaseSize);

        std::vector<decltype(darwinVersionString)> splitResult;
        using CharT = decltype(darwinVersionString)::value_type;
        boost::split(splitResult, darwinVersionString, [] (CharT c) { return c == '.'; });
        if (splitResult.size() < 3) {
            throw std::runtime_error("Error parsing Darwin version string");
        }
        return boost::lexical_cast<unsigned>(splitResult[0]);
    }();
    return v;
}

static CF::String getStringProperty (io_object_t device, CF::String key, bool recursive=false) {
    auto valueRef = IORegistryEntrySearchCFProperty(device,
        kIOServicePlane, key, 
        kCFAllocatorDefault, recursive
                             ? kIORegistryIterateRecursively
                             : kNilOptions);
    auto value = CF::String{valueRef};
    if (valueRef) {
        CFRelease(valueRef);
    }
    return value;
}

static UniqueIoObject getUsbDeviceIterator () {
    // OS X 10.11 (Darwin 15) overhauled the USB system, introducing the
    // IOUSBHostDevice class name.
    auto classes = IOServiceMatching(darwinVersionMajor() < 15
                                     ? kIOUSBDeviceClassName
                                     : "IOUSBHostDevice");
    io_iterator_t iter;
    auto kr = IOServiceGetMatchingServices(kIOMasterPortDefault, classes, &iter);
    if (kIOReturnSuccess != kr) {
        throw std::runtime_error("Could not get USB devices from the IORegistry.");
    }
    return UniqueIoObject{iter};
}

class Device {
public:
    Device (std::string path, std::string productString)
        : mPath(path), mProductString(productString)
    {}

    std::string path () {
        return mPath;
    }

    std::string productString () {
        return mProductString;
    }

private:
    std::string mPath;
    std::string mProductString;
};

class DeviceIterator
    : public boost::iterator_facade<
        DeviceIterator, Device, boost::single_pass_traversal_tag> {
public:
    DeviceIterator ()
        : mIter(0)
    {}

    friend DeviceIterator begin (const DeviceIterator&) {
        return DeviceIterator{getUsbDeviceIterator()};
    }

    friend DeviceIterator end (const DeviceIterator&) {
        return DeviceIterator{};
    }

private:
    DeviceIterator (UniqueIoObject iter)
        : mIter(iter)
    {
        increment();
    }

    friend class boost::iterator_core_access;

    void increment () {
        std::string path;
        std::string productString;
        while (auto device = UniqueIoObject{IOIteratorNext(mIter)}) {
            // The device also has a "USB Product Name" property which we ought to
            // be able to use, but on 10.11, OS X mangles '-' to '_', and on 10.10
            // and earlier, the string returned is not null-terminated. The USB
            // product name is available unmangled as the device's registry entry
            // name instead.
            io_name_t name;
            auto kr = IORegistryEntryGetNameInPlane(device, kIOServicePlane, name);
            if (kIOReturnSuccess != kr) {
                continue;
            }
            productString = std::string(name);

            path = std::string(getStringProperty(device, "IOCalloutDevice", true));
            if (!path.length()) {
                continue;
            }
            mDevice = {path, productString};
            return;
        }
    }

    bool equal (const DeviceIterator& other) const {
        // IOKit iterators are only equal if they're invalid.
        return !IOIteratorIsValid(mIter) && !IOIteratorIsValid(other.mIter);
    }

    Device& dereference () const {
        return const_cast<Device&>(mDevice);
    }

    UniqueIoObject mIter;
    Device mDevice;
};


std::string dongleDevicePathImpl (boost::system::error_code& ec) {
    boost::log::sources::logger lg;
    ec = baromesh::Status::DONGLE_NOT_FOUND;
    try {
        for (auto& device : DeviceIterator{}) {
            auto productValue = device.productString();
            for (auto& expectedProduct : usbDongleProductStrings()) {
                // TODO: test if productValue is truly null-terminated properly on
                // OS X 10.10. Until this is tested, we'll keep the old method of
                // comparing product strings: true if the expected product string
                // is a prefix of the device's product string.
                if (expectedProduct.size() <= productValue.size()
                    && expectedProduct.end() == std::mismatch(
                        expectedProduct.begin(),
                        expectedProduct.end(),
                        productValue.begin()).first) {
                    return device.path();
                }
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

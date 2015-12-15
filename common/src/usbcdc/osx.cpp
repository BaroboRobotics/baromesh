#include "devices.hpp"

#include "osx_uniqueioobject.hpp"

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

namespace usbcdc {

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
    DeviceIterator (UniqueIoObject&& iter)
        : mIter(std::move(iter))
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
        mIter = UniqueIoObject{0};
    }

    bool equal (const DeviceIterator& other) const {
        // Only equal if they're both end iterators.
        return !mIter && !other.mIter;
    }

    Device& dereference () const {
        return const_cast<Device&>(mDevice);
    }

    UniqueIoObject mIter;
    Device mDevice;
};

DeviceRange devices () {
    return DeviceIterator{};
}

} // namespace usbcdc
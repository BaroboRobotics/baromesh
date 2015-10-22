#include "osx_uniqueioobject.hpp"

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include "CF++.h"

#include <IOKit/IOKitLib.h>
#include <IOKit/IOBSD.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOReturn.h>

#include <algorithm>

#include <string.h>
#include <unistd.h>

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
    // OS X 10.11 overhauled the USB system, introducing the IOUSBHostDevice
    // class name.
    auto classes = IOServiceMatching("IOUSBHostDevice");
    if (!classes) {
        // Fall back to the IOUSBDevice class name--we are on 10.10 or less.
        classes = IOServiceMatching(kIOUSBDeviceClassName);
    }
    io_iterator_t iter;
    auto kr = IOServiceGetMatchingServices(kIOMasterPortDefault, classes, &iter);
    if (kIOReturnSuccess != kr) {
        throw std::runtime_error("Could not get USB devices from the IORegistry.");
    }
    return UniqueIoObject{iter};
}

int dongleDevicePathImpl (char *buf, size_t len) {
    boost::log::sources::logger lg;
    auto iter = getUsbDeviceIterator();
    while (auto device = UniqueIoObject{IOIteratorNext(iter)}) {
        for (auto i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
            auto expectedManufacturer = std::string(g_barobo_usb_dongle_ids[i].manufacturer);
            auto expectedProduct = std::string(g_barobo_usb_dongle_ids[i].product);

            auto manufacturerValue = std::string(getStringProperty(device, "USB Vendor Name"));
            // The device also has a "USB Product Name" property which we ought to
            // be able to use, but on 10.11, OS X mangles '-' to '_', and on 10.10
            // and earlier, the string returned is not null-terminated. The USB
            // product name is available unmangled as the device's registry entry
            // name instead.
            io_name_t buffer;
            auto kr = IORegistryEntryGetNameInPlane(device, kIOServicePlane, buffer);
            if (kIOReturnSuccess != kr) {
                continue;
            }
            auto productValue = std::string(buffer);

            // TODO: test if productValue is truly null-terminated properly on
            // OS X 10.10. Until this is tested, we'll keep the old method of
            // comparing product strings: true if the expected product string
            // is a prefix of the device's product string.
            if (manufacturerValue == expectedManufacturer
                && expectedProduct.size() <= productValue.size()
                && expectedProduct.end() == std::mismatch(
                    expectedProduct.begin(),
                    expectedProduct.end(),
                    productValue.begin()).first) {
                auto path = std::string(getStringProperty(device, "IOCalloutDevice", true));
                if (!access(path.c_str(), R_OK | W_OK)) {
                    // Success!
                    strncpy(buf, path.c_str(), len);
                    buf[len-1] = 0;
                    return 0;
                }
                else if (EACCES == errno) {
                    BOOST_LOG(lg) << "Dongle found at " << path << ", but user does not have "
                        << "sufficient read/write permissions.";
                }
                else {
                    char errbuf[256];
                    strerror_r(errno, errbuf, sizeof(errbuf));
                    BOOST_LOG(lg) << "access(\"" << path << "\", R_OK|W_OK) failed: " << errbuf;
                }
            }
        }
    }
    return -1;
}

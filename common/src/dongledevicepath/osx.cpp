#include "osx_uniqueioobject.hpp"

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include "CF++.h"

#include <IOKit/IOKitLib.h>
#include <IOKit/IOBSD.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOReturn.h>

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
    io_iterator_t iter;
    auto kr = IOServiceGetMatchingServices(kIOMasterPortDefault,
        IOServiceMatching(kIOUSBDeviceClassName), &iter);
    if (kIOReturnSuccess != kr) {
        throw std::runtime_error("Could not get USB devices from the IORegistry.");
    }
    return UniqueIoObject{iter};
}

int dongleDevicePathImpl (char *buf, size_t len) {
    auto iter = getUsbDeviceIterator();
    while (auto device = UniqueIoObject{IOIteratorNext(iter)}) {
        for (auto i = 0; i < NUM_BAROBO_USB_DONGLE_IDS; ++i) {
            auto expectedManufacturer = g_barobo_usb_dongle_ids[i].manufacturer;
            auto expectedProduct = g_barobo_usb_dongle_ids[i].product;

            auto manufacturerValue = std::string(getStringProperty(device, "USB Vendor Name"));
            auto productValue = std::string(getStringProperty(device, "USB Product Name"));

            if (manufacturerValue == expectedManufacturer
                && productValue == expectedProduct) {
                auto path = std::string(getStringProperty(device, "IOCalloutDevice", true));
                if (!access(path.c_str(), R_OK | W_OK)) {
                    // Success!
                    strncpy(buf, path.c_str(), len);
                    buf[len-1] = 0;
                    return 0;
                }
                else if (EACCES == errno) {
                    boost::log::sources::logger lg;
                    BOOST_LOG(lg) << "Dongle found at " << path << ", but user does not have "
                        << "sufficient read/write permissions.";
                    
                }
                else {
                    char errbuf[256];
                    strerror_r(errno, errbuf, sizeof(errbuf));
                    boost::log::sources::logger lg;
                    BOOST_LOG(lg) << "access(\"" << path << "\", R_OK|W_OK) failed: " << errbuf;
                }
            }
        }
    }
    return -1;
}

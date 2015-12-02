#include "baromesh/system_error.hpp"

#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <memory>

#include "windowsguids.hpp"
#include "windowserror.hpp"

#include <windows.h>
#include <setupapi.h>

#include <devpkey.h>

// Stackoverflow Oleg
typedef BOOL (WINAPI *FN_SetupDiGetDevicePropertyW)(
    __in       HDEVINFO DeviceInfoSet,
    __in       PSP_DEVINFO_DATA DeviceInfoData,
    __in       const DEVPROPKEY *PropertyKey,
    __out      DEVPROPTYPE *PropertyType,
    __out_opt  PBYTE PropertyBuffer,
    __in       DWORD PropertyBufferSize,
    __out_opt  PDWORD RequiredSize,
    __in       DWORD Flags
);

static FN_SetupDiGetDevicePropertyW fn_SetupDiGetDevicePropertyW () {
    static FN_SetupDiGetDevicePropertyW fn
        = (FN_SetupDiGetDevicePropertyW)
            GetProcAddress (GetModuleHandle (TEXT("Setupapi.dll")), "SetupDiGetDevicePropertyW");
    return fn;
}

static bool isWin7OrGreater () {
    return !!fn_SetupDiGetDevicePropertyW();
}

class Device : public SP_DEVINFO_DATA {
public:
    Device (std::shared_ptr<void> handle) : mHandle(handle) {}

    std::string registryProperty (DWORD key) {
        DWORD type;
        DWORD size = 0;
        if (!SetupDiGetDeviceRegistryPropertyA(mHandle.get(), this, key,
                                               &type, nullptr, 0, &size)) {
            auto err = GetLastError();
            if (ERROR_INSUFFICIENT_BUFFER != err) {
                throw WindowsError{"SetupDiGetDeviceRegistryProperty", err};
            }
        }

        auto buf = std::string(size, 0);

        if (!SetupDiGetDeviceRegistryPropertyA(mHandle.get(), this, key,
                                               &type, PBYTE(buf.data()), size, nullptr)) {
            throw WindowsError{"SetupDiGetDeviceRegistryProperty", GetLastError()};
        }

        if (REG_SZ != type) {
            throw std::runtime_error{"Registry property is not a string"};
        }

        return buf;
    }

    std::string property (const DEVPROPKEY* key) {
        DEVPROPTYPE type;
        DWORD size = 0;
        assert(isWin7OrGreater());
        if (!fn_SetupDiGetDevicePropertyW()(mHandle.get(), this, key,
                                       &type, nullptr, 0, &size, 0)) {
            auto err = GetLastError();
            if (ERROR_INSUFFICIENT_BUFFER != err) {
                throw WindowsError{"SetupDiGetDeviceProperty", err};
            }
        }

        auto buf = std::string(size, 0);

        if (!fn_SetupDiGetDevicePropertyW()(mHandle.get(), this, key,
                                       &type, PBYTE(buf.data()), size, nullptr, 0)) {
            throw WindowsError{"SetupDiGetDeviceProperty", GetLastError()};
        }

        if (DEVPROP_TYPE_STRING != type) {
            throw std::runtime_error{"Registry property is not a string"};
        }

        return buf;
    }

private:
    // Intrusive iterator, to save on copying HDEVINFO shared_ptrs around.
    friend class DeviceIterator;
    std::shared_ptr<void> mHandle;
};

class DeviceIterator
    : public boost::iterator_facade<
        DeviceIterator, Device, boost::single_pass_traversal_tag
    > {
public:
    DeviceIterator ()
        : mIndex(kEnd), mDevice(nullptr) {}

    DeviceIterator (std::shared_ptr<void> handle)
        : mIndex(kBegin), mDevice(handle)
    {
        increment();
    }

private:
    friend class boost::iterator_core_access;

    void increment () {
        if (kEnd != mIndex) {
            mDevice.cbSize = sizeof(SP_DEVINFO_DATA);
            if (!SetupDiEnumDeviceInfo(mDevice.mHandle.get(), mIndex++, &mDevice)) {
                auto err = GetLastError();
                if (ERROR_NO_MORE_ITEMS == err) {
                    mIndex = kEnd;
                }
                else if (ERROR_SUCCESS != err) {
                    throw WindowsError{"SetupDiEnumDeviceInfo", err};
                }
            }
        }
    }

    bool equal (const DeviceIterator& other) const {
        // To be equal, either our indices are the same and we're both end
        // iterators, or our indices are the same and we use the same handle.
        return mIndex == other.mIndex
            && (kEnd == mIndex || mDevice.mHandle == other.mDevice.mHandle);
    }

    Device& dereference () const {
        return const_cast<Device&>(mDevice);
    }

    static const DWORD kBegin = 0;
    static const DWORD kEnd = DWORD(-1);
    DWORD mIndex;
    Device mDevice;
};

class DeviceList {
public:
    DeviceList (const GUID* classGuid, const char* enumerator)
        : mHandle(makeDiList(classGuid, enumerator), SetupDiDestroyDeviceInfoList)
    {}

    DeviceIterator begin () { return { mHandle }; }
    DeviceIterator end () { return {}; }

private:
    static HDEVINFO makeDiList (const GUID* classGuid, const char* enumerator) {
        auto diList = SetupDiGetClassDevs(classGuid, enumerator, nullptr, DIGCF_PRESENT);
        if (INVALID_HANDLE_VALUE == diList) {
            auto err = GetLastError();
            throw WindowsError{"SetupDiGetClassDevs", err};
        }
        return diList;
    }

    std::shared_ptr<void> mHandle;
};

/* Find an attached dongle device and return the COM port name via the output
 * parameter tty. tty is a user-supplied buffer of size len. Return the COM
 * port number, if anyone cares. On error, return -1. */
std::string dongleDevicePathImpl (boost::system::error_code& ec) {
    boost::log::sources::logger lg;
    BOOST_LOG(lg) << "Windows 7? " << isWin7OrGreater();
    ec = baromesh::Status::DONGLE_NOT_FOUND;
    try {
        /* Get all USB devices that provide a serial or parallel port interface. */
        auto usbSerialDevices = DeviceList{&GUID_DEVCLASS_PORTS, "USB"};
        /* Now iterate over each device in the COM port interface class. */
        for (auto& device : usbSerialDevices) {
            // The Windows 10 in-box usbser.sys driver reports the product
            // string the DEVPKEY way.
            auto productValue = isWin7OrGreater()
                ? device.property(&DEVPKEY_Device_BusReportedDeviceDesc)
                : device.registryProperty(SPDRP_DEVICEDESC);
            BOOST_LOG(lg) << "productValue: " << productValue;
            if (usbDongleProductStrings().count(productValue)) {
                auto path = device.registryProperty(SPDRP_FRIENDLYNAME);
                BOOST_LOG(lg) << "friendly name: " << path;
                auto first = path.find_last_of('(');
                auto last = path.find_last_of(')');
                if (first >= last || std::string::npos == first || std::string::npos == last) {
                    throw std::runtime_error("Dongle found but it has no device path");
                }
                path.replace(0, first + 1, "\\\\.\\");
                path.erase(last);
                ec = baromesh::Status::OK;
                BOOST_LOG(lg) << "path: " << path;
                return path;
            }
        }
    }
    catch (boost::system::system_error& e) {
        ec = e.code();
    }
    catch (std::exception& e) {
        BOOST_LOG(lg) << "Exception getting dongle device path: " << e.what();
    }
    return "";
}

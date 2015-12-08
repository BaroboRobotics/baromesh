#include "baromesh/system_error.hpp"

#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <iomanip>
#include <memory>

#include "windows_guids.hpp"
#include "windows_error.hpp"
#include "windows_utf.hpp"

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

static std::shared_ptr<void> makeDeviceList (const GUID* classGuid, const char* enumerator) {
    auto diList = SetupDiGetClassDevs(classGuid, enumerator, nullptr, DIGCF_PRESENT);
    if (INVALID_HANDLE_VALUE == diList) {
        auto err = GetLastError();
        throw WindowsError{"SetupDiGetClassDevs", err};
    }
    return std::shared_ptr<void>(diList, SetupDiDestroyDeviceInfoList);
}

class Device : public SP_DEVINFO_DATA {
public:
    std::string path () {
        boost::log::sources::logger lg;
        // The friendly name will be something like "Some Device (COM3)",
        // in which case we would want to return "\\.\COM3".
        // \\.\ is the Windows equivalent of /dev in Linux.
        auto friendly = getRegistryProperty(SPDRP_FRIENDLYNAME);
        friendly.erase(friendly.find_last_of(')'));
        auto first = friendly.find_last_of('(');
        if (std::string::npos != first) {
            friendly.replace(0, first + 1, "\\\\.\\");
            return friendly;
        }
        throw "USB serial device has no path";
        return {};
    }

    std::string productString () {
        // The Windows 10 in-box usbser.sys driver reports the product
        // string the DEVPKEY way.
        return isWin7OrGreater()
            ? getProperty(&DEVPKEY_Device_BusReportedDeviceDesc)
            : getRegistryProperty(SPDRP_DEVICEDESC);
    }

private:
    Device (std::shared_ptr<void> handle) : mHandle(handle) {}

    std::string getRegistryProperty (DWORD key) {
        DWORD type;
        DWORD size = 0;
        if (!SetupDiGetDeviceRegistryPropertyA(mHandle.get(), this,
                                               key, &type,
                                               nullptr, 0,
                                               &size)) {
            auto err = GetLastError();
            if (ERROR_INSUFFICIENT_BUFFER != err) {
                throw WindowsError{"SetupDiGetDeviceRegistryProperty", err};
            }
        }

        auto result = std::vector<char>(size);
        if (!SetupDiGetDeviceRegistryPropertyA(mHandle.get(), this,
                                               key, &type,
                                               PBYTE(result.data()), size,
                                               nullptr)) {
            throw WindowsError{"SetupDiGetDeviceRegistryProperty", GetLastError()};
        }

        if (REG_SZ != type) {
            throw std::runtime_error{"Registry property is not a string"};
        }

        return result.data();
    }

    std::string getProperty (const DEVPROPKEY* key) {
        DEVPROPTYPE type;
        DWORD size = 0;
        assert(isWin7OrGreater());
        if (!fn_SetupDiGetDevicePropertyW()(mHandle.get(), this,
                                            key, &type,
                                            nullptr, 0,
                                            &size, 0)) {
            auto err = GetLastError();
            if (ERROR_INSUFFICIENT_BUFFER != err) {
                throw WindowsError{"SetupDiGetDeviceProperty", err};
            }
        }

        // If the requested size is not a multiple of wstring's element type,
        // bump it to the next multiple.
        const auto wcSize = sizeof(std::wstring::value_type);
        size = (size - 1) + wcSize - (size - 1) % wcSize;
        auto result = std::vector<std::wstring::value_type>(size / wcSize);
        if (!fn_SetupDiGetDevicePropertyW()(mHandle.get(), this,
                                            key, &type,
                                            PBYTE(result.data()), size,
                                            nullptr, 0)) {
            throw WindowsError{"SetupDiGetDeviceProperty", GetLastError()};
        }

        if (DEVPROP_TYPE_STRING != type) {
            throw std::runtime_error{"Registry property is not a string"};
        }

        return toUtf8(result.data());
    }

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
        : mIndex(kEnd), mDevice(nullptr)
    {}

    friend DeviceIterator begin (const DeviceIterator&) {
        return DeviceIterator{makeDeviceList(&GUID_DEVCLASS_PORTS, "USB")};
    }

    friend DeviceIterator end (const DeviceIterator&) {
        return DeviceIterator{};
    }

private:
    friend class boost::iterator_core_access;

    // Create a new begin iterator
    DeviceIterator (std::shared_ptr<void> handle)
        : mIndex(kBegin), mDevice(handle)
    {
        increment();
    }

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

/* Find an attached dongle device and return the COM port name via the output
 * parameter tty. tty is a user-supplied buffer of size len. Return the COM
 * port number, if anyone cares. On error, return -1. */
std::string dongleDevicePathImpl (boost::system::error_code& ec) {
    boost::log::sources::logger lg;
    ec = baromesh::Status::DONGLE_NOT_FOUND;
    try {
        // Iterate through USB CDC devices
        for (auto& device : DeviceIterator{}) {
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

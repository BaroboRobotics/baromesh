#include "devices.hpp"

#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <iomanip>
#include <memory>

#include "windows_guids.hpp"
#include "windows_error.hpp"
#include "windows_utf.hpp"

#include <windows.h>
#include <setupapi.h>

#include <devpkey.h>

namespace usbcdc {

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

static std::shared_ptr<void> makeDevInfoList (const GUID* classGuid, const char* enumerator) {
    auto diList = SetupDiGetClassDevs(classGuid, enumerator, nullptr, DIGCF_PRESENT);
    if (INVALID_HANDLE_VALUE == diList) {
        auto err = GetLastError();
        throw WindowsError{"SetupDiGetClassDevs", err};
    }
    return std::shared_ptr<void>(diList, SetupDiDestroyDeviceInfoList);
}

class DevInfo : public SP_DEVINFO_DATA {
public:
    std::string path () {
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
    DevInfo (std::shared_ptr<void> handle) : mHandle(handle) {}

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
    friend class DevInfoIterator;
    std::shared_ptr<void> mHandle;
};

class DevInfoIterator
    : public boost::iterator_facade<
        DevInfoIterator, DevInfo, boost::single_pass_traversal_tag
    > {
public:
    DevInfoIterator ()
        : mIndex(kEnd), mDevInfo(nullptr)
    {}

    friend DevInfoIterator begin (const DevInfoIterator&) {
        return DevInfoIterator{makeDevInfoList(&GUID_DEVCLASS_PORTS, "USB")};
    }

    friend DevInfoIterator end (const DevInfoIterator&) {
        return DevInfoIterator{};
    }

private:
    friend class boost::iterator_core_access;

    // Create a new begin iterator
    DevInfoIterator (std::shared_ptr<void> handle)
        : mIndex(kBegin), mDevInfo(handle)
    {
        increment();
    }

    void increment () {
        if (kEnd != mIndex) {
            mDevInfo.cbSize = sizeof(SP_DEVINFO_DATA);
            if (!SetupDiEnumDeviceInfo(mDevInfo.mHandle.get(), mIndex++, &mDevInfo)) {
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

    bool equal (const DevInfoIterator& other) const {
        // To be equal, either our indices are the same and we're both end
        // iterators, or our indices are the same and we use the same handle.
        return mIndex == other.mIndex
            && (kEnd == mIndex || mDevInfo.mHandle == other.mDevInfo.mHandle);
    }

    DevInfo dereference () const {
        return mDevInfo;//const_cast<DevInfo&>(mDevInfo);
    }

    static const DWORD kBegin = 0;
    static const DWORD kEnd = DWORD(-1);
    DWORD mIndex;
    DevInfo mDevInfo;
};

static Device toDevice (DevInfo di) {
    return Device{di.path(), di.productString()};
}

using namespace boost::adaptors;

DeviceRange devices () {
    auto diIter = DevInfoIterator{};
    return boost::make_iterator_range(begin(diIter), end(diIter))
        | transformed(toDevice);
}

} // namespace usbcdc
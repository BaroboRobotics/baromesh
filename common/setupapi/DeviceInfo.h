// Downloaded from http://www.codeproject.com/Articles/31749/Device-Information
// Written by Jonathan Davies, tweaked by Harris

//DeviceInfo.h
#pragma once
#include <setupapi.h>					// ::SetupDi********* 
#include <atlbase.h>					// ATLASSERT & ATLTRACE
#include "CommandError.h"				// CWin32ErrToWString
#pragma comment(lib, "setupapi.lib")

/**
	Deals with device driver information via the Setup API by wrapping SetupDiXXXXXXXXXXXX calls.
	Creating this C++ class resulted in the 100 lines of code upon which it was based being replaced by 40 lines in the application.

	Note all strings have been internally declared as being of MAX_PATH length.
	The first time EnumDeviceInfo is called it will access the SP_DEVINFO_DATA at index 0. The next time it is called it will access
	the SP_DEVINFO_DATA at index 1, the next index 2 and so on.

	\version 0.1
	\todo 
      - Should this object use the same copy semantics using Release (Detach in ATL) as CDeviceImageList?

  	\bug None documented.

	\warning Not designed to be inherited.
	\warning The use of a non-standard assignment generates C4239 in the calling application (Level 4 warning)
*/
class CDevInfo
{
private:
	HDEVINFO		m_hDevInfo;			//!< Reference to device information set
	SP_DEVINFO_DATA m_spDevInfoData;	//!< Device information structure (references a device instance that is a member of a device information set)
	BOOL			m_bDevInfo;			//!< Tested to ensure EnumDeviceInfo has been called
	short           m_MemberIndex;		//!< Preserves state between EnumDeviceInfo calls
public:
	/**
		Obtain a handle (held within CDevInfo) to a device information set that contains requested device information elements for a local machine.
		@param hWndParent	A handle of the top-level window to be used for a user interface when associated with installing a device instance in the device information 
							set. Can be NULL.
		@param Flags		The device information set to be enumerated over.
		@param ClassGuid	A pointer to the GUID for a device-class or a device interface class.
		@param Enumerator	A pointer to a NULL-terminated string that supplies the name of a Plug and Play (PnP) enumerator or a PnP device instance identifier.

		To obtain an information set including all devices use the default Flags parameter. To obtain a set of only devices present use:
		\code
		CDevInfo cDevInfo(m_hWnd, DIGCF_ALLCLASSES | DIGCF_PRESENT);
		\endcode
		To obtain a list of devices that have a specific device interface and are present use:
		\code
		const GUID Guid = GUID_DEVCLASS_PORTS;
		CDevInfo cDevInfo(m_hWnd, DIGCF_PRESENT |  DIGCF_DEVICEINTERFACE , &Guid);
		\endcode
	*/
	CDevInfo( HWND hWndParent, DWORD Flags= DIGCF_ALLCLASSES | DIGCF_PROFILE, CONST GUID *ClassGuid=0, PCWSTR Enumerator=0) : 
		m_hDevInfo(0L),
		m_bDevInfo(FALSE),
		m_MemberIndex(-1)
	{
		ATLASSERT(::IsWindow(hWndParent) || 0 == hWndParent);
		m_hDevInfo = SetupDiGetClassDevs(ClassGuid, Enumerator, hWndParent, Flags);
		if(m_hDevInfo == INVALID_HANDLE_VALUE)
		{
			DWORD dwErr = GetLastError();
			std::wstring szErr = CWin32ErrToWString()(dwErr);
			ATLTRACE(_T("SetupDiGetClassDevs failed because %s\n"),szErr.c_str());
			ATLASSERT(m_hDevInfo != INVALID_HANDLE_VALUE);
		}
	}
private:
	/**
		Return the current HDEVINFO and relinquish control.
	*/
	HDEVINFO Release()
	{
		HDEVINFO tmp = m_hDevInfo;
		m_hDevInfo = 0;
		return tmp;
	}
public:
	/**
		Create a new CDevInfo which takes over ownership of a HDEVINFO control set from an existing object.
	*/
	CDevInfo(CDevInfo & _DevInfo) :  m_hDevInfo(_DevInfo.Release()) 
	{
	}

	/**
		Destroy the current HDEVINFO, have the rhs relinquish ownership of its HDEVINFO and takeover ownership.
	*/
	CDevInfo & operator= (CDevInfo & pSource)
	{
		if(&pSource != this)
		{
			if(m_hDevInfo != 0)
			{
				BOOL b = ::SetupDiDestroyDeviceInfoList(m_hDevInfo);
				if(FALSE == b)
				{
					ATLASSERT(!"Failed to destroy device set");
				}
			}
		}
		m_hDevInfo = pSource.Release();
		return *this;
	}
	

	/**
		Gets a Device information structure (references a device instance that is a member of a device information set)
		The private variable m_MemberIndex is set at -1 in the constructor and incremented each time EnumDeviceInfo is called. 
		FALSE is returned if an attempt to enumerate after the last device is made.
	*/
	BOOL EnumDeviceInfo(void)
	{
		ATLASSERT(m_hDevInfo);
		m_MemberIndex++;
		m_spDevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
		m_bDevInfo = ::SetupDiEnumDeviceInfo(m_hDevInfo, m_MemberIndex, &m_spDevInfoData);
		return m_bDevInfo;
	}
	/**
		Retrieve a specified Plug and Play device property.
		@param Property				A value indicating the property to be retrieved such as SPDRP_CLASSGUID or SPDRP_FRIENDLYNAME. See
									http://msdn.microsoft.com/en-us/library/ms792967.aspx for full details.
		@param PropertyBuffer		A pointer to a buffer that receives the property that is being retrieved.
	*/
	BOOL GetDeviceRegistryProperty(DWORD Property, PBYTE PropertyBuffer)
	{
		ATLASSERT(PropertyBuffer);
		ATLASSERT(m_bDevInfo);
		BOOL bGotRegProp = ::SetupDiGetDeviceRegistryProperty(m_hDevInfo, &m_spDevInfoData,
													  Property,
													  0L,
													  PropertyBuffer,
													  2048,
													  0);
		return bGotRegProp;
	}
	/**
		Retrieve the index within the class-image list of a specified device-class.

		@param	ClassImageListData	A pointer to an SP_CLASSIMAGELIST_DATA structure that describes a class-image list that includes 
									the image for the device-class that is specified by the ClassGuid parameter. 
		@param	ImageIndex			A pointer to an INT-typed variable that receives the index of the specified class-image in the 
									class-image list.
	*/
	BOOL GetClassImageIndex(PSP_CLASSIMAGELIST_DATA ClassImageListData, PINT ImageIndex)
	{
		ATLASSERT(ClassImageListData);
		ATLASSERT(ImageIndex);
		ATLASSERT(m_bDevInfo);
		return ::SetupDiGetClassImageIndex(ClassImageListData, &m_spDevInfoData.ClassGuid, ImageIndex);
	}
	/**
		Retrieve the device-class description associated with the specified device-class GUID.
		@param	ClassDescription	A pointer to a character buffer that receives the device-class description.
		For example: wchar_t  szDesc[MAX_PATH] = {0};cDevInfo.GetClassDescription(szDesc);
	*/
	BOOL GetClassDescription(PWSTR ClassDescription)
	{
		ATLASSERT(ClassDescription);
		ATLASSERT(m_bDevInfo);
		return ::SetupDiGetClassDescription(&m_spDevInfoData.ClassGuid, ClassDescription, MAX_PATH, NULL);
	}
	/**
		 Deletes a device information set aquired on creation and frees all associated memory.
	*/
	~CDevInfo()
	{
		if(m_hDevInfo)
		{
			BOOL b = ::SetupDiDestroyDeviceInfoList(m_hDevInfo);
			ATLASSERT(b);
			m_hDevInfo = 0L;
		}
	}
};
/** 
	\example DevInfoTester.cpp
	This is how CDevInfo was developed to be used. This example finds ports by using the port GUID:
 */

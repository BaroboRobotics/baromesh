//DeviceImages.h
// JGD 25.10.2008
#pragma once
#include <setupapi.h>
#pragma comment(lib, "setupapi.lib")

/**
	CDeviceImageList creates a data-structure which is basically an image-list containing icons for all classes of device driver.
	The icons in the list are those used in the Windows Device-Manager view. Icons specific to a manufacturer are not included. 
	Once a device-class image list is created then it can be used to pass the image list to a CListViewCtrl and to obtain the index 
	into the list for a particular class bitmap. The non-standard copy constructor and assignment operator transfer ownership of a 
	data structure rather than than make a copy. The object therefore behaves as an auto_ptr and the rhs releases ownership which 
	is taken up by the lhs. [Ref: C++ in action 10.6-10.8, B.Milewski, ISBN 0-201-69948-6, Addison-Wesley 2001].
*/
class CDeviceImageList
{
private:
	SP_CLASSIMAGELIST_DATA m_spImageData;		//!< Structure containing class image list information.
public:
	/**
		Builds an image-list that contains bitmaps for every installed device-class.
	*/
	CDeviceImageList()
	{
		RtlZeroMemory(&m_spImageData, sizeof(SP_CLASSIMAGELIST_DATA));
		m_spImageData.cbSize = sizeof(SP_CLASSIMAGELIST_DATA);
		BOOL b = SetupDiGetClassImageList(&m_spImageData);
		if(FALSE == b)
		{
			ATLASSERT(!"Failed to aquire class image list");
		}
	}
	/**
		Non-standard copy constructor takes ownership of an image-list rather than making a copy of it.
		@param _spImageData The CDeviceImageList which is relinquishing control of its data structure.
	*/
	CDeviceImageList(CDeviceImageList & _spImageData) : m_spImageData(_spImageData.Release()) 
	{
	}
	/**
		Relinquishs ownership of the class-image list data structure containing the bitmaps.
		@return A structure which describes a class-image list. 
	*/
	SP_CLASSIMAGELIST_DATA Release()
	{
		SP_CLASSIMAGELIST_DATA tmp = m_spImageData;
		RtlZeroMemory(&m_spImageData, sizeof(SP_CLASSIMAGELIST_DATA));
		return tmp;
	}
	/**
		If a bitmap data structure is owned destroy it.
	*/
	~CDeviceImageList()
	{
		if(m_spImageData.ImageList != 0)
		{
			BOOL b = SetupDiDestroyClassImageList(&m_spImageData);
			if(FALSE == b)
			{
				ATLASSERT(!"Failed to destroy class image list");
			}
		}
	}
	/**
		If this CDeviceImageList (the lhs) has ownership of a class-image data-structure it will be replaced 
		by the rhs CDeviceImageList's class-image data-structure.
	*/
	CDeviceImageList & operator= (CDeviceImageList & pSource)
	{
		if(&pSource != this)
		{
			if(m_spImageData.ImageList != 0)
			{
				BOOL b = SetupDiDestroyClassImageList(&m_spImageData);
				if(FALSE == b)
				{
					ATLASSERT(!"Failed to destroy class image list");
				}
			}
			m_spImageData = pSource.Release();
		}
		return *this;
	}
	/**
		Use this operator to get the attached handle of the CDeviceImageList object. 
		@return If successful, a handle to the image list data structure represented by the CDeviceImageList object; otherwise NULL. 
	*/
	operator HIMAGELIST() const throw()
	{ 
		return m_spImageData.ImageList; 
	}
	/**
		Use this operator to get a pointer to the the structure describing a class image list. 
		@return If successful, a handle to the image list data structure represented by the CDeviceImageList object; otherwise NULL. 
	*/
	operator PSP_CLASSIMAGELIST_DATA()
	{
		// Has ownership of the image-list has been relinquished to another object?
		ATLASSERT(m_spImageData.ImageList != 0);
		return static_cast<PSP_CLASSIMAGELIST_DATA>(&m_spImageData);
	}
};


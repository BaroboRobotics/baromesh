#ifndef BAROMESH_DONGLEDEVICEPATH_OSX_UNIQUEIOOBJECT_HPP
#define BAROMESH_DONGLEDEVICEPATH_OSX_UNIQUEIOOBJECT_HPP

#include <IOKit/IOKitLib.h>

class UniqueIoObject {
public:
	UniqueIoObject () = default;
	explicit UniqueIoObject (io_object_t obj) : mObj(obj) {}
	~UniqueIoObject () { IOObjectRelease(mObj); }
	operator io_object_t () const { return mObj; }

	UniqueIoObject (const UniqueIoObject&) = delete;
	UniqueIoObject& operator= (const UniqueIoObject&) = delete;

	UniqueIoObject (UniqueIoObject&& that) {
		swap(*this, that);
	}

	UniqueIoObject& operator= (UniqueIoObject&& that) {
		swap(*this, that);
		return *this;
	}

	friend void swap (UniqueIoObject& lhs, UniqueIoObject& rhs) {
		using std::swap;
		swap(lhs.mObj, rhs.mObj);
	}

private:
	io_object_t mObj = 0;
};

#endif
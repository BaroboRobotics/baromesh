#ifndef BAROMESH_DONGLEDEVICEPATH_OSX_UNIQUEIOOBJECT_HPP
#define BAROMESH_DONGLEDEVICEPATH_OSX_UNIQUEIOOBJECT_HPP

#include <IOKit/IOKitLib.h>

// io_object_t is typically an unsigned int, so the usual shared_ptr<void>
// trick (i.e., shared_ptr<void>(obj, IOObjectRelease)) won't work to implement
// RAII. Instead, we can store a non-owning shared_ptr<void> object on the side
// that calls IOObjectRelease. Note that the io_object_t is stored in two
// places: in SharedIoObject for access, and in the shared_ptr<void> subobject
// for release.
class SharedIoObject {
public:
	SharedIoObject () = default;
	explicit SharedIoObject (io_object_t obj)
		: mGuard(nullptr, [obj] (void*) { IOObjectRelease(obj); })
		, mObj(obj)
	{}
	operator io_object_t () const { return mObj; }
private:
	std::shared_ptr<void> mGuard;
	io_object_t mObj = 0;
};

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
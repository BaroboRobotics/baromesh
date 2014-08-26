struct usb_dongle_id {
  const char *manufacturer;
  const char *product;
};

/* List of valid Barobo dongle manufacturer and product strings. The platform-
 * specific Mobot_dongleGetTTY() functions should depend on this data for
 * finding the dongle. Update this list as necessary. */
static const usb_dongle_id g_barobo_usb_dongle_ids[] = {
  { "Barobo, Inc.", "Mobot USB-Serial Adapter" },
  { "Barobo, Inc.", "Linkbot USB-Serial Adapter" },
  { "Barobo, Inc.", "Barobo USB-Serial Adapter" }
};

/* For convenience. */
#define NUM_BAROBO_USB_DONGLE_IDS \
  (sizeof(g_barobo_usb_dongle_ids) / sizeof(g_barobo_usb_dongle_ids[0]))

#ifdef _WIN32
#include "dongledevicepath/win32.cpp"
#elif defined(__linux__)
#include "dongledevicepath/popen3.c"
#include "dongledevicepath/linux.cpp"
#elif defined(__APPLE__) && defined(__MACH__)
#include "dongledevicepath/osx.cpp"
#else
#error No dongledevicepath.cpp available for this platform.
#endif

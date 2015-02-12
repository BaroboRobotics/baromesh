#ifndef BAROMESH_LINKBOT_HPP
#define BAROMESH_LINKBOT_HPP

#include "baromesh/linkbot.h"

#include <string>
#include <stdint.h>

namespace barobo {

/* A C++03-compatible Linkbot API. */
class Linkbot {
public:
    // Construct a Linkbot backed by a given TCP/IP host and service. For
    // example, Linkbot{"127.0.0.1", "42010"} would attempt to start
    // communicating with a robot interface at localhost:42010.
    Linkbot (const std::string& host, const std::string& service);

    // Ask the daemon to resolve the given serial ID to a TCP/IP host:service,
    // and construct a Linkbot backed by this TCP endpoint.
    explicit Linkbot (const std::string& serialId);

    ~Linkbot ();

private:
    // noncopyable
    Linkbot (const Linkbot&);
    Linkbot& operator= (const Linkbot&);

public:
    std::string serialId () const;

    bool operator== (const Linkbot& that) const {
      return this->serialId() == that.serialId();
    }

    bool operator!= (const Linkbot& that) const {
        return !operator==(that);
    }

    // All member functions may throw a barobo::Error exception on failure.

    /* GETTERS */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void getAccelerometer (int& timestamp, double&, double&, double&);
    void getFormFactor(FormFactor::Type & form);
    void getJointAngles (int& timestamp, double&, double&, double&);
    void getJointSpeeds(double&, double&, double&);
    void getJointStates(int& timestamp, 
                        JointState::Type & s1,
                        JointState::Type & s2,
                        JointState::Type & s3);
    void getLedColor (int&, int&, int&);
    void getVersions (uint32_t&, uint32_t&, uint32_t&);

    /* SETTERS */
    void resetEncoderRevs();
    void setBuzzerFrequency (double);
    void setJointSpeeds (int mask, double, double, double);
    void setJointStates(
        int mask,
        JointState::Type s1, double d1,
        JointState::Type s2, double d2,
        JointState::Type s3, double d3);
    void setLedColor (int, int, int);

    /* MOVEMENT */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void drive (int mask, double, double, double);
    void driveTo (int mask, double, double, double);
    void move (int mask, double, double, double);
    // moveContinuous takes three angular speed coefficients. Use -1 to move
    // a motor backward, +1 to move it forward.
    void moveContinuous (int mask, double, double, double);
    void moveTo (int mask, double, double, double);
    void motorPower(int mask, int m1, int m2, int m3);
    void stop (int mask = 0x07);

    /* CALLBACKS */
    typedef void (*ButtonEventCallback)(Button::Type button, ButtonState::Type event, int timestamp, void* userData);
    // EncoderEventCallback's anglePosition parameter is reported in degrees.
    typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
    typedef void (*JointEventCallback)(int jointNo, JointState::Type event, int timestamp, void* userData);
    typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);

    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    void setButtonEventCallback (ButtonEventCallback, void* userData);
    void setEncoderEventCallback (EncoderEventCallback, double granularity, void* userData);
    void setJointEventCallback (JointEventCallback, void* userData);
    void setAccelerometerEventCallback (AccelerometerEventCallback, void* userData);

    /* MISC */
    void writeEeprom(uint32_t address, const uint8_t *data, size_t size);

private:
    struct Impl;
    Impl* m;
};

} // namespace barobo

#endif

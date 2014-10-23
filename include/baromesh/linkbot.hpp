#ifndef BAROMESH_LINKBOT_HPP
#define BAROMESH_LINKBOT_HPP

#include "baromesh/error.hpp"

#include <string>

namespace barobo {

enum ButtonState {
    UP,
    DOWN
};

enum JointState {
    JOINT_HOLD,
    JOINT_STOP,
    JOINT_FAIL
};

enum MotorDir {
    FORWARD,
    BACKWARD,
    NEUTRAL,
    HOLD
};

/* A C++03-compatible Linkbot API. */
class Linkbot {
public:
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

    void connect ();
    void disconnect ();

    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void drive (int mask, double, double, double);
    void driveTo (int mask, double, double, double);
    void getJointAngles (int& timestamp, double&, double&, double&, int=10);
    void getAccelerometer (int& timestamp, double&, double&, double&);
    void move (int mask, double, double, double);
    void moveContinuous (int mask, MotorDir dir1, MotorDir dir2, MotorDir dir3);
    void moveTo (int mask, double, double, double);
    void moveWait (int mask);
    void setLedColor (int, int, int);
    void setEncoderEventThreshold (int, double);
    void setJointSpeeds (int mask, double, double, double);
    void stop ();
    void setBuzzerFrequencyOn (float);
    void getVersions (uint32_t&, uint32_t&, uint32_t&);

    typedef void (*ButtonEventCallback)(int buttonNo, ButtonState event, int timestamp, void* userData);
    // EncoderEventCallback's anglePosition parameter is reported in degrees.
    typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
    typedef void (*JointEventCallback)(int jointNo, JointState event, void* userData);
    typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);

    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    void setButtonEventCallback (ButtonEventCallback, void* userData);
    void setEncoderEventCallback (EncoderEventCallback, void* userData);
    void setJointEventCallback (JointEventCallback, void* userData);
    void setAccelerometerEventCallback (AccelerometerEventCallback, void* userData);

private:
    struct Impl;
    Impl* m;
};

} // namespace barobo

#endif

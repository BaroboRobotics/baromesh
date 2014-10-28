#ifndef BAROMESH_LINKBOT_HPP
#define BAROMESH_LINKBOT_HPP

#include "baromesh/error.hpp"

#include <string>

namespace barobo {

// Since C++03 does not support enum classes, emulate them using namespaces.
// Usage example:
//     ButtonState::Type bs = ButtonState::UP;
namespace ButtonState {
    enum Type {
        UP,
        DOWN
    };
}

namespace FormFactor {
    enum Type {
        I,
        L,
        T
    };
}

namespace JointState {
    enum Type {
        STOP,
        HOLD,
        MOVING,
        FAIL
    };
}

namespace MotorDir {
    enum Type {
        FORWARD,
        BACKWARD,
        NEUTRAL,
        HOLD
    };
}

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
    void getAccelerometer (int& timestamp, double&, double&, double&);
    void getFormFactor(FormFactor::Type & form);
    void getJointAngles (int& timestamp, double&, double&, double&);
    void getJointStates(int& timestamp, 
                        JointState::Type & s1,
                        JointState::Type & s2,
                        JointState::Type & s3);
    void move (int mask, double, double, double);
    void moveContinuous (int mask, MotorDir::Type dir1, MotorDir::Type dir2, MotorDir::Type dir3);
    void moveTo (int mask, double, double, double);
    /* Debate: Should moveWait be implemented in a higher level? Technically, it
     * can be implemented with other existing functions, thereby making it not a
     * member of the set of primitives... */
    //void moveWait (int mask); 
    void setLedColor (int, int, int);
    void getLedColor (int&, int&, int&);
    void setEncoderEventThreshold (int, double);
    void setJointSpeeds (int mask, double, double, double);
    void stop ();
    void setBuzzerFrequencyOn (float);
    void getVersions (uint32_t&, uint32_t&, uint32_t&);

    typedef void (*ButtonEventCallback)(int buttonNo, ButtonState::Type event, int timestamp, void* userData);
    // EncoderEventCallback's anglePosition parameter is reported in degrees.
    typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
    typedef void (*JointEventCallback)(int jointNo, JointState::Type event, int timestamp, void* userData);
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

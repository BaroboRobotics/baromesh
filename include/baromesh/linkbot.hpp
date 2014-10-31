#ifndef BAROMESH_LINKBOT_HPP
#define BAROMESH_LINKBOT_HPP

#include "baromesh/error.hpp"
#include "baromesh/linkbot.h"

#include <string>
#include <stdint.h>

namespace barobo {

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
    virtual void drive (int mask, double, double, double);
    virtual void driveTo (int mask, double, double, double);
    virtual void getAccelerometer (int& timestamp, double&, double&, double&);
    virtual void getFormFactor(FormFactor & form);
    virtual void getJointAngles (int& timestamp, double&, double&, double&, int=10);
    virtual void getJointStates(int& timestamp, 
                        JointState & s1, 
                        JointState & s2, 
                        JointState & s3);
    virtual void move (int mask, double, double, double);
    virtual void moveContinuous (int mask, MotorDir dir1, MotorDir dir2, MotorDir dir3);
    virtual void moveTo (int mask, double, double, double);
    /* Debate: Should moveWait be implemented in a higher level? Technically, it
     * can be implemented with other existing functions, thereby making it not a
     * member of the set of primitives... */
    //void moveWait (int mask); 
    virtual void setLedColor (int, int, int);
    virtual void setEncoderEventThreshold (int, double);
    virtual void setJointSpeeds (int mask, double, double, double);
    virtual void stop ();
    virtual void setBuzzerFrequencyOn (float);
    void getVersions (uint32_t&, uint32_t&, uint32_t&);

    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    void setButtonEventCallback (ButtonEventCallback, void* userData);
    void setEncoderEventCallback (EncoderEventCallback, float granularity, void* userData);
    void setEncoderEventCallback (EncoderEventCallback, void* userData);
    void setJointEventCallback (JointEventCallback, void* userData);
    void setAccelerometerEventCallback (AccelerometerEventCallback, void* userData);

private:
    struct Impl;
    Impl* m;
};

} // namespace barobo

#endif

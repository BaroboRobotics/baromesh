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
    // All member functions may throw a barobo::Error exception on failure.

    /* GETTERS */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void getAccelerometer (int& timestamp, double&, double&, double&);
    void getBatteryVoltage(double& voltage);
    void getFormFactor(FormFactor::Type & form);
    void getJointAngles (int& timestamp, double&, double&, double&);
    void getJointSpeeds(double&, double&, double&);
    void getJointStates(int& timestamp, 
                        JointState::Type & s1,
                        JointState::Type & s2,
                        JointState::Type & s3);
    void getLedColor (int&, int&, int&);
    void getVersions (uint32_t&, uint32_t&, uint32_t&);
    void getSerialId(std::string& serialId);
    void getJointSafetyThresholds(int&, int&, int&);
    void getJointSafetyAngles(double&, double&, double&);

    /* SETTERS */
    void resetEncoderRevs();
    void setBuzzerFrequency (double);
    void setJointAccelI(int mask, double, double, double);
    void setJointAccelF(int mask, double, double, double);
    void setJointSpeeds (int mask, double, double, double);
    void setJointStates(
        int mask,
        JointState::Type s1, double d1,
        JointState::Type s2, double d2,
        JointState::Type s3, double d3);
    void setJointStates(
        int mask,
        JointState::Type s1, double d1, double timeout1, JointState::Type end1,
        JointState::Type s2, double d2, double timeout2, JointState::Type end2,
        JointState::Type s3, double d3, double timeout3, JointState::Type end3
        );
    void setLedColor (int, int, int);
    void setJointSafetyThresholds(int mask, int t1=100, int t2=100, int t3=100);
    void setJointSafetyAngles(int mask, double t1=10, double t2=10, double t3=10);

    /* MOVEMENT */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void drive (int mask, double, double, double);
    void driveTo (int mask, double, double, double);
    void move (int mask, double, double, double);
    // moveContinuous takes three angular speed coefficients. Use -1 to move
    // a motor backward, +1 to move it forward.
    void moveAccel(int mask, int relativeMask,
        double a0, double timeout0, JointState::Type endstate0,
        double a1, double timeout1, JointState::Type endstate1,
        double a2, double timeout2, JointState::Type endstate2);
    void moveContinuous (int mask, double, double, double);
    void moveTo (int mask, double, double, double);
    void moveSmooth(int mask, int relativeMask, double a0, double a1, double a2);
    void motorPower(int mask, int m1, int m2, int m3);
    void stop (int mask = 0x07);

    /* CALLBACKS */
    typedef void (*ButtonEventCallback)(Button::Type button, ButtonState::Type event, int timestamp, void* userData);
    // EncoderEventCallback's anglePosition parameter is reported in degrees.
    typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
    typedef void (*JointEventCallback)(int jointNo, JointState::Type event, int timestamp, void* userData);
    typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);
    typedef void (*ConnectionTerminatedCallback)(int timestamp, void* userData);

    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    void setButtonEventCallback (ButtonEventCallback, void* userData);
    void setEncoderEventCallback (EncoderEventCallback, double granularity, void* userData);
    void setJointEventCallback (JointEventCallback, void* userData);
    void setAccelerometerEventCallback (AccelerometerEventCallback, void* userData);
    void setConnectionTerminatedCallback (ConnectionTerminatedCallback, void* userData);

    /* MISC */
    void writeEeprom(uint32_t address, const uint8_t *data, size_t size);
    void readEeprom(uint32_t address, size_t recvsize, uint8_t *buffer);
    void writeTwi(uint32_t address, const uint8_t *data, size_t size);
    void readTwi(uint32_t address, size_t recvsize, uint8_t *buffer);
    void writeReadTwi(
        uint32_t address, 
        const uint8_t *sendbuf, 
        size_t sendsize,
        uint8_t* recvbuf,
        size_t recvsize);

private:
    struct Impl;
    Impl* m;
};

} // namespace barobo

#endif

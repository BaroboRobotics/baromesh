#ifndef QLINKBOT_H__
#define QLINKBOT_H__

#include <stdexcept>
#include <string>

namespace barobo {

enum MotorDir {
    FORWARD,
    BACKWARD,
    NEUTRAL,
    HOLD
};

/* A C++03-compatible Linkbot API. */
class Linkbot {
public:
    explicit Linkbot (const std::string&);
    ~Linkbot ();

private:
    // noncopyable
    Linkbot (const Linkbot&);
    Linkbot& operator= (const Linkbot&);

public:
    void connectRobot();
    void disconnectRobot();
    int enableAccelEventCallback();
    int enableButtonCallback();
    int enableJointEventCallback();
    int disableAccelEventCallback ();
    int disableButtonCallback ();
    int disableJointEventCallback ();
    std::string getSerialID() const;

    inline bool operator== (const Linkbot& other) {
      return this->getSerialID() == other.getSerialID();
    }

    inline bool operator!= (const Linkbot& other) {
        return !operator==(other);
    }

    // functions take angles in degrees
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions
    int drive(int mask, double, double, double);
    int driveTo(int mask, double, double, double);
    int getJointAngles (int & timestamp, double&, double&, double&, int=10);
    int getAccelerometer(int &timestamp, double&, double&, double&);
    int move (int mask, double, double, double);
    int moveContinuous(int mask, MotorDir dir1, MotorDir dir2, MotorDir dir3);
    int moveTo (int mask, double, double, double);
    int moveWait(int mask);
    int setColorRGB (int, int, int);
    int setJointEventThreshold (int, double);
    int setJointSpeeds (int mask, double, double, double);
    int stop ();
    int setBuzzerFrequencyOn (float);
    int getVersions (uint32_t&, uint32_t&, uint32_t&);

    // Exceptions thrown by connectRobot(). Maybe get rid of them, switch to
    // return codes for everything?
    struct ConnectionRefused : std::runtime_error {
        ConnectionRefused (std::string s) : std::runtime_error(s) { }
    };

    struct VersionMismatch : std::runtime_error {
        VersionMismatch (std::string s) : std::runtime_error(s) { }
    };

    typedef void (*ButtonChangedCallback)(int button, int event, void* userData);
    typedef void (*JointsChangedCallback)(double j1, double j2, double j3, int mask, void* userData);
    typedef void (*JointChangedCallback)(int joint, double anglePosition, void* userData);
    typedef void (*AccelChangedCallback)(double x, double y, double z, void* userData);

    void setButtonChangedCallback (ButtonChangedCallback cb, void* userData);
    void setJointsChangedCallback (JointsChangedCallback cb, void* userData);
    void setJointChangedCallback (JointChangedCallback cb, void* userData);
    void setAccelChangedCallback (AccelChangedCallback cb, void* userData);

private:
    struct Impl;
    Impl* m;
};

} // namespace barobo

#endif

#ifndef QLINKBOT_H__
#define QLINKBOT_H__

#include "qbarobo_global.h"

#include <QObject>

#include <stdexcept>
#include <memory>

enum MotorDir {
    FORWARD,
    BACKWARD,
    NEUTRAL,
    HOLD
};

class QLINKBOT_API QLinkbot : public QObject
{
    Q_OBJECT
public:
    explicit QLinkbot(const QString&);
    ~QLinkbot ();

    // noncopyable
    QLinkbot (const QLinkbot&) = delete;
    QLinkbot& operator= (const QLinkbot&) = delete;

    // movable
    friend void swap (QLinkbot& lhs, QLinkbot& rhs);
    QLinkbot (QLinkbot&&);

    void connectRobot();
    void disconnectRobot();
    int enableAccelEventCallback();
    int enableButtonCallback();
    int enableJointEventCallback();
    int disableAccelEventCallback ();
    int disableButtonCallback ();
    int disableJointEventCallback ();
    QString getSerialID() const;

    inline bool operator==(const QLinkbot& other) { 
      return this->getSerialID() == other.getSerialID();
    }
    inline bool operator!=(const QLinkbot& other){return !operator==(other);}

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

    struct ConnectionRefused : std::runtime_error {
        ConnectionRefused (std::string s) : std::runtime_error(s) { }
    };
    struct VersionMismatch : std::runtime_error {
        VersionMismatch (std::string s) : std::runtime_error(s) { }
    };

signals:
    void buttonChanged(QLinkbot *linkbot, int button, int event);
    void jointsChanged(QLinkbot *linkbot, double j1, double j2, double j3, int mask);
    void jointChanged(QLinkbot *linkbot, int joint, double anglePosition);
    void accelChanged(QLinkbot *linkbot, double x, double y, double z);

public slots:
    void newAccelValues(double x, double y, double z);
    void newButtonValues(int button, int buttonDown);
    void newMotorValues(double j1, double j2, double j3, int mask);

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};

#endif

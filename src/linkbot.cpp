#include "barobo/qlinkbot.hpp"

#include "baromesh/baromesh.hpp"

#include <iostream>

#undef M_PI
#define M_PI 3.14159265358979323846

namespace {

template <class T>
T degToRad (T x) { return T(double(x) * M_PI / 180.0); }

template <class T>
T radToDeg (T x) { return T(double(x) * 180.0 / M_PI); }

} // file namespace

using MethodIn = rpc::MethodIn<barobo::Robot>;

struct Linkbot::Impl {
    Impl (const std::string& id)
        : serialId(id)
        , proxy(id.toStdString()) { }

    std::string serialId;
    robot::Proxy proxy;
};

Linkbot::Linkbot(const std::string& id)
        : m(new Linkbot::Impl(id)) {
    m->proxy.buttonEvent.connect(BIND_MEM_CB(&Linkbot::newButtonValues, this));
    m->proxy.encoderEvent.connect(BIND_MEM_CB(&Linkbot::newMotorValues, this));
}

// Out-of-line destructor (even if empty) is needed for unique_ptr, see
// http://herbsutter.com/gotw/_100/
Linkbot::~Linkbot () { }

void swap (Linkbot& lhs, Linkbot& rhs) {
    using std::swap;
    swap(lhs.m, rhs.m);
}

Linkbot::Linkbot (Linkbot&& other) {
    swap(*this, other);
}

void Linkbot::disconnectRobot()
{
    m->proxy.disconnect().get();
}

void Linkbot::connectRobot()
{
    auto serviceInfo = m->proxy.connect().get();

    // Check version before we check if the connection succeeded--the user will
    // probably want to know to flash the robot, regardless.
    if (serviceInfo.rpcVersion() != rpc::Version<>::triplet()) {
        throw VersionMismatch(m->serialId.toStdString() + " RPC version " +
            to_string(serviceInfo.rpcVersion()) + " != local RPC version " +
            to_string(rpc::Version<>::triplet()));
    }
    else if (serviceInfo.interfaceVersion() != rpc::Version<barobo::Robot>::triplet()) {
        throw VersionMismatch(m->serialId.toStdString() + " Robot interface version " +
            to_string(serviceInfo.interfaceVersion()) + " != local Robot interface version " +
            to_string(rpc::Version<barobo::Robot>::triplet()));
    }

    if (serviceInfo.connected()) {
        qDebug().nospace() << qPrintable(m->serialId) << ": connected";
    }
    else {
        throw ConnectionRefused(m->serialId.toStdString() + " refused our connection");
    }
}

int Linkbot::enableAccelEventCallback()
{
#warning Unimplemented stub function in qlinkbot
    qWarning() << "Unimplemented stub function in qlinkbot";
}

int Linkbot::enableButtonCallback()
{
    try {
        m->proxy.fire(MethodIn::enableButtonEvent{true}).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::enableJointEventCallback()
{
    try {
        m->proxy.fire(MethodIn::enableEncoderEvent {
            true, { true, degToRad(float(20.0)) },
            true, { true, degToRad(float(20.0)) },
            true, { true, degToRad(float(20.0)) }
        }).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
    }
}

std::string Linkbot::getSerialID() const {
    return m->serialId;
}

void Linkbot::newAccelValues(double x, double y, double z)
{
    emit accelChanged(this, x, y, z);
}

void Linkbot::newButtonValues(int button, int buttonDown)
{
    emit buttonChanged(this, button, buttonDown);
}

void Linkbot::newMotorValues(double j1, double j2, double j3, int mask)
{
    emit jointsChanged(this, j1, j2, j3, mask);
    double angles[3];
    angles[0] = j1;
    angles[1] = j2;
    angles[2] = j3;
    int i;
    for(i = 0; i < 3; i++) {
        if(mask & (1<<i)) {
            emit jointChanged(this, i+1, angles[i]);
        }
    }
}

int Linkbot::setJointSpeeds (double s0, double s1, double s2) {
    try {
        auto f0 = m->proxy.fire(MethodIn::setMotorControllerOmega { 0, float(degToRad(s0)) });
        auto f1 = m->proxy.fire(MethodIn::setMotorControllerOmega { 1, float(degToRad(s1)) });
        auto f2 = m->proxy.fire(MethodIn::setMotorControllerOmega { 2, float(degToRad(s2)) });
        f0.get();
        f1.get();
        f2.get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::disableAccelEventCallback () {
#warning Unimplemented stub function in qlinkbot
    qWarning() << "Unimplemented stub function in qlinkbot";
    return 0;
}

int Linkbot::disableButtonCallback () {
    try {
        m->proxy.fire(MethodIn::enableButtonEvent{false}).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::disableJointEventCallback () {
    try {
        m->proxy.fire(MethodIn::enableEncoderEvent {
            true, { false, 0 },
            true, { false, 0 },
            true, { false, 0 }
        }).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
    }
    return 0;
}

int Linkbot::getJointAngles (double& a0, double& a1, double& a2, int) {
    try {
        auto values = m->proxy.fire(MethodIn::getEncoderValues{}).get();
        assert(values.values_count >= 3);
        a0 = radToDeg(values.values[0]);
        a1 = radToDeg(values.values[1]);
        a2 = radToDeg(values.values[2]);
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::moveNB (double a0, double a1, double a2) {
    try {
        m->proxy.fire(MethodIn::move {
            true, { barobo_Robot_Goal_Type_RELATIVE, float(degToRad(a0)) },
            true, { barobo_Robot_Goal_Type_RELATIVE, float(degToRad(a1)) },
            true, { barobo_Robot_Goal_Type_RELATIVE, float(degToRad(a2)) }
        }).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::moveToNB (double a0, double a1, double a2) {
    try {
        m->proxy.fire(MethodIn::move {
            true, { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a0)) },
            true, { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a1)) },
            true, { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a2)) }
        }).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::stop () {
    try {
        m->proxy.fire(MethodIn::stop{}).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::setColorRGB (int r, int g, int b) {
    try {
        m->proxy.fire(MethodIn::setLedColor{
            uint32_t(r << 16 | g << 8 | b)
        }).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::setJointEventThreshold (int, double) {
#warning Unimplemented stub function in qlinkbot
    qWarning() << "Unimplemented stub function in qlinkbot";
    return 0;
}

int Linkbot::setBuzzerFrequencyOn (float freq) {
    try {
        m->proxy.fire(MethodIn::setBuzzerFrequency{freq}).get();
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::getVersions (uint32_t& major, uint32_t& minor, uint32_t& patch) {
    try {
        auto version = m->proxy.fire(MethodIn::getFirmwareVersion{}).get();
        major = version.major;
        minor = version.minor;
        patch = version.patch;
        qDebug().nospace() << qPrintable(m->serialId) << " Firmware version "
                           << major << '.' << minor << '.' << patch;
    }
    catch (std::exception& e) {
        qDebug().nospace() << qPrintable(m->serialId) << ": " << e.what();
        return -1;
    }
    return 0;
}

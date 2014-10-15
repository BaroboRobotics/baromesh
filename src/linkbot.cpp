#include "baromesh/robotproxy.hpp"

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include <iostream>
#include <memory>
#include <string>

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
        , proxy(id)
    {}

    mutable boost::log::sources::logger log;

    std::string serialId;
    robot::Proxy proxy;

    void newButtonValues (int button, int event) {
        if (buttonChangedCallback) {
            buttonChangedCallback(button, event);
        }
    }

    void newMotorValues(double j1, double j2, double j3, int mask) {
        if (jointsChangedCallback) {
            jointsChangedCallback(j1, j2, j3, mask);
        }

        if (jointChangedCallback) {
            double angles[3];
            angles[0] = j1;
            angles[1] = j2;
            angles[2] = j3;
            int i;
            for(i = 0; i < 3; i++) {
                if(mask & (1<<i)) {
                    jointChangedCallback(i+1, angles[i]);
                }
            }
        }
    }


    std::function<void(int,int)> buttonChangedCallback;
    std::function<void(double,double,double,int)> jointsChangedCallback;
    std::function<void(int,double)> jointChangedCallback;
    std::function<void(double,double,double)> accelChangedCallback;
};

Linkbot::Linkbot (const std::string& id)
    : m(nullptr)
{
    // By using a unique_ptr, we can guarantee that our Impl will get cleaned
    // up if any code in the rest of the ctor throws.
    std::unique_ptr<Impl> p { new Linkbot::Impl(id) };

    p->proxy.buttonEvent.connect(
        BIND_MEM_CB(&Linkbot::Impl::newButtonValues, p.get())
    );
    p->proxy.encoderEvent.connect(
        BIND_MEM_CB(&Linkbot::Impl::newMotorValues, p.get())
    );

    // Our C++03 API only uses a raw pointer, so transfer ownership from the
    // unique_ptr to the raw pointer. This should always be the last line of
    // the ctor.
    m = p.release();
}

Linkbot::~Linkbot () {
    // This should probably be the only line in Linkbot's dtor. Let Impl's
    // subobjects clean themselves up.
    delete m;
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
        throw VersionMismatch(m->serialId + " RPC version " +
            to_string(serviceInfo.rpcVersion()) + " != local RPC version " +
            to_string(rpc::Version<>::triplet()));
    }
    else if (serviceInfo.interfaceVersion() != rpc::Version<barobo::Robot>::triplet()) {
        throw VersionMismatch(m->serialId + " Robot interface version " +
            to_string(serviceInfo.interfaceVersion()) + " != local Robot interface version " +
            to_string(rpc::Version<barobo::Robot>::triplet()));
    }

    if (serviceInfo.connected()) {
        BOOST_LOG(m->log) << m->serialId << ": connected";
    }
    else {
        throw ConnectionRefused(m->serialId + " refused our connection");
    }
}

using namespace std::placeholders; // _1, _2, etc.

void setButtonChangedCallback (ButtonChangedCallback cb, void* userData) {
    m->buttonChangedCallback =
        cb
        ? std::bind(cb, _1, _2, userData)
        : nullptr;
}

void setJointsChangedCallback (JointsChangedCallback cb, void* userData) {
    m->jointsChangedCallback =
        cb
        ? std::bind(cb, _1, _2, _3, _4, userData)
        : nullptr;
}

void setJointChangedCallback (JointChangedCallback cb, void* userData) {
    m->jointChangedCallback =
        cb
        ? std::bind(cb, _1, _2, userData)
        : nullptr;
}

void setAccelChangedCallback (AccelChangedCallback cb, void* userData) {
    m->accelChangedCallback =
        cb
        ? std::bind(cb, _1, _2, _3, userData)
        : nullptr;
}

int Linkbot::enableAccelEventCallback()
{
#warning Unimplemented stub function in Linkbot
    BOOST_LOG(m->log) << "Unimplemented stub function in Linkbot";
}

int Linkbot::enableButtonCallback()
{
    try {
        m->proxy.fire(MethodIn::enableButtonEvent{true}).get();
    }
    catch (std::exception& e) {
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
    }
}

std::string Linkbot::getSerialID() const {
    return m->serialId;
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::disableAccelEventCallback () {
#warning Unimplemented stub function in Linkbot
    BOOST_LOG(m->log) << "Unimplemented stub function in Linkbot";
    return 0;
}

int Linkbot::disableButtonCallback () {
    try {
        m->proxy.fire(MethodIn::enableButtonEvent{false}).get();
    }
    catch (std::exception& e) {
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
        return -1;
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::stop () {
    try {
        m->proxy.fire(MethodIn::stop{}).get();
    }
    catch (std::exception& e) {
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
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
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
        return -1;
    }
    return 0;
}

int Linkbot::setJointEventThreshold (int, double) {
#warning Unimplemented stub function in Linkbot
    BOOST_LOG(m->log) << "Unimplemented stub function in Linkbot";
    return 0;
}

int Linkbot::setBuzzerFrequencyOn (float freq) {
    try {
        m->proxy.fire(MethodIn::setBuzzerFrequency{freq}).get();
    }
    catch (std::exception& e) {
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
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
        BOOST_LOG(m->log) << m->serialId << " Firmware version "
                           << major << '.' << minor << '.' << patch;
    }
    catch (std::exception& e) {
        BOOST_LOG(m->log) << m->serialId << ": " << e.what();
        return -1;
    }
    return 0;
}

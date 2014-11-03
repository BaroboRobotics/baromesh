#include "baromesh/linkbot.hpp"
#include "baromesh/robotproxy.hpp"

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include <iostream>
#include <memory>
#include <string>

#undef M_PI
#define M_PI 3.14159265358979323846

namespace barobo {

namespace {

template <class T>
T degToRad (T x) { return T(double(x) * M_PI / 180.0); }

template <class T>
T radToDeg (T x) { return T(double(x) * 180.0 / M_PI); }

} // file namespace

using MethodIn = rpc::MethodIn<barobo::Robot>;
using MethodResult = rpc::MethodResult<barobo::Robot>;

struct Linkbot::Impl {
    Impl (const std::string& id)
        : serialId(id)
        , proxy(id)
    {
    }

    mutable boost::log::sources::logger log;

    std::string serialId;
    robot::Proxy proxy;

    void newButtonValues (int button, int event, int timestamp) {
        if (buttonEventCallback) {
            buttonEventCallback(button, static_cast<ButtonState::Type>(event), timestamp);
        }
    }

    void newEncoderValues (int jointNo, double anglePosition, int timestamp) {
        if (encoderEventCallback) {
            encoderEventCallback(jointNo, radToDeg(anglePosition), timestamp);
        }
    }

    void newJointState (int jointNo, int state, int timestamp) {
        if (jointEventCallback) {
            jointEventCallback(jointNo, static_cast<JointState::Type>(state), timestamp);
        }
    }

    void newAccelerometerValues(double x, double y, double z, int timestamp) {
        if (accelerometerEventCallback) {
            accelerometerEventCallback(x, y, z, timestamp);
        }
    }

    std::function<void(int, ButtonState::Type, int)> buttonEventCallback;
    std::function<void(int,double, int)> encoderEventCallback;
    std::function<void(int,JointState::Type, int)> jointEventCallback;
    std::function<void(double,double,double,int)> accelerometerEventCallback;
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
        BIND_MEM_CB(&Linkbot::Impl::newEncoderValues, p.get())
    );
    p->proxy.jointEvent.connect(
        BIND_MEM_CB(&Linkbot::Impl::newJointState, p.get())
    );
    p->proxy.accelerometerEvent.connect(
        BIND_MEM_CB(&Linkbot::Impl::newAccelerometerValues, p.get())
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

std::string Linkbot::serialId () const {
    return m->serialId;
}

void Linkbot::connect()
{
    try {
        auto serviceInfo = m->proxy.connect().get();

        // Check version before we check if the connection succeeded--the user will
        // probably want to know to flash the robot, regardless.
        if (serviceInfo.rpcVersion() != rpc::Version<>::triplet()) {
            throw Error(std::string("RPC version ") +
                to_string(serviceInfo.rpcVersion()) + " != local RPC version " +
                to_string(rpc::Version<>::triplet()));
        }
        else if (serviceInfo.interfaceVersion() != rpc::Version<barobo::Robot>::triplet()) {
            throw Error(std::string("Robot interface version ") +
                to_string(serviceInfo.interfaceVersion()) + " != local Robot interface version " +
                to_string(rpc::Version<barobo::Robot>::triplet()));
        }

        if (serviceInfo.connected()) {
            BOOST_LOG(m->log) << m->serialId << ": connected";
        }
        else {
            throw Error(std::string("connection refused"));
        }
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::disconnect()
{
    try {
        m->proxy.disconnect().get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

using namespace std::placeholders; // _1, _2, etc.

void Linkbot::drive (int mask, double, double, double)
{
    #warning Unimplemented stub function in Linkbot
}

void Linkbot::driveTo (int mask, double, double, double)
{
    #warning Unimplemented stub function in Linkbot
}

void Linkbot::getAccelerometer (int& timestamp, double&, double&, double&)
{
    #warning Unimplemented stub function in Linkbot
}

void Linkbot::getFormFactor(FormFactor::Type& form)
{
    try {
        auto value = m->proxy.fire(MethodIn::getFormFactor{}).get();
        form = FormFactor::Type(value.value);
    } 
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getJointAngles (int& timestamp, double& a0, double& a1, double& a2) {
    try {
        auto values = m->proxy.fire(MethodIn::getEncoderValues{}).get();
        assert(values.values_count >= 3);
        a0 = radToDeg(values.values[0]);
        a1 = radToDeg(values.values[1]);
        a2 = radToDeg(values.values[2]);
        timestamp = values.timestamp;
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getJointStates(int& timestamp, 
                             JointState::Type& s1,
                             JointState::Type& s2,
                             JointState::Type& s3)
{
    try {
        auto values = m->proxy.fire(MethodIn::getJointStates{}).get();
        assert(values.values_count >= 3);
        s1 = static_cast<JointState::Type>(values.values[0]);
        s2 = static_cast<JointState::Type>(values.values[1]);
        s3 = static_cast<JointState::Type>(values.values[2]);
    } 
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setButtonEventCallback (ButtonEventCallback cb, void* userData) {
    const bool enable = !!cb;

    try {
        m->proxy.fire(MethodIn::enableButtonEvent{enable}).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }

    if (enable) {
        m->buttonEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->buttonEventCallback = nullptr;
    }
}

void Linkbot::setEncoderEventCallback (EncoderEventCallback cb, 
                                       float granularity, void* userData) 
{
    const bool enable = !!cb;
    granularity = degToRad(granularity);

    try {
        m->proxy.fire(MethodIn::enableEncoderEvent {
            true, { enable, granularity },
            true, { enable, granularity },
            true, { enable, granularity }
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }

    if (enable) {
        m->encoderEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->encoderEventCallback = nullptr;
    }
}

void Linkbot::setEncoderEventCallback (EncoderEventCallback cb, void* userData) 
{
    const bool enable = !!cb;
    auto granularity = degToRad(enable ? 20.0 : 0);

    try {
        m->proxy.fire(MethodIn::enableEncoderEvent {
            true, { enable, granularity },
            true, { enable, granularity },
            true, { enable, granularity }
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }

    if (enable) {
        m->encoderEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->encoderEventCallback = nullptr;
    }
}

void Linkbot::setJointEventCallback (JointEventCallback cb, void* userData) {
    const bool enable = !!cb;

    try {
        m->proxy.fire(MethodIn::enableJointEvent {
            enable
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }

    if (enable) {
        m->jointEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->jointEventCallback = nullptr;
    }
}

void Linkbot::setAccelerometerEventCallback (AccelerometerEventCallback cb, void* userData) {
    const bool enable = !!cb;
    auto granularity = float(enable ? 0.05 : 0);

    try {
        m->proxy.fire(MethodIn::enableAccelerometerEvent {
            enable, granularity
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }

    if (enable) {
        m->accelerometerEventCallback = std::bind(cb, _1, _2, _3, _4, userData);
    }
    else {
        m->accelerometerEventCallback = nullptr;
    }
}

void Linkbot::setJointSpeeds (int mask, double s0, double s1, double s2) {
    try {
        using Future = std::future<MethodResult::setMotorControllerOmega>;
        std::list<Future> futures;

        uint32_t jointNo = 0;
        for (auto s : { s0, s1, s2 }) {
            if (mask & (1 << jointNo)) {
                auto f = m->proxy.fire(
                    MethodIn::setMotorControllerOmega {
                        jointNo, float(degToRad(s))
                    }
                );
                futures.emplace_back(std::move(f));
            }
            ++jointNo;
        }

        for (auto& f : futures) {
            f.get();
        }
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::move (int mask, double a0, double a1, double a2) {
    try {
        m->proxy.fire(MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE, float(degToRad(a0)) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE, float(degToRad(a1)) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE, float(degToRad(a2)) }
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::moveContinuous (int mask, double c0, double c1, double c2) {
    try {
        m->proxy.fire(MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_INFINITE, float(c0) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_INFINITE, float(c1) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_INFINITE, float(c2) }
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::moveTo (int mask, double a0, double a1, double a2) {
    try {
        m->proxy.fire(MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a0)) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a1)) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a2)) }
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::stop () {
    try {
        m->proxy.fire(MethodIn::stop{}).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setLedColor (int r, int g, int b) {
    try {
        m->proxy.fire(MethodIn::setLedColor{
            uint32_t(r << 16 | g << 8 | b)
        }).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getLedColor (int& r, int& g, int& b) {
    try {
        auto color = m->proxy.fire(MethodIn::getLedColor{}).get();
        r = 0xff & color.value >> 16;
        g = 0xff & color.value >> 8;
        b = 0xff & color.value;
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setEncoderEventThreshold (int, double) {
#warning Unimplemented stub function in Linkbot
    BOOST_LOG(m->log) << "Unimplemented stub function in Linkbot";
}

void Linkbot::setBuzzerFrequencyOn (float freq) {
    try {
        m->proxy.fire(MethodIn::setBuzzerFrequency{freq}).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getVersions (uint32_t& major, uint32_t& minor, uint32_t& patch) {
    try {
        auto version = m->proxy.fire(MethodIn::getFirmwareVersion{}).get();
        major = version.major;
        minor = version.minor;
        patch = version.patch;
        BOOST_LOG(m->log) << m->serialId << " Firmware version "
                           << major << '.' << minor << '.' << patch;
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

} // namespace

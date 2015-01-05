#include "baromesh/linkbot.hpp"
#include "baromesh/error.hpp"
#include "baromesh/iocore.hpp"

#include "gen-robot.pb.hpp"

#include "basicdaemon.hpp"

#include "rpc/asio/client.hpp"
#include "sfp/asio/messagequeue.hpp"

#include <boost/asio/connect.hpp>
#include <boost/asio/use_future.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

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

std::chrono::milliseconds requestTimeout() {
    static const std::chrono::milliseconds kRequestTimeout { 1000 };
    return kRequestTimeout;
}

std::chrono::milliseconds daemonConnectTimeout() {
    static const std::chrono::milliseconds kDaemonConnectTimeout { 1000 };
    return kDaemonConnectTimeout;
}

} // file namespace

using MethodIn = rpc::MethodIn<barobo::Robot>;
using MethodResult = rpc::MethodResult<barobo::Robot>;
using Broadcast = rpc::Broadcast<barobo::Robot>;

using boost::asio::use_future;

struct Linkbot::Impl {
    using Interface = barobo::Robot;
    Impl (const std::string& id)
        : serialId(id)
        , ioCore(baromesh::IoCore::get(false))
        , daemon(ioCore->ios(), log)
        , client(ioCore->ios(), log)
    {
        using Tcp = boost::asio::ip::tcp;
        Tcp::resolver resolver {ioCore->ios()};
        auto daemonEndpointIter = resolver.resolve(Tcp::resolver::query("127.0.0.1", "42000"));

        boost::asio::connect(daemon.client().messageQueue().stream(), daemonEndpointIter);
        daemon.client().messageQueue().asyncHandshake(use_future).get();
        auto info = asyncConnect(daemon.client(), daemonConnectTimeout(), use_future).get();
#warning check for version mismatch
        BOOST_LOG(log) << "Daemon has RPC version " << info.rpcVersion()
                       << ", interface version " << info.interfaceVersion();

        auto robotEndpointIter = daemon.asyncResolveSerialId(serialId, use_future).get();

        BOOST_LOG(log) << "connecting to " << serialId
                          << " at " << robotEndpointIter->endpoint();

        boost::asio::connect(client.messageQueue().stream(), robotEndpointIter);
        client.messageQueue().asyncHandshake(use_future).get();

        clientFinishedFuture = asyncRunClient(client, *this, use_future);

        auto serviceInfo = asyncConnect(client, requestTimeout(), use_future).get();

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
    }

    ~Impl () {
        try {
            asyncDisconnect(client, requestTimeout(), use_future).get();
            daemon.client().close();
            BOOST_LOG(log) << "waiting for asyncRunClient to finish";
            client.close();
            clientFinishedFuture.get();
            BOOST_LOG(log) << "asyncRunClient finished";
        }
        catch (std::exception& e) {
            BOOST_LOG(log) << "Linkbot destructor swallowed exception: " << e.what();
        }
    }

    mutable boost::log::sources::logger log;

    std::string serialId;

    std::shared_ptr<baromesh::IoCore> ioCore;
    using TcpClient = rpc::asio::Client<sfp::asio::MessageQueue<boost::asio::ip::tcp::socket>>;
    baromesh::BasicDaemon<TcpClient> daemon;
    TcpClient client;

    std::future<void> clientFinishedFuture;

    template <class B>
    void broadcast (B&& args) {
        onBroadcast(std::forward<B>(args));
    }

    void onBroadcast (Broadcast::buttonEvent b) {
        if (buttonEventCallback) {
            buttonEventCallback(static_cast<Button::Type>(b.button),
                                static_cast<ButtonState::Type>(b.state),
                                b.timestamp);
        }
    }

    void onBroadcast (Broadcast::encoderEvent b) {
        if (encoderEventCallback) {
            encoderEventCallback(b.encoder, radToDeg(b.value), b.timestamp);
        }
    }

    void onBroadcast (Broadcast::accelerometerEvent b) {
        if (accelerometerEventCallback) {
            accelerometerEventCallback(b.x, b.y, b.z, b.timestamp);
        }
    }

    void onBroadcast (Broadcast::jointEvent b) {
        if (jointEventCallback) {
#warning b.state is barobo_rpc_Robot_JointEventType, which has different values from JointState::Type!!!
            jointEventCallback(b.joint, static_cast<JointState::Type>(b.event), b.timestamp);
        }
    }

    void onBroadcast (Broadcast::debugMessageEvent e) {
        std::cout << "Debug message from robot: " << e.bytestring << std::endl;
    }

    std::function<void(Button::Type, ButtonState::Type, int)> buttonEventCallback;
    std::function<void(int,double, int)> encoderEventCallback;
    std::function<void(int,JointState::Type, int)> jointEventCallback;
    std::function<void(double,double,double,int)> accelerometerEventCallback;
};

Linkbot::Linkbot (const std::string& id) try
    : m(new Linkbot::Impl{id})
{}
catch (std::exception& e) {
    throw Error(id + ": " + e.what());
}

Linkbot::~Linkbot () {
    delete m;
}

std::string Linkbot::serialId () const {
    return m->serialId;
}

using namespace std::placeholders; // _1, _2, etc.

/* GETTERS */

void Linkbot::getAccelerometer (int& timestamp, double&x, double&y, double&z)
{
    try {
        auto value = asyncFire(m->client, MethodIn::getAccelerometerData{}, requestTimeout(), use_future).get();
        x = value.x;
        y = value.y;
        z = value.z;
    } 
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getFormFactor(FormFactor::Type& form)
{
    try {
        auto value = asyncFire(m->client, MethodIn::getFormFactor{}, requestTimeout(), use_future).get();
        form = FormFactor::Type(value.value);
    } 
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getJointAngles (int& timestamp, double& a0, double& a1, double& a2) {
    try {
        auto values = asyncFire(m->client, MethodIn::getEncoderValues{}, requestTimeout(), use_future).get();
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

void Linkbot::getJointSpeeds(double&s1, double&s2, double&s3)
{
    try {
        auto values = asyncFire(m->client, MethodIn::getMotorControllerOmega{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        s1 = radToDeg(values.values[0]);
        s2 = radToDeg(values.values[1]);
        s3 = radToDeg(values.values[2]);
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
        auto values = asyncFire(m->client, MethodIn::getJointStates{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        s1 = static_cast<JointState::Type>(values.values[0]);
        s2 = static_cast<JointState::Type>(values.values[1]);
        s3 = static_cast<JointState::Type>(values.values[2]);
    } 
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getLedColor (int& r, int& g, int& b) {
    try {
        auto color = asyncFire(m->client, MethodIn::getLedColor{}, requestTimeout(), use_future).get();
        r = 0xff & color.value >> 16;
        g = 0xff & color.value >> 8;
        b = 0xff & color.value;
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getVersions (uint32_t& major, uint32_t& minor, uint32_t& patch) {
    try {
        auto version = asyncFire(m->client, MethodIn::getFirmwareVersion{}, requestTimeout(), use_future).get();
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

/* SETTERS */
void Linkbot::resetEncoderRevs() {
    try {
        asyncFire(m->client, MethodIn::resetEncoderRevs{}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setBuzzerFrequencyOn (float freq) {
    try {
        asyncFire(m->client, MethodIn::setBuzzerFrequency{freq}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setJointSpeeds (int mask, double s0, double s1, double s2) {
    try {
        barobo_Robot_setMotorControllerOmega_In arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { s0, s1, s2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(degToRad(s));
            }
            jointFlag <<= 1;
        }
        asyncFire(m->client, arg, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setJointStates(
        int mask,
        JointState::Type s1, double d1,
        JointState::Type s2, double d2,
        JointState::Type s3, double d3
        )
{
    barobo_Robot_Goal_Type goalType[3];
    barobo_Robot_Goal_Controller controllerType[3];
    JointState::Type jointStates[3];
    float coefficients[3];
    jointStates[0] = s1;
    jointStates[1] = s2;
    jointStates[2] = s3;
    coefficients[0] = d1;
    coefficients[1] = d2;
    coefficients[2] = d3;
    for(int i = 0; i < 3; i++) {
        switch(jointStates[i]) {
            case JointState::STOP:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case JointState::HOLD:
                goalType[i] = barobo_Robot_Goal_Type_RELATIVE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
            case JointState::MOVING:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_CONSTVEL;
                break;
            default:
                break;
        }
    }
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { goalType[0], coefficients[0], true, controllerType[0] },
            bool(mask&0x02), { goalType[1], coefficients[1], true, controllerType[1] },
            bool(mask&0x04), { goalType[2], coefficients[2], true, controllerType[2] }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setLedColor (int r, int g, int b) {
    try {
        asyncFire(m->client, MethodIn::setLedColor{
            uint32_t(r << 16 | g << 8 | b)
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

/* MOVEMENT */

void Linkbot::drive (int mask, double a0, double a1, double a2)
{
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(degToRad(a0)),
                               true, 
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(degToRad(a2)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::driveTo (int mask, double a0, double a1, double a2)
{
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, 
                               float(degToRad(a0)),
                               true, 
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, 
                               float(degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, 
                               float(degToRad(a2)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::move (int mask, double a0, double a1, double a2) {
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(degToRad(a0)),
                               false},
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(degToRad(a1)),
                               false},
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(degToRad(a2)),
                               false}
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::moveContinuous (int mask, double c0, double c1, double c2) {
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_INFINITE, float(c0), false },
            bool(mask&0x02), { barobo_Robot_Goal_Type_INFINITE, float(c1), false },
            bool(mask&0x04), { barobo_Robot_Goal_Type_INFINITE, float(c2), false }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::moveTo (int mask, double a0, double a1, double a2) {
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a0)) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a1)) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a2)) }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::motorPower(int mask, int m1, int m2, int m3)
{
    try {
        asyncFire(m->client, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_INFINITE, 
                               float(m1),
                               true, 
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_INFINITE, 
                               float(m2),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_INFINITE, 
                               float(m3),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::stop (int mask) {
    try {
        asyncFire(m->client, MethodIn::stop{true, static_cast<uint32_t>(mask)}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

/* CALLBACKS */

void Linkbot::setAccelerometerEventCallback (AccelerometerEventCallback cb, void* userData) {
    const bool enable = !!cb;
    auto granularity = float(enable ? 0.05 : 0);

    try {
        asyncFire(m->client, MethodIn::enableAccelerometerEvent {
            enable, granularity
        }, requestTimeout(), use_future).get();
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

void Linkbot::setButtonEventCallback (ButtonEventCallback cb, void* userData) {
    const bool enable = !!cb;

    try {
        asyncFire(m->client, MethodIn::enableButtonEvent{enable}, requestTimeout(), use_future).get();
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
                                       double granularity, void* userData)
{
    const bool enable = !!cb;
    granularity = degToRad(granularity);

    try {
        asyncFire(m->client, MethodIn::enableEncoderEvent {
            true, { enable, float(granularity) },
            true, { enable, float(granularity) },
            true, { enable, float(granularity) }
        }, requestTimeout(), use_future).get();
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
        asyncFire(m->client, MethodIn::enableJointEvent {
            enable
        }, requestTimeout(), use_future).get();
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

void Linkbot::writeEeprom(uint32_t address, const uint8_t *data, size_t size)
{
    if(size > 128) {
        throw Error(m->serialId + ": Payload size too large");
    }
    try {
        MethodIn::writeEeprom arg;
        arg.address = address;
        memcpy(arg.data.bytes, data, size);
        arg.data.size = size;
        asyncFire(m->client, arg, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

} // namespace
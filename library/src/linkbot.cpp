#include "baromesh/linkbot.hpp"
#include "baromesh/error.hpp"
#include "baromesh/iocore.hpp"

#include "baromesh/daemon.hpp"

#include "gen-robot.pb.hpp"

#include <boost/asio/use_future.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace barobo {

namespace {

std::chrono::milliseconds requestTimeout () {
    return std::chrono::milliseconds{1000};
}

} // file namespace

using MethodIn = rpc::MethodIn<barobo::Robot>;
using MethodResult = rpc::MethodResult<barobo::Robot>;
using Broadcast = rpc::Broadcast<barobo::Robot>;

using boost::asio::use_future;

struct Linkbot::Impl {
    Impl (const std::string& id)
        : serialId(id)
        , ioCore(baromesh::IoCore::get(false))
        , resolver(ioCore->ios())
        , daemon(ioCore->ios(), log)
        , robot(ioCore->ios(), log)
    {
        auto daemonQuery = decltype(resolver)::query {
            baromesh::daemonHostName(), baromesh::daemonServiceName()
        };
        BOOST_LOG(log) << "Connecting to the daemon at "
                       << daemonQuery.host_name() << ":" << daemonQuery.service_name();
        auto daemonIter = resolver.resolve(daemonQuery);
        baromesh::asyncInitTcpClient(daemon, daemonIter, use_future).get();
        rpc::asio::asyncConnect<barobo::Daemon>(daemon, requestTimeout(), use_future).get();
        daemonRunDone = rpc::asio::asyncRunClient<barobo::Daemon>(daemon, *this, use_future);

        // FIXME rename asyncResolveSerialId since it returns a resolver query,
        // not an endpoint iterator.
        auto robotHostServicePair = baromesh::asyncResolveSerialId(daemon,
            serialId, requestTimeout(), use_future).get();
        BOOST_LOG(log) << "Connecting to " << serialId << " proxy at "
                       << robotHostServicePair.first << ":"
                       << robotHostServicePair.second;
        auto robotIter = resolver.resolve(decltype(resolver)::query{
            robotHostServicePair.first, robotHostServicePair.second});
        baromesh::asyncInitTcpClient(robot, robotIter, use_future).get();
        rpc::asio::asyncConnect<barobo::Robot>(robot, requestTimeout(), use_future).get();
        robotRunDone = rpc::asio::asyncRunClient<barobo::Robot>(robot, *this, use_future);
    }

    ~Impl () {
        if (robotRunDone.valid()) {
            try {
                BOOST_LOG(log) << "Disconnecting robot client";
                asyncDisconnect(robot, requestTimeout(), use_future).get();
                robot.close();
                robotRunDone.get();
            }
            catch (std::exception& e) {
                BOOST_LOG(log) << "Exception during disconnect: " << e.what();
            }
        }
        if (daemonRunDone.valid()) {
            try {
                BOOST_LOG(log) << "Disconnecting daemon client";
                asyncDisconnect(daemon, requestTimeout(), use_future).get();
                daemon.close();
                daemonRunDone.get();
            }
            catch (std::exception& e) {
                BOOST_LOG(log) << "Exception during disconnect: " << e.what();
            }
        }
    }

    template <class B>
    void broadcast (B&& args) {
        onBroadcast(std::forward<B>(args));
    }

    void onBroadcast (rpc::Broadcast<barobo::Daemon>::dongleEvent b) {
        if (!b.status) {
            BOOST_LOG(log) << "Dongle available";
        }
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
            encoderEventCallback(b.encoder, baromesh::radToDeg(b.value), b.timestamp);
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
        BOOST_LOG(log) << "Debug message from robot: " << e.bytestring;
    }

    mutable boost::log::sources::logger log;

    std::string serialId;

    std::shared_ptr<baromesh::IoCore> ioCore;
    boost::asio::ip::tcp::resolver resolver;

    baromesh::TcpClient daemon;
    baromesh::TcpClient robot;

    std::future<void> robotRunDone;
    std::future<void> daemonRunDone;

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
        auto value = asyncFire(m->robot, MethodIn::getAccelerometerData{}, requestTimeout(), use_future).get();
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
        auto value = asyncFire(m->robot, MethodIn::getFormFactor{}, requestTimeout(), use_future).get();
        form = FormFactor::Type(value.value);
    } 
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getJointAngles (int& timestamp, double& a0, double& a1, double& a2) {
    try {
        auto values = asyncFire(m->robot, MethodIn::getEncoderValues{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        a0 = baromesh::radToDeg(values.values[0]);
        a1 = baromesh::radToDeg(values.values[1]);
        a2 = baromesh::radToDeg(values.values[2]);
        timestamp = values.timestamp;
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::getJointSpeeds(double&s1, double&s2, double&s3)
{
    try {
        auto values = asyncFire(m->robot, MethodIn::getMotorControllerOmega{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        s1 = baromesh::radToDeg(values.values[0]);
        s2 = baromesh::radToDeg(values.values[1]);
        s3 = baromesh::radToDeg(values.values[2]);
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
        auto values = asyncFire(m->robot, MethodIn::getJointStates{}, requestTimeout(), use_future).get();
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
        auto color = asyncFire(m->robot, MethodIn::getLedColor{}, requestTimeout(), use_future).get();
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
        auto version = asyncFire(m->robot, MethodIn::getFirmwareVersion{}, requestTimeout(), use_future).get();
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
        asyncFire(m->robot, MethodIn::resetEncoderRevs{}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setBuzzerFrequency (double freq) {
    try {
        asyncFire(m->robot, MethodIn::setBuzzerFrequency{float(freq)}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::setJointSpeeds (int mask, double s0, double s1, double s2) {
    try {
        MethodIn::setMotorControllerOmega arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { s0, s1, s2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(baromesh::degToRad(s));
            }
            jointFlag <<= 1;
        }
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
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
        asyncFire(m->robot, MethodIn::move {
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
        asyncFire(m->robot, MethodIn::setLedColor{
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
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(baromesh::degToRad(a0)),
                               true, 
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(baromesh::degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(baromesh::degToRad(a2)),
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
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, 
                               float(baromesh::degToRad(a0)),
                               true, 
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, 
                               float(baromesh::degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, 
                               float(baromesh::degToRad(a2)),
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
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(baromesh::degToRad(a0)),
                               false},
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(baromesh::degToRad(a1)),
                               false},
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE, 
                               float(baromesh::degToRad(a2)),
                               false}
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::moveContinuous (int mask, double c0, double c1, double c2) {
    try {
        asyncFire(m->robot, MethodIn::move {
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
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, float(baromesh::degToRad(a0)) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, float(baromesh::degToRad(a1)) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, float(baromesh::degToRad(a2)) }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::motorPower(int mask, int m1, int m2, int m3)
{
    try {
        asyncFire(m->robot, MethodIn::move {
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
        asyncFire(m->robot, MethodIn::stop{true, static_cast<uint32_t>(mask)}, requestTimeout(), use_future).get();
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
        asyncFire(m->robot, MethodIn::enableAccelerometerEvent {
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
        asyncFire(m->robot, MethodIn::enableButtonEvent{enable}, requestTimeout(), use_future).get();
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
    granularity = baromesh::degToRad(granularity);

    try {
        asyncFire(m->robot, MethodIn::enableEncoderEvent {
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
        asyncFire(m->robot, MethodIn::enableJointEvent {
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
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::readEeprom(uint32_t address, size_t recvsize, uint8_t *buffer)
{
    if(recvsize > 128) {
        throw Error(m->serialId + ": Payload size too large");
    }
    try {
        MethodIn::readEeprom arg;
        arg.address = address;
        arg.size = recvsize;
        auto result = asyncFire(m->robot, arg, requestTimeout(), use_future).get();
        memcpy(buffer, result.data.bytes, result.data.size);
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::writeTwi(uint32_t address, const uint8_t *data, size_t size)
{
    if(size > 128) {
        throw Error(m->serialId + ": Payload size too large");
    }
    try {
        MethodIn::writeTwi arg;
        arg.address = address;
        memcpy(arg.data.bytes, data, size);
        arg.data.size = size;
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::readTwi(uint32_t address, size_t recvsize, uint8_t *buffer)
{
    if(recvsize > 128) {
        throw Error(m->serialId + ": Payload size too large");
    }
    try {
        MethodIn::readTwi arg;
        arg.address = address;
        arg.recvsize = recvsize;
        auto result = asyncFire(m->robot, arg, requestTimeout(), use_future).get();
        memcpy(buffer, result.data.bytes, result.data.size);
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

void Linkbot::writeReadTwi(
    uint32_t address, 
    const uint8_t *sendbuf, 
    size_t sendsize,
    uint8_t* recvbuf,
    size_t recvsize)
{
    if((recvsize > 128) || (sendsize > 128)) {
        throw Error(m->serialId + ": Payload size too large");
    }
    try {
        MethodIn::writeReadTwi arg;
        arg.address = address;
        arg.recvsize = recvsize;
        memcpy(arg.data.bytes, sendbuf, sendsize);
        arg.data.size = sendsize;
        auto result = asyncFire(m->robot, arg, requestTimeout(), use_future).get();
        memcpy(recvbuf, result.data.bytes, result.data.size);
    }
    catch (std::exception& e) {
        throw Error(m->serialId + ": " + e.what());
    }
}

} // namespace

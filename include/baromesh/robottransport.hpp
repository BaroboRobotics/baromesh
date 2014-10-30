#ifndef ROBOT_TRANSPORT
#define ROBOT_TRANSPORT

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include "dongleproxy.hpp"

namespace robot {

class Transport {
public:
    Transport (std::string serialId) : mSerialId(serialId) {
        auto success = mDongleProxy.registerRobotTransport(this);
        // we are a unique transport
        assert(success); // maybe this should be a throw
    }

    ~Transport () {
        auto success = mDongleProxy.unregisterRobotTransport(this);
        assert(success);
    }

    void sendMessage (const uint8_t* bytes, size_t size) {
        BOOST_LOG_NAMED_SCOPE("robot::Transport::sendMessage");

        barobo_Dongle_SerialId serialId;
        memcpy(serialId.value, mSerialId.c_str(), 5);

        barobo_Dongle_Payload payload;
        assert(size <= sizeof(payload.value.bytes));
        memcpy(payload.value.bytes, bytes, size);
        payload.value.size = size;

        BOOST_LOG(mLog) << "generating transmitUnicast request";
        auto f = mDongleProxy.fire(rpc::MethodIn<barobo::Dongle>::transmitUnicast {
                serialId, payload
        });
        BOOST_LOG(mLog) << "transmitUnicast request sent, waiting on response";
        f.get();
        BOOST_LOG(mLog) << "transmitUnicast complete";
    }

    util::Signal<void(const uint8_t*, size_t)> sigMessageReceived;

    std::string serialId() const { return mSerialId; }

private:
    boost::log::sources::logger_mt mLog;

    const std::string mSerialId;
    dongle::Proxy mDongleProxy;
};

} // namespace robot

#endif

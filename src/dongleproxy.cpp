#include "baromesh/robottransport.hpp"
#include "baromesh/dongleproxy.hpp"

namespace dongle {

bool Proxy::registerRobotTransport(robot::Transport* rt) {
    decltype(mRobotTransports)::iterator i;
    bool success;
    std::tie(i, success) =
        mRobotTransports.insert(std::make_pair(rt->serialId(), rt));
    return success;
}

bool Proxy::unregisterRobotTransport (robot::Transport* rt) {
    auto nRemoved = mRobotTransports.erase(rt->serialId());
    return 1 == nRemoved;
}

void Proxy::onBroadcast(Broadcast::receiveUnicast arg) {
    BOOST_LOG_NAMED_SCOPE("dongle::Proxy::onBroadcast");
    auto i = mRobotTransports.find(arg.serialId.value);
    if (i != mRobotTransports.end()) {
        auto transport = i->second;
        transport->sigMessageReceived(arg.payload.value.bytes,
                                      arg.payload.value.size);
    } else {
        BOOST_LOG(mLog) << "message received for " << arg.serialId.value
                        << " with no registered robot transport, ignoring";
    }
}


} // namespace dongle

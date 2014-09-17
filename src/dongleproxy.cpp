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
#if 0
    printf("received from %s |", arg.serialId.value);
    if (arg.payload.value.size) {
        for (size_t i = 0; i < arg.payload.value.size; ++i) {
            printf(" %02x", arg.payload.value.bytes[i]);
        }
    }
    else {
        printf(" (empty)");
    }
    printf("\n");
#endif
    auto i = mRobotTransports.find(arg.serialId.value);
    if (i != mRobotTransports.end()) {
        auto transport = i->second;
        transport->sigMessageReceived(arg.payload.value.bytes,
                                      arg.payload.value.size);
    } else {
        printf("Who again? I know not that robot!\n");
    }
}


} // namespace dongle
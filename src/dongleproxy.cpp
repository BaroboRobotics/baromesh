#include "robottransport.hpp"
#include "dongleproxy.hpp"

namespace dongle {

void Proxy::registerRobotTransport(robot::Transport* rt) {
    decltype(mRobotTransports)::iterator i;
    bool success;
    std::tie(i, success) =
        mRobotTransports.insert(std::make_pair(rt->serialId(), rt));
    assert(success);
}

void Proxy::onBroadcast(Broadcast::receiveUnicast arg) {
    printf("received from %s:%d |",
            arg.source.serialId, arg.source.port);
    if (arg.payload.value.size) {
        for (size_t i = 0; i < arg.payload.value.size; ++i) {
            printf(" %02x", arg.payload.value.bytes[i]);
        }
    }
    else {
        printf(" (empty)");
    }
    printf("\n");
    if (arg.source.port == 0) {
        auto i = mRobotTransports.find(arg.source.serialId);
        if (i != mRobotTransports.end()) {
            i->second->sigMessageReceived(arg.payload.value.bytes,
                                          arg.payload.value.size);
        } else {
            printf("Who again? I know not that robot!\n");
        }
    }
    else {
        std::cerr << "Source port was not 0, but I only talk to robots!\n";
        assert(false);
    }
}


} // namespace dongle

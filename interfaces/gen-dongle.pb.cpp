#include "gen-dongle.pb.hpp"
#include "rpc/def.hpp"

RPCDEF_CPP((barobo, Dongle),
        (transmitUnicast)
        (transmitBroadcast)
        ,
        (receiveUnicast)
        (receiveRobotEvent)
        )

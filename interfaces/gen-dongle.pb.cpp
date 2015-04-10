#include "gen-dongle.pb.hpp"
#include "rpc/def.hpp"

RPCDEF_CPP((barobo, Dongle),
        (transmitUnicast)
        (transmitRadioBroadcast)
        ,
        (receiveUnicast)
        (receiveRadioBroadcast)
        )

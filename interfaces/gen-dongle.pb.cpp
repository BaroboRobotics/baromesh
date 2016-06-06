#include "gen-dongle.pb.hpp"
#include "rpc/def.hpp"

RPCDEF_CPP((barobo, Dongle),
		(getFirmwareVersion)
        (transmitUnicast)
        (transmitRadioBroadcast)
        ,
        (receiveTransmission)
        )

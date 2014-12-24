#include "gen-daemon.pb.hpp"
#include "rpc/def.hpp"

RPCDEF_CPP((barobo, Daemon),
        (resolveSerialId)
        ,
        (robotPluggedIn)
        )

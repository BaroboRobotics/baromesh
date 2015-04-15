#ifndef barobo_Daemon_INTERFACE
#define barobo_Daemon_INTERFACE

#include "rpc/def.hpp"
#include "daemon.pb.h"

RPCDEF_HPP(
        // the interface we're defining and it's version triplet
        (barobo, Daemon), (0, 1, 1),
        // all methods
        (resolveSerialId)
        (cycleDongle)
        (sendRobotPing)
        ,
        // all broadcasts
        (dongleEvent)
        (robotEvent)
        )

#endif

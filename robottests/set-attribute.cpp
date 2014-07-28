#include "robotservice.hpp"
#include "robotproxy.hpp"
#include "connectedrpcobject.hpp"

#include <iostream>

int main () {
    enum { SUCCEEDED, FAILED };
    int testResult = SUCCEEDED;

    using Attribute = rpc::Attribute<barobo::Robot>;
    using Method = rpc::MethodIn<barobo::Robot>;
    using Broadcast = rpc::Broadcast<barobo::Robot>;

    ConnectedRpcObject<RobotService, RobotProxy> widget;

    int blah = 500;

    try {
        widget.proxy().set(Attribute::ledColor{blah}).get();
        std::cout << "set(attribute) succeeded\n";
        auto result = widget.proxy().get(Attribute::ledColor()).get();
        assert(blah == result.value);
        std::cout << "get(attribute) returned: " << result.value << '\n';
    }
    catch (const rpc::Error& exc) {
        std::cout << "RPC error: " << exc.what() << '\n';
        testResult = FAILED;
    }

    return testResult;
}

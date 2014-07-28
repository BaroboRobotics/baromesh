#ifndef ROBOTIMPL_HPP_
#define ROBOTIMPL_HPP_

#include <functional>

#include "rpc/service.hpp"
#include "gen-robot.pb.hpp"

class RobotService : public rpc::Service<RobotService, barobo::Robot> {
public:
    RobotService (std::function<void(const BufferType&)> postFunc) : mPostFunc(postFunc) { }

    void post (const BufferType& buffer) {
        mPostFunc(buffer);
    }
    
    using MethodIn = rpc::MethodIn<barobo::Robot>;
    using MethodResult = rpc::MethodResult<barobo::Robot>;
    using Attribute = rpc::Attribute<barobo::Robot>;

    Attribute::ledColor onGet (Attribute::ledColor) {
        return mLedColor;
    }

    void onSet (Attribute::ledColor color) {
        mLedColor = color;
        broadcast(color);
    }

    MethodResult::dummyMethod onFire (MethodIn::dummyMethod) {
        return {};
    }

private:
    Attribute::ledColor mLedColor;
    std::function<void(const BufferType&)> mPostFunc;
    
};

#endif

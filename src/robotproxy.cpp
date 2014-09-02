#include "baromesh/robotproxy.hpp"

namespace robot {

const char* buttonToString (barobo_Robot_Button button) {
    switch (button) {
        case barobo_Robot_Button_POWER:
            return "POWER";
        case barobo_Robot_Button_A:
            return "A";
        case barobo_Robot_Button_B:
            return "B";
        default:
            return "(unknown)";
    }
}

const char* buttonStateToString (barobo_Robot_ButtonState state) {
    switch (state) {
        case barobo_Robot_ButtonState_UP:
            return "UP";
        case barobo_Robot_ButtonState_DOWN:
            return "DOWN";
        default:
            return "(unknown)";
    }
}

} // namespace robot

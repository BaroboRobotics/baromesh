#include "baromesh/linkbot.h"
#include "baromesh/linkbot.hpp"

struct Linkbot {
    Linkbot (const char* serialId) : impl(serialId)
    {
    }
    barobo::Linkbot impl;
};

Linkbot* linkbotNew(const char* serialId)
{
    
    return new Linkbot(serialId);
}

int linkbotConnect(Linkbot* l)
{
    try {
        l->impl.connect();
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}

int linkbotDisconnect(Linkbot* l)
{
    try {
        l->impl.disconnect();
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}


#define LINKBOT_C_WRAPPER_FUNC_IMPL(cpp_name, ...) \
do \
{ \
    try { \
        l->impl. cpp_name (__VA_ARGS__); \
        return 0; \
    } \
    catch (std::exception& e) { \
        fprintf(stderr, "Runtime exception: %s\n", e.what()); \
        return -1; \
    } \
} while(0)

/*
int Linkbot_drive(Linkbot*, int mask, double j1, double j2, double j3);
int Linkbot_driveTo(Linkbot*, int mask, double j1, double j2, double j3);
int Linkbot_getAccelerometer(Linkbot*, int*timestamp, double*x, double*y,
                             double*z);

int Linkbot_getFormFactor(Linkbot*, enum FormFactor*);
int Linkbot_getJointAngles(Linkbot*, int*timestamp, double*j1, double*j2,
                           double*j3);
*/

int linkbotGetFormFactor(Linkbot *l, barobo::FormFactor *form)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getFormFactor, *form);
}

int linkbotGetJointStates(Linkbot *l, int *timestamp, barobo::JointState* j1, barobo::JointState* j2,
                           barobo::JointState* j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointStates, *timestamp, *j1, *j2, *j3);
}

int linkbotMove(Linkbot* l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(move, mask, j1, j2, j3);
}

int linkbotMoveTo(Linkbot *l, int mask, double j1, double j2, double j3) {
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveTo, mask, j1, j2, j3);
}

#define SET_EVENT_CALLBACK(cbname) \
int linkbotSet##cbname(Linkbot* l, barobo::cbname cb, void* userData) \
{ \
    try { \
        l->impl.set##cbname(cb, userData); \
        return 0; \
    } \
    catch (std::exception& e) { \
        fprintf(stderr, "Runtime exception: %s\n", e.what()); \
        return -1; \
    } \
}

SET_EVENT_CALLBACK(ButtonEventCallback)
SET_EVENT_CALLBACK(EncoderEventCallback)
SET_EVENT_CALLBACK(JointEventCallback)
SET_EVENT_CALLBACK(AccelerometerEventCallback)


#undef SET_EVENT_CALLBACK

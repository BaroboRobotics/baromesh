#include "baromesh/linkbot.h"
#include "baromesh/linkbot.hpp"

struct linkbot_s {
    linkbot_s (const char* serialId) : impl(serialId)
    {
    }
    barobo::Linkbot impl;
};

linkbot_t* Linkbot_new(const char* serialId)
{
    
    return new linkbot_s(serialId);
}

int Linkbot_connect(linkbot_t* l)
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

int Linkbot_disconnect(linkbot_t* l)
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

/*
int Linkbot_drive(linkbot_t*, int mask, double j1, double j2, double j3);
int Linkbot_driveTo(linkbot_t*, int mask, double j1, double j2, double j3);
int Linkbot_getAccelerometer(linkbot_t*, int*timestamp, double*x, double*y,
                             double*z);

int Linkbot_getFormFactor(linkbot_t*, enum FormFactor*);
int Linkbot_getJointAngles(linkbot_t*, int*timestamp, double*j1, double*j2,
                           double*j3);
int Linkbot_getJointStates(linkbot_t*, enum JointState* j1, enum JointState* j2,
                           enum JointState* j3);
*/
int Linkbot_move(linkbot_t* l, int mask, double j1, double j2, double j3)
{
    try {
        l->impl.move(mask, j1, j2, j3);
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}
//int Linkbot_moveTo(linkbot_t*, int mask, double j1, double j2, double j3);

#define SET_EVENT_CALLBACK(cbname) \
int Linkbot_set##cbname(linkbot_t* l, barobo::cbname cb, void* userData) \
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

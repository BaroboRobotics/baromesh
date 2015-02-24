#include <iostream>
#include <string>
#include "baromesh/linkbot.h"
#include "baromesh/linkbot.hpp"

namespace baromesh {

struct Linkbot {
    Linkbot (const char* host, const char* service)
        : impl(std::string(host), std::string(service))
    {}

    Linkbot (const char* serialId)
        : impl(std::string(serialId))
    {}

    barobo::Linkbot impl;
};

}

using namespace baromesh;

Linkbot* linkbotFromTcpEndpoint(const char* host, const char* service)
{
    try {
        return new Linkbot(host, service);
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return nullptr;
    }
}

Linkbot* linkbotFromSerialId(const char* serialId)
{
    try {
        return new Linkbot(serialId);
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return nullptr;
    }
}

void linkbotDelete(Linkbot* l)
{
    delete l;
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

/* GETTERS */

int linkbotGetAccelerometer(Linkbot *l, int *timestamp, double *x, double *y,
                           double *z)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getAccelerometer, *timestamp, *x, *y, *z);
}


int linkbotGetFormFactor(Linkbot *l, barobo::FormFactor::Type *form)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getFormFactor, *form);
}

int linkbotGetJointAngles(Linkbot *l, int *timestamp, double *j1, double *j2, double *j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointAngles, *timestamp, *j1, *j2, *j3);
}

int linkbotGetJointSpeeds(Linkbot *l, double *s1, double *s2, double *s3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointSpeeds, *s1, *s2, *s3);
}

int linkbotGetJointStates(Linkbot *l, int *timestamp, 
                          barobo::JointState::Type* j1, 
                          barobo::JointState::Type* j2,
                          barobo::JointState::Type* j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointStates, *timestamp, *j1, *j2, *j3);
}

int linkbotGetLedColor(Linkbot *l, int *r, int *g, int *b)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getLedColor, *r, *g, *b);
}

int linkbotGetVersions (Linkbot* l, unsigned* major, unsigned* minor, unsigned* patch)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getVersions, *major, *minor, *patch);
}

int linkbotGetSerialId(Linkbot* l, char* serialId)
{
    std::string id;
    try {
        l->impl.getSerialId(id);
        snprintf(serialId, 5, "%s", id.c_str());
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}
int linkbotGetJointSafetyThresholds(Linkbot* l, int* t1, int* t2, int* t3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointSafetyThresholds, *t1, *t2, *t3);
}


/* SETTERS */

int linkbotResetEncoderRevs(Linkbot *l)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(resetEncoderRevs);
}

int linkbotSetBuzzerFrequency(Linkbot *l, float freq)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setBuzzerFrequency, freq);
}

int linkbotSetJointSpeeds(Linkbot *l, int mask, double j1, double j2, 
                          double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointSpeeds, mask, j1, j2, j3);
}

int linkbotSetJointStates(Linkbot *l, int mask,
        barobo::JointState::Type s1, double d1,
        barobo::JointState::Type s2, double d2,
        barobo::JointState::Type s3, double d3
        )
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointStates, mask, 
        s1, d1,
        s2, d2,
        s3, d3);
}

int linkbotSetJointStatesTimed(Linkbot *l, int mask,
        barobo::JointState::Type s1, double d1, double timeout1, barobo::JointState::Type end1,
        barobo::JointState::Type s2, double d2, double timeout2, barobo::JointState::Type end2,
        barobo::JointState::Type s3, double d3, double timeout3, barobo::JointState::Type end3
        )
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointStates, mask,
        s1, d1, timeout1, end1,
        s2, d2, timeout2, end2,
        s3, d3, timeout3, end3);
}

int linkbotSetLedColor(baromesh::Linkbot *l, int r, int g, int b)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setLedColor, r, g, b);
}

/* MOVEMENT */

int linkbotMoveContinuous(Linkbot *l, int mask, 
                          double d1,
                          double d2,
                          double d3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveContinuous, mask, d1, d2, d3);
}

int linkbotWriteEeprom(Linkbot *l, unsigned int address, const char *data, unsigned int size)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(writeEeprom, uint32_t(address), (uint8_t*)(data), size_t(size));
}

int linkbotDrive(Linkbot *l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(drive, mask, j1, j2, j3);
}

int linkbotDriveTo(Linkbot *l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(driveTo, mask, j1, j2, j3);
}

int linkbotMove(Linkbot *l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(move, mask, j1, j2, j3);
}

int linkbotMoveTo(Linkbot *l, int mask, double j1, double j2, double j3) {
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveTo, mask, j1, j2, j3);
}

int linkbotMotorPower(Linkbot *l, int mask, int m1, int m2, int m3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(motorPower, mask, m1, m2, m3);
}

int linkbotStop(Linkbot *l, int mask) {
    LINKBOT_C_WRAPPER_FUNC_IMPL(stop, mask);
}

/* CALLBACKS */

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
//SET_EVENT_CALLBACK(EncoderEventCallback)
SET_EVENT_CALLBACK(JointEventCallback)
SET_EVENT_CALLBACK(AccelerometerEventCallback)

int linkbotSetEncoderEventCallback(Linkbot* l, 
                                   barobo::EncoderEventCallback cb,
                                   float granularity,
                                   void* userData)
{
    try { 
        l->impl.setEncoderEventCallback(cb, granularity, userData); 
        return 0; 
    } 
    catch (std::exception& e) { 
        fprintf(stderr, "Runtime exception: %s\n", e.what()); 
        return -1; 
    } 
}


#undef SET_EVENT_CALLBACK

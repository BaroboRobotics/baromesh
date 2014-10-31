#ifndef BAROMESH_LINKBOT_H_
#define BAROMESH_LINKBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
namespace barobo {
#endif
enum ButtonState {
    UP,
    DOWN
};

enum FormFactor {
    FORMFACTOR_I,
    FORMFACTOR_L,
    FORMFACTOR_T
};

enum JointState {
    JOINT_STOP,
    JOINT_HOLD,
    JOINT_MOVING,
    JOINT_FAIL
};

enum MotorDir {
    FORWARD,
    BACKWARD,
    NEUTRAL,
    HOLD
};

typedef void (*ButtonEventCallback)(int buttonNo, barobo::ButtonState event, int timestamp, void* userData);
// EncoderEventCallback's anglePosition parameter is reported in degrees.
typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
typedef void (*JointEventCallback)(int jointNo, barobo::JointState event, int timestamp, void* userData);
typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);

#ifdef __cplusplus
} // namespace barobo
#endif

//struct Linkbot;
typedef struct Linkbot Linkbot;

Linkbot* linkbotNew(const char* serialId);

int linkbotConnect(Linkbot*);
int linkbotDisconnect(Linkbot*);

/*
int Linkbot_drive(Linkbot*, int mask, double j1, double j2, double j3);
int Linkbot_driveTo(Linkbot*, int mask, double j1, double j2, double j3);
int Linkbot_getAccelerometer(Linkbot*, int*timestamp, double*x, double*y,
                             double*z);

int Linkbot_getFormFactor(Linkbot*, enum FormFactor*);
int Linkbot_getJointAngles(Linkbot*, int*timestamp, double*j1, double*j2,
                           double*j3);
*/
/* GETTERS */

int linkbotGetFormFactor(Linkbot *l, barobo::FormFactor *form);
int linkbotGetJointStates(Linkbot*, int *timestamp, barobo::JointState* j1, 
                          barobo::JointState* j2, barobo::JointState* j3);

int linkbotMove(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMoveTo(Linkbot*, int mask, double j1, double j2, double j3);

#define SET_EVENT_CALLBACK(cbname) \
int linkbotSet##cbname(Linkbot* l, barobo::cbname cb, void* userData)
SET_EVENT_CALLBACK(ButtonEventCallback);
//SET_EVENT_CALLBACK(EncoderEventCallback);
SET_EVENT_CALLBACK(JointEventCallback);
SET_EVENT_CALLBACK(AccelerometerEventCallback);
#undef SET_EVENT_CALLBACK

int linkbotSetEncoderEventCallback(Linkbot* l, 
                                   barobo::EncoderEventCallback cb,
                                   float granularity,
                                   void* userData);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

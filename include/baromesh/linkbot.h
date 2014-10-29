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

typedef void (*ButtonEventCallback)(int buttonNo, ButtonState event, int timestamp, void* userData);
// EncoderEventCallback's anglePosition parameter is reported in degrees.
typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
typedef void (*JointEventCallback)(int jointNo, JointState event, int timestamp, void* userData);
typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);

#ifdef __cplusplus
}
#endif

struct linkbot_s;
typedef struct linkbot_s linkbot_t;

linkbot_t* Linkbot_new(const char* serialId);

int Linkbot_connect(linkbot_t*);
int Linkbot_disconnect(linkbot_t*);

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
int Linkbot_move(linkbot_t*, int mask, double j1, double j2, double j3);
//int Linkbot_moveTo(linkbot_t*, int mask, double j1, double j2, double j3);

#define SET_EVENT_CALLBACK(cbname) \
int Linkbot_set##cbname(linkbot_t* l, barobo::cbname cb, void* userData)
SET_EVENT_CALLBACK(ButtonEventCallback);
SET_EVENT_CALLBACK(EncoderEventCallback);
SET_EVENT_CALLBACK(JointEventCallback);
SET_EVENT_CALLBACK(AccelerometerEventCallback);
#undef SET_EVENT_CALLBACK

#ifdef __cplusplus
}
#endif

#endif

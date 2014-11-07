#ifndef BAROMESH_LINKBOT_H_
#define BAROMESH_LINKBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

namespace barobo {

// Since C++03 does not support enum classes, emulate them using namespaces.
// Usage example:
//     ButtonState::Type bs = ButtonState::UP;
namespace ButtonState {
    enum Type {
        UP,
        DOWN
    };
}

namespace FormFactor {
    enum Type {
        I,
        L,
        T
    };
}

// hlh: I got rid of MotorDir. Its values were FORWARD, BACKWARD, NEUTRAL, and
// HOLD. We were considering merging MotorDir with JointState, as I recall.
namespace JointState {
    enum Type {
        STOP,
        HOLD,
        MOVING,
        FAIL
    };
}

typedef void (*ButtonEventCallback)(int buttonNo, ButtonState::Type event, int timestamp, void* userData);
// EncoderEventCallback's anglePosition parameter is reported in degrees.
typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
typedef void (*JointEventCallback)(int jointNo, JointState::Type event, int timestamp, void* userData);
typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);

} // namespace barobo

//struct Linkbot;
typedef struct Linkbot Linkbot;

Linkbot* linkbotNew(const char* serialId);

/* CONNECTION */
int linkbotConnect(Linkbot*);
int linkbotDisconnect(Linkbot*);

/* GETTERS */
int linkbotGetAccelerometer(Linkbot *l, int *timestamp, double *x, double *y, 
                            double *z);
int linkbotGetFormFactor(Linkbot *l, barobo::FormFactor::Type *form);
int linkbotGetJointAngles(Linkbot *l, int* timestamp, double *j1, double *j2, 
                          double *j3);
int linkbotGetJointSpeeds(Linkbot *l, double *s1, double *s2, double *s3);
int linkbotGetJointStates(Linkbot*, int *timestamp, barobo::JointState::Type *j1, 
                          barobo::JointState::Type *j2, 
                          barobo::JointState::Type *j3);
int linkbotGetLedColor(Linkbot *l, int *r, int *g, int *b);

/* SETTERS */
int linkbotSetEncoderEventThreshold(Linkbot *l, int jointNo, double thresh);
int linkbotSetJointSpeeds(Linkbot *l, int mask, double j1, double j2, 
                          double j3);
int linkbotSetBuzzerFrequencyOn(Linkbot *l, float freq);

/* MOVEMENT */
int linkbotMoveContinuous(Linkbot *l, int mask, 
                          barobo::JointState::Type d1, 
                          barobo::JointState::Type d2, 
                          barobo::JointState::Type d3);
int linkbotDrive(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotDriveTo(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMove(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMoveTo(Linkbot*, int mask, double j1, double j2, double j3);

/* CALLBACKS */
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

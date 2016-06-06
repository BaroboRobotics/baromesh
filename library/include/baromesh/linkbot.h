#ifndef BAROMESH_LINKBOT_H_
#define BAROMESH_LINKBOT_H_

#ifdef _WIN32
#define LIBLINKBOT_EXPORT __declspec ( dllexport ) 
#else
#define LIBLINKBOT_EXPORT 
#endif


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

namespace Button {
    enum Type {
        POWER,
        A,
        B
    };
}

namespace FormFactor {
    enum Type {
        I,
        L,
        T
    };
}

// Keep in sync with enumeration in robot.proto
namespace JointState {
    enum Type {
        COAST,
        HOLD,
        MOVING,
        FAILURE = 4
    };
}

typedef void (*ButtonEventCallback)(Button::Type button, ButtonState::Type event, int timestamp, void* userData);
// EncoderEventCallback's anglePosition parameter is reported in degrees.
typedef void (*EncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
typedef void (*JointEventCallback)(int jointNo, JointState::Type event, int timestamp, void* userData);
typedef void (*AccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);
typedef void (*ConnectionTerminatedCallback)(int timestamp, void* userData);

} // namespace barobo

//struct Linkbot;
namespace baromesh {
typedef struct Linkbot Linkbot;
}

baromesh::Linkbot* linkbotFromTcpEndpoint(const char* host, const char* service);
baromesh::Linkbot* linkbotFromSerialId(const char* serialId);
void linkbotDelete(baromesh::Linkbot* l);

/* MISC */
int linkbotWriteEeprom(baromesh::Linkbot *l, unsigned int address, const char *data, unsigned int size);

/* GETTERS */
LIBLINKBOT_EXPORT int linkbotGetAccelerometer(baromesh::Linkbot *l, int *timestamp, double *x, double *y, 
                            double *z);
LIBLINKBOT_EXPORT int linkbotGetBatteryVoltage(baromesh::Linkbot *l, double*);
LIBLINKBOT_EXPORT int linkbotGetFormFactor(baromesh::Linkbot *l, barobo::FormFactor::Type *form);
LIBLINKBOT_EXPORT int linkbotGetJointAngles(baromesh::Linkbot *l, int* timestamp, double *j1, double *j2, 
                          double *j3);
LIBLINKBOT_EXPORT int linkbotGetJointSpeeds(baromesh::Linkbot *l, double *s1, double *s2, double *s3);
LIBLINKBOT_EXPORT int linkbotGetJointStates(baromesh::Linkbot*, int *timestamp, barobo::JointState::Type *j1, 
                          barobo::JointState::Type *j2, 
                          barobo::JointState::Type *j3);
LIBLINKBOT_EXPORT int linkbotGetLedColor(baromesh::Linkbot *l, int *r, int *g, int *b);
LIBLINKBOT_EXPORT int linkbotGetVersions (baromesh::Linkbot *l, unsigned*, unsigned*, unsigned*);
LIBLINKBOT_EXPORT int linkbotGetSerialId(baromesh::Linkbot* l, char* serialId);
LIBLINKBOT_EXPORT int linkbotGetJointSafetyThresholds(baromesh::Linkbot* l, int*, int*, int*);
LIBLINKBOT_EXPORT int linkbotGetJointSafetyAngles(baromesh::Linkbot* l, double*, double*, double*);

/* SETTERS */
LIBLINKBOT_EXPORT int linkbotSetAlphaI(baromesh::Linkbot *l, int mask, 
    double a1, double a2, double a3);
LIBLINKBOT_EXPORT int linkbotSetAlphaF(baromesh::Linkbot *l, int mask, 
    double a1, double a2, double a3);
LIBLINKBOT_EXPORT int linkbotResetEncoderRevs(baromesh::Linkbot *l);
LIBLINKBOT_EXPORT int linkbotSetBuzzerFrequency(baromesh::Linkbot *l, float freq);
LIBLINKBOT_EXPORT int linkbotSetJointSpeeds(baromesh::Linkbot *l, int mask, double j1, double j2, 
                          double j3);
LIBLINKBOT_EXPORT int linkbotSetJointStates(baromesh::Linkbot *l, int mask,
        barobo::JointState::Type s1, double d1,
        barobo::JointState::Type s2, double d2,
        barobo::JointState::Type s3, double d3);
LIBLINKBOT_EXPORT int linkbotSetJointStatesTimed(baromesh::Linkbot *l, int mask,
        barobo::JointState::Type s1, double d1, double timeout1, barobo::JointState::Type end1,
        barobo::JointState::Type s2, double d2, double timeout2, barobo::JointState::Type end2,
        barobo::JointState::Type s3, double d3, double timeout3, barobo::JointState::Type end3);
LIBLINKBOT_EXPORT int linkbotSetLedColor(baromesh::Linkbot *l, int r, int g, int b);
LIBLINKBOT_EXPORT int linkbotSetJointSafetyThresholds(baromesh::Linkbot *l, int mask, int t1, int t2, int t3);
LIBLINKBOT_EXPORT int linkbotSetJointSafetyAngles(baromesh::Linkbot *l, int mask, double t1, double t2, double t3);

/* MOVEMENT */
LIBLINKBOT_EXPORT int linkbotMoveAccel(baromesh::Linkbot *l, int mask, int relativeMask,
    double a0, double timeout0, barobo::JointState::Type endstate0,
    double a1, double timeout1, barobo::JointState::Type endstate1,
    double a2, double timeout2, barobo::JointState::Type endstate2);
LIBLINKBOT_EXPORT int linkbotMoveSmooth(baromesh::Linkbot *l, 
    int mask, int relativeMask, double a0, double a1, double a2);
LIBLINKBOT_EXPORT int linkbotMoveContinuous(baromesh::Linkbot *l, int mask, 
                          double d1, 
                          double d2, 
                          double d3);
LIBLINKBOT_EXPORT int linkbotDrive(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
LIBLINKBOT_EXPORT int linkbotDriveTo(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
LIBLINKBOT_EXPORT int linkbotMotorPower(baromesh::Linkbot*, int mask, int m1, int m2, int m3);
LIBLINKBOT_EXPORT int linkbotMove(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
LIBLINKBOT_EXPORT int linkbotMoveTo(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
LIBLINKBOT_EXPORT int linkbotStop(baromesh::Linkbot*, int mask);

/* CALLBACKS */
#define SET_EVENT_CALLBACK(cbname) \
int linkbotSet##cbname(baromesh::Linkbot* l, barobo::cbname cb, void* userData)
SET_EVENT_CALLBACK(ButtonEventCallback);
//SET_EVENT_CALLBACK(EncoderEventCallback);
SET_EVENT_CALLBACK(JointEventCallback);
SET_EVENT_CALLBACK(AccelerometerEventCallback);
SET_EVENT_CALLBACK(ConnectionTerminatedCallback);
#undef SET_EVENT_CALLBACK

int linkbotSetEncoderEventCallback(baromesh::Linkbot* l, 
                                   barobo::EncoderEventCallback cb,
                                   float granularity,
                                   void* userData);

#ifdef __cplusplus
} // extern "C"
#endif

#undef LINKBOT_EXPORT

#endif

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

namespace JointState {
    enum Type {
        COAST,
        HOLD,
        MOVING,
        FAILURE
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
int linkbotGetAccelerometer(baromesh::Linkbot *l, int *timestamp, double *x, double *y, 
                            double *z);
int linkbotGetBatteryVoltage(baromesh::Linkbot *l, double*);
int linkbotGetFormFactor(baromesh::Linkbot *l, barobo::FormFactor::Type *form);
int linkbotGetJointAngles(baromesh::Linkbot *l, int* timestamp, double *j1, double *j2, 
                          double *j3);
int linkbotGetJointSpeeds(baromesh::Linkbot *l, double *s1, double *s2, double *s3);
int linkbotGetJointStates(baromesh::Linkbot*, int *timestamp, barobo::JointState::Type *j1, 
                          barobo::JointState::Type *j2, 
                          barobo::JointState::Type *j3);
int linkbotGetLedColor(baromesh::Linkbot *l, int *r, int *g, int *b);
int linkbotGetVersions (baromesh::Linkbot *l, unsigned*, unsigned*, unsigned*);
int linkbotGetSerialId(baromesh::Linkbot* l, char* serialId);
int linkbotGetJointSafetyThresholds(baromesh::Linkbot* l, int*, int*, int*);
int linkbotGetJointSafetyAngles(baromesh::Linkbot* l, double*, double*, double*);

/* SETTERS */
int linkbotResetEncoderRevs(baromesh::Linkbot *l);
int linkbotSetBuzzerFrequency(baromesh::Linkbot *l, float freq);
int linkbotSetJointSpeeds(baromesh::Linkbot *l, int mask, double j1, double j2, 
                          double j3);
int linkbotSetJointStates(baromesh::Linkbot *l, int mask,
        barobo::JointState::Type s1, double d1,
        barobo::JointState::Type s2, double d2,
        barobo::JointState::Type s3, double d3);
int linkbotSetJointStatesTimed(baromesh::Linkbot *l, int mask,
        barobo::JointState::Type s1, double d1, double timeout1, barobo::JointState::Type end1,
        barobo::JointState::Type s2, double d2, double timeout2, barobo::JointState::Type end2,
        barobo::JointState::Type s3, double d3, double timeout3, barobo::JointState::Type end3);
int linkbotSetLedColor(baromesh::Linkbot *l, int r, int g, int b);
int linkbotSetJointSafetyThresholds(baromesh::Linkbot *l, int mask, int t1, int t2, int t3);
int linkbotSetJointSafetyAngles(baromesh::Linkbot *l, int mask, double t1, double t2, double t3);

/* MOVEMENT */
int linkbotMoveAccel(baromesh::Linkbot *l, int mask, int relativeMask,
    double a0, double timeout0, barobo::JointState::Type endstate0,
    double a1, double timeout1, barobo::JointState::Type endstate1,
    double a2, double timeout2, barobo::JointState::Type endstate2);
int linkbotMoveSmooth(baromesh::Linkbot *l, 
    int mask, int relativeMask, double a0, double a1, double a2);
int linkbotMoveContinuous(baromesh::Linkbot *l, int mask, 
                          double d1, 
                          double d2, 
                          double d3);
int linkbotDrive(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
int linkbotDriveTo(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMotorPower(baromesh::Linkbot*, int mask, int m1, int m2, int m3);
int linkbotMove(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMoveTo(baromesh::Linkbot*, int mask, double j1, double j2, double j3);
int linkbotStop(baromesh::Linkbot*, int mask);

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

#endif

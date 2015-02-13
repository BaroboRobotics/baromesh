#ifndef barobo_Robot_INTERFACE
#define barobo_Robot_INTERFACE

#include "rpc/def.hpp"
#include "robot.pb.h"

RPCDEF_HPP(
        // the interface we're defining and it's version triplet
        (barobo, Robot), (0, 1, 0),
        // all methods
        (getAccelerometerData)
        (getLedColor)
        (setLedColor)
        (getBuzzerFrequency)
        (setBuzzerFrequency)
        (enableAccelerometerEvent)
        (enableButtonEvent)
        (enableEncoderEvent)
        (enableJointEvent)
        (getMotorControllerSafetyThreshold)
        (getMotorControllerAlphaI)
        (getMotorControllerAlphaF)
        (getMotorControllerOmega)
        (getMotorControllerProportionalGain)
        (getMotorControllerIntegratorGain)
        (getMotorControllerDerivativeGain)
        (setMotorControllerSafetyThreshold)
        (setMotorControllerAlphaI)
        (setMotorControllerAlphaF)
        (setMotorControllerOmega)
        (setMotorControllerProportionalGain)
        (setMotorControllerIntegratorGain)
        (setMotorControllerDerivativeGain)
        (getEncoderValues)
        (getFormFactor)
        (getJointStates)
        (getButtonState)
        (getFirmwareVersion)
        (move)
        (resetEncoderRevs)
        (stop)
        (writeEeprom)
        (readEeprom)
        (writeTwi)
        (readTwi)
        (writeReadTwi)
        ,
        // all broadcasts
        (buttonEvent)
        (encoderEvent)
        (accelerometerEvent)
        (jointEvent)
        (debugMessageEvent)
        )

#endif

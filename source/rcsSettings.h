#ifndef rcsSettings_h
#define rcsSettings_h

#include "Arduino.h"
#include "rcsVersion.h"

#define STEERING_DISCRETE  0
#define STEERING_ANALOG    1
#define STEERING_SERVO    2
//#define BMW_I8 1
//#define BMW_I8_2 1
//#define LAMBO 1
//#define SERVO_CAR 3

#ifdef LAMBO
  #define ANALOG_LEFT        200
  #define ANALOG_RIGHT       824
#else
  #ifdef BMW_I8_2
    #define ANALOG_LEFT        700
    #define ANALOG_RIGHT       300
  #else
    #define ANALOG_LEFT        300
    #define ANALOG_RIGHT       700
  #endif
#endif
#define DRIVING_PWM        0
#define DRIVING_SERVO_1    1

//#define MAX_PWM_FORWARD    200
//#define MAX_PWM_REVERSE    200

typedef struct _rcsSettings_t {
  uint8_t recSize;
  uint16_t fwVersion;
  uint8_t drivingType; //= DRIVING_PWM;
  uint8_t maxPwmForward; // = 200
  uint8_t maxPwmReverse; // = 200
  uint8_t steeringType; // = STEERING_ANALOG;
  uint16_t steeringLeft; // = ANALOG_RIGHT;
  uint16_t steeringRight; // = ANALOG_LEFT;
  uint8_t i2cSlaveAddr; // = 8 
  //flags:
  uint8_t reverseDrivingDir; // = 1;  //valge i8
  uint8_t i2cSlaveEnabled; // = 0
  uint8_t btMode; // = 0 - all enabled, 1 
} rcsSettings_t;

static rcsSettings_t defSettings = {
    .recSize = sizeof(rcsSettings_t), 
    .fwVersion = FW_VERSION,
    .drivingType = DRIVING_PWM,
    //.drivingType = DRIVING_SERVO_1,
    .maxPwmForward = 200,
    .maxPwmReverse = 200,
  #ifdef SERVO_CAR
    .steeringType = STEERING_SERVO,
    .steeringLeft = 2000,
    .steeringRight = 1000,
  #else 
  #if BMW_I8
    .steeringType = STEERING_ANALOG,
  #else
    .steeringType = STEERING_DISCRETE,
  #endif
    .steeringLeft = 325,
    .steeringRight = 700,
  #endif    
    .i2cSlaveAddr = 8,
  #ifdef SERVO_CAR
    .reverseDrivingDir = 0,
  #else
  #ifdef BMW_I8
    .reverseDrivingDir = 1,
  #else
    .reverseDrivingDir = 0,
  #endif
  #endif
    .i2cSlaveEnabled = 0,
    .btMode = 0
};
    
#endif

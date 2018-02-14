#ifndef rcsCommandLib_h
#define rcsCommandLib_h

#include "rcsConfig.h"
#include "rcsSettings.h"
#include "Arduino.h"
//#include "SoftwareSerial.h"


#define BT_START_BYTE     0xFE
#define BT_COMMAND_DATA_SIZE 127
#define BT_CMD_GET_STATUS   0x01
#define BT_CMD_UPDATE_DRIVE 0x02
#define BT_CMD_SET_SWITCHES 0x03
#define BT_CMD_UPDATE_SETTINGS 0x04
#define BT_CMD_SAVE_SETTINGS 0x05

#define BT_RESPONSE_STATUS  0x01
#define BT_RESPONSE_DEBUG   0x02
#define BT_RESPONSE_SETTINGS 0x03

#define BT_REQUEST_NONE     0x00
#define BT_REQUEST_STATUS   0x01
#define BT_REQUEST_SETTINGS 0x02
 
typedef struct _btComm {
  uint8_t command;
  uint8_t dataLength;
  uint8_t data[BT_COMMAND_DATA_SIZE];
} btCommand;

typedef struct _btDataGetStatus {
  uint16_t timestamp;
  uint8_t request;
} btDataGetStatus;

typedef struct _btDataUpdateDrive {
  int16_t steering; // negative steer to left, positive to right
  int16_t driving;  // positive means forward, negative backward, absolute value is throttle position/torque
  int16_t braking;  // positive value is brake pedal position/force
  uint16_t timestamp;  // timestamp from host, send it back with status report
  uint8_t request;  // request type info
} btDataUpdateDrive;

typedef struct _btDataSetSwitches {
  uint8_t SwitchStates; //bitfield switch states
} btDataSetSwitches;

typedef struct _btDataStatus {
  uint16_t timestamp;
  uint16_t batVoltage_mV;        // bat voltage in millivolts
  uint16_t analogSteeringValue;  // ADC value in range 0-1023 (or up to 4095)
  uint16_t speed_m_s;            // 10x speed in m/s
} btResponseDataStatus;

void btInit();
btCommand * btGetNextCommand();
void btSendResponse(uint8_t command, const uint8_t *data, uint8_t data_len);
void btSendDebugStr(const char *str);

#endif

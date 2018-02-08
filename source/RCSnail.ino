
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <limits.h>
#include <EEPROM.h>
#include "rcsSettings.h"
#include "rcsCommandLib.h"
#include "rcsConfig.h"

rcsSettings_t carSetup;
int16_t _steering = 0;
int16_t _driving = 0;
int16_t _braking = 0;
typedef enum _adcMode_t {
  ADC_STEERING,
  ADC_VOLTAGE
} adcMode_t;
volatile adcMode_t _adcMode;
volatile uint16_t _steeringPos = 0;
volatile uint16_t _batVoltage = 0;

uint8_t _steeringPin1 = steering_pin1;
uint8_t _steeringPin2 = steering_pin2;
uint8_t _drive_pin1 = drive_pin1;
uint8_t _drive_pin2 = drive_pin2;
uint8_t _mosfet_pin1 = mosfet_pin1;
uint8_t _mosfet_pin2 = mosfet_pin2;

Servo *servoDriving; 
Servo *servoSteering;

uint32_t lastUpdate = 0;
bool _btConnected = false;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void settingsChanged()
{
  if (carSetup.reverseDrivingDir)
  {
     _drive_pin1 = drive_pin2;
     _drive_pin2 = drive_pin1;
     _mosfet_pin1 = mosfet_pin2;
     _mosfet_pin2 = mosfet_pin1;
  }
  else
  {
     _drive_pin1 = drive_pin1;
     _drive_pin2 = drive_pin2;
     _mosfet_pin1 = mosfet_pin1;
     _mosfet_pin2 = mosfet_pin2;    
  }
  Wire.begin(carSetup.i2cSlaveAddr);
}

void loadSettings()
{
  EEPROM.get(0, carSetup);
  if (carSetup.recSize != sizeof(carSetup) || carSetup.fwVersion != FW_VERSION)
  {
    //revert to default settings
    carSetup = defSettings;
  }
}

void saveSettings()
{
  EEPROM.put(0, carSetup);  
}

void setup()
{    
  btInit();
  loadSettings();
  settingsChanged();
  
  pinMode(steering_pin1, OUTPUT);
  pinMode(steering_pin2, OUTPUT);
    
  pinMode(_drive_pin1, OUTPUT);
  pinMode(_drive_pin2, OUTPUT);

  pinMode(_mosfet_pin1, OUTPUT);
  pinMode(_mosfet_pin2, OUTPUT);

  pinMode(connected_led, OUTPUT);
 
  pinMode(motor_drive_nsleep_pin, OUTPUT);
  digitalWrite(motor_drive_nsleep_pin, HIGH);  

  //servo1.attach(servo_pin1);
  servo2.attach(servo_pin2);
  servo3.attach(servo_pin3);
  servo4.attach(servo_pin4);
  pinMode(servo_pin4, OUTPUT);
  pinMode(servo_pin2, OUTPUT);
  digitalWrite(servo_pin2, HIGH);

  servoDriving = &servo3;
  servoSteering = &servo2;
  //servoSteering = &servo4;
  
  pinMode(servo_pin1, INPUT);
  //digitalWrite(servo_pin1, LOW);  
  //digitalWrite(servo_pin2, LOW);
    
  //pinMode(servo_pin3, OUTPUT);
  //digitalWrite(servo_pin3, LOW);
  
  Wire.onReceive(receiveI2cEvent); // register event

  //init interrupt based ADC
  //we rely on Arduino init and just enable the interrupts
  ADCSRA |= _BV(ADIE);           
  
  //using timer0 compare register to execute ADC measurements in every millisecond
  OCR0A = 0x10;
  TIMSK0 |= _BV(OCIE0A);
}

void sendBtStatus(uint16_t timestamp, uint8_t request)
{
  btResponseDataStatus response;
  response.timestamp = timestamp;
  response.batVoltage_mV = _batVoltage * 13; // 13 == 3300mV / 1023 * 133kOhm / 33kOhm --bat voltage in millivolts
  response.analogSteeringValue = _steeringPos;
  response.speed_m_s = 0;

  if (request == BT_REQUEST_STATUS)
  {
    btSendResponse(BT_RESPONSE_STATUS, (uint8_t *)&response, sizeof(response));
  }
  else if (request == BT_REQUEST_SETTINGS)
  {
    btSendResponse(BT_RESPONSE_SETTINGS, (uint8_t *)&carSetup, sizeof(carSetup));
  }
}

void handleBtCommandGetStatus(btDataGetStatus *statusRequest)
{
  sendBtStatus(statusRequest->timestamp, statusRequest->request);
}

void updateDrive(int16_t steering, int16_t driving, int16_t braking)
{
  if (DRIVING_PWM == carSetup.drivingType)
  {
    if (braking > 0)
    {
      digitalWrite(_mosfet_pin1, LOW);
      digitalWrite(_mosfet_pin2, LOW);
      delay(1);
      analogWrite(_drive_pin1, map(braking, 0, INT_MAX, 0, 255));
      analogWrite(_drive_pin2, map(braking, 0, INT_MAX, 0, 255));
    }
    else if (driving >= 0)
    {
      digitalWrite(_mosfet_pin2, LOW);  
      //force _drive_pin2 low before setting mosfet high
      delay(1);
      analogWrite(_drive_pin2, 0);
      analogWrite(_drive_pin1, map(driving, 0, INT_MAX, 0, carSetup.maxPwmForward));
      digitalWrite(_mosfet_pin1, HIGH);  
    }
    else
    {
      digitalWrite(_mosfet_pin1, LOW);
      //force _drive_pin1 low before setting mosfet high
      delay(1);
      analogWrite(_drive_pin1, 0);
      analogWrite(_drive_pin2, map(driving, -1, INT_MIN, 0, carSetup.maxPwmReverse));
      digitalWrite(_mosfet_pin2, HIGH);  
    }
  }
  else
  {
    if (braking > 0 || 0 == driving)
    {
      servoDriving->writeMicroseconds(1500);      
    }
    else
    {
      if (carSetup.reverseDrivingDir)
      {
        servoDriving->writeMicroseconds(map(driving, INT_MIN, INT_MAX, 2000, 1000));
      }
      else
      {
        servoDriving->writeMicroseconds(map(driving, INT_MIN, INT_MAX, 1000, 2000));
      }
    }
  }

  //servo1.writeMicroseconds(map(drvCmd->steering, INT_MIN, INT_MAX, 1000, 2000));
  //servo2.writeMicroseconds(map(drvCmd->steering, INT_MIN, INT_MAX, 1000, 2000));

  _steering = steering;
  _driving = driving;
  _braking = braking;
}
  
void updateSteering()
{
  if (!_btConnected)
  {
    analogWrite(_steeringPin1, 0);
    analogWrite(_steeringPin2, 0);
    return;
  }
  uint16_t target = map(_steering, INT_MIN, INT_MAX, carSetup.steeringLeft, carSetup.steeringRight);
  //#define delta 40
  #define delta 5
  //depends from bat voltage!
  //#define steeringPwm 167
  //#define steeringPwm 47
  #define steeringPwm 167
  #define steeringPwmMin 87
  #define steeringPwmMax 255

  if(carSetup.steeringType == STEERING_ANALOG) 
  {
    if (target > _steeringPos + delta)
    {
      //analogWrite(_steeringPin1, steeringPwm);
      analogWrite(_steeringPin1, constrain(map(target - _steeringPos, delta, 256, steeringPwmMin, steeringPwmMax), 0, 255));
      analogWrite(_steeringPin2, 0);
    }
    else if (target + delta < _steeringPos)
    {
      analogWrite(_steeringPin1, 0);
      analogWrite(_steeringPin2, constrain(map(_steeringPos - target, delta, 512, steeringPwmMin, steeringPwmMax), 0, 255));
      //analogWrite(_steeringPin2, steeringPwm);
    }
    else
    {
      analogWrite(_steeringPin1, 255);
      analogWrite(_steeringPin2, 255);
    }
  }
  else if (carSetup.steeringType == STEERING_DISCRETE) 
  {
    int16_t steering = _steering;
    if (carSetup.steeringLeft > carSetup.steeringRight)
      steering = -steering;
    if (steering >= 1000)
    {
      analogWrite(_steeringPin1, map(_steering, 0, INT_MAX, 0, 255));
      analogWrite(_steeringPin2, 0);
    }
    else if (steering <= 1000)
    {
      analogWrite(_steeringPin1, 0);
      analogWrite(_steeringPin2, map(_steering, -1, INT_MIN, 0, 255));
    }
    else
    {
      analogWrite(_steeringPin1, 0);
      analogWrite(_steeringPin2, 0);
    }
  }
  else
  {
    servoSteering->writeMicroseconds(target);
  }
}

void handleBtCommand(btCommand *command)
{
  _btConnected = true;
  lastUpdate = millis();
  switch (command->command) 
  {
    case BT_CMD_GET_STATUS:
      if (command->dataLength >= sizeof(btDataGetStatus))
        handleBtCommandGetStatus(((btDataGetStatus *)command->data));
      break;
    case BT_CMD_UPDATE_DRIVE:
      if (command->dataLength >= 6)
      {
        btDataUpdateDrive *updateDrv = (btDataUpdateDrive *)command->data;
        updateDrive(
            updateDrv->steering, 
            updateDrv->driving, 
            updateDrv->braking);
        if (command->dataLength >= sizeof(btDataUpdateDrive)) 
        {
          sendBtStatus(updateDrv->timestamp, updateDrv->request);
        }
      }
      break;
    case BT_CMD_SET_SWITCHES:
      if (command->dataLength >= sizeof(btDataSetSwitches)) 
      {
        btDataSetSwitches * setSwitches = (btDataSetSwitches *)command->data;
        digitalWrite(servo_pin3, setSwitches->SwitchStates & 0x04 ? LOW : HIGH);  //LED input is inverted
        ///_i2cEnabled = setSwitches->SwitchStates & 0x10 ? true : false;
      }
      break;
    case BT_CMD_UPDATE_SETTINGS:
      //btSendDebugStr("Update recv");
      if (command->dataLength = sizeof(rcsSettings_t)) 
      {
        //btSendDebugStr("Update DataLen");
        rcsSettings_t * settings = (rcsSettings_t *)command->data;
        if(settings->recSize == sizeof(rcsSettings_t) && settings->fwVersion == FW_VERSION)
        {
          //btSendDebugStr("Update changed");
          carSetup = *settings;
          settingsChanged();
        }
      }
      break;
    case BT_CMD_SAVE_SETTINGS:
      //btSendDebugStr("Save settings");
      saveSettings();
      break;
  }
}
 
void receiveI2cEvent(int howMany)
{
  btCommand command;
  uint8_t i = 0;
  while ((i < sizeof(command)) && Wire.available()) 
  {
    uint8_t c = Wire.read(); // receive byte as a character
    ((uint8_t *)&command)[i] = c;
    i++;
  }
  if (carSetup.i2cSlaveEnabled)
    handleBtCommand(&command);
}

void startADC(adcMode_t mode)
{
  uint8_t channel = 0;
  _adcMode = mode;
  
  switch (_adcMode)
  {
    case ADC_STEERING:
      channel = steering_feedback_pin;
      ///_steeringPos = analogRead(steering_feedback_pin);
      break;
    case ADC_VOLTAGE:
      channel = battery_voltage_pin;
      ///_batVoltage = analogRead(battery_voltage_pin);
      break;
  }
  if (channel >= 14)
  {
    channel -= 14;
  }
  
  //select MUX and start ADC
  ADMUX  = _BV(REFS0) | channel;
  ADCSRA |= _BV(ADIF) | _BV(ADSC);
}

// ADC completion interrupt handler
ISR(ADC_vect)
{
  switch (_adcMode)
  {
    case ADC_STEERING:
      _steeringPos = ADCL | (ADCH << 8);
      updateSteering();
      //take next measurement
      startADC(ADC_VOLTAGE);
      break;
    case ADC_VOLTAGE:
      _batVoltage = ADCL | (ADCH << 8);
      break;
  }
}

// timer0 compare interrupt, called once every millisecond
SIGNAL(TIMER0_COMPA_vect) 
{
  startADC(ADC_STEERING);
}

void loop(){  
  btCommand *command = btGetNextCommand();
  if(command) {
    if (!carSetup.i2cSlaveEnabled || command->command != BT_CMD_UPDATE_DRIVE)
    {
      handleBtCommand(command);
    }
  }    

  if (lastUpdate + 800 < millis())
  {
    _btConnected = false;
    updateDrive(_steering, 0, 0);
  }

  if (_btConnected)
  {  
    digitalWrite(connected_led, 1);
  }
  else
  {
    digitalWrite(connected_led, 0);
  }
}



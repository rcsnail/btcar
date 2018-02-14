#include "rcsCommandLib.h"
#include <avr/wdt.h>

int btEnableValue = 0;           // bt chip enable voltage after restart
//SoftwareSerial mySerial(rxPin, txPin); // RX, TX

//    wdt_enable(WDTO_15MS);

#define soft_reset()        \
do                          \
{                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
} while(0)

typedef enum _btParseState {
  BT_WAIT_START,
  BT_WAIT_LENGTH,
  BT_WAIT_COMMAND,
  BT_READ_DATA,
  BT_WAIT_CHECKSUM,
  BT_WAIT_RN4677_COMMAND,
  BT_WAIT_RN4677_DISRES,
  BT_WAIT_RN4677_END,
  BT_WAIT_ST500_BOOT_COMMAND
} btParseState;

typedef struct _btCommParseState {
  btParseState state;
  uint8_t readPos;
  uint8_t checksum;
} btCommandParseState;

btCommand _btCommand;
btCommandParseState _btCommandParseState = {BT_WAIT_START};

void rn4677_enableFastData(void);

btCommand * btGetNextCommand()
{
  uint8_t b;
  while (Serial.available()) {
    b = Serial.read();
  
    //Serial.println(b, HEX);

    switch (_btCommandParseState.state) {
      case BT_WAIT_START:
        if (b == BT_START_BYTE) {
          _btCommandParseState.state = BT_WAIT_LENGTH;
        }
        else if ('%' == b) {
          _btCommandParseState.state = BT_WAIT_RN4677_COMMAND;
        }
        else if ('0' == b) {
          _btCommandParseState.state = BT_WAIT_ST500_BOOT_COMMAND;
        }
        else {
          //for debugging
          //Serial.println(b, HEX);
        }
        break;
      case BT_WAIT_LENGTH: 
        _btCommand.dataLength = b;
        if (0 < _btCommand.dataLength && _btCommand.dataLength <= BT_COMMAND_DATA_SIZE + 1) {
          _btCommandParseState.checksum = b;
          _btCommand.dataLength--;
          _btCommandParseState.state = BT_WAIT_COMMAND;
        } else {
          _btCommandParseState.state = BT_WAIT_START;
        }
        break;
      case BT_WAIT_COMMAND:
        _btCommand.command = b;
        _btCommandParseState.checksum += b;
        _btCommandParseState.readPos = 0;
        if (_btCommand.dataLength)
          _btCommandParseState.state = BT_READ_DATA;
        else
          _btCommandParseState.state = BT_WAIT_CHECKSUM;
        break;
      case BT_READ_DATA:
        _btCommand.data[_btCommandParseState.readPos++] = b;
        _btCommandParseState.checksum += b;
        if (_btCommandParseState.readPos == _btCommand.dataLength) {
          _btCommandParseState.state = BT_WAIT_CHECKSUM;
        }
        break;
      case BT_WAIT_CHECKSUM:
        _btCommandParseState.state = BT_WAIT_START;
        
        if (b == _btCommandParseState.checksum) {
          return &_btCommand;
        } else {
          //checksum error
        }
        break;
      case BT_WAIT_RN4677_COMMAND:
        if ('D' == b || 'R' == b)
          _btCommandParseState.state = BT_WAIT_RN4677_DISRES;
        else
          _btCommandParseState.state = BT_WAIT_RN4677_END;
        break;
      case BT_WAIT_RN4677_DISRES:
        if ('%' == b) {
          _delay_ms(1000);
          //rn4677_enableFastData();
          _btCommandParseState.state = BT_WAIT_START;
        }
        break;
      case BT_WAIT_RN4677_END:
        if ('%' == b) {
          _btCommandParseState.state = BT_WAIT_START;
        }
        break;
      case BT_WAIT_ST500_BOOT_COMMAND:
        if (' ' == b) 
          soft_reset();
        _btCommandParseState.state = BT_WAIT_START;
        break;
      default:
        _btCommandParseState.state = BT_WAIT_START;
    }
  }
  return NULL;
}

void btSendResponse(uint8_t command, const uint8_t *data, uint8_t data_len) 
{
  uint8_t checksum = command + data_len + 1;
  Serial.write(BT_START_BYTE);
  Serial.write(data_len + 1);
  Serial.write(command);
  Serial.write(data, data_len);
  for(uint8_t i = 0; i < data_len; i++) {
    checksum += data[i];
  }
  Serial.write(checksum);  
}

void btSendDebugStr(const char *str)
{
  btSendResponse(BT_RESPONSE_DEBUG, (uint8_t *)str, strlen(str));
}

void btSendStr(const char *str)
{
  while(*str) {
    Serial.write(*str++);
    delay(1);
  }
}

void rn4677_sendCmd(const char *str)
{
  btSendStr(str);
  btSendStr("\r\n");
  Serial.find('>');
}

void rn4677_reset() 
{
  delay(50);
  pinMode(rn4677_sw_btn, OUTPUT);
  digitalWrite(rn4677_sw_btn, LOW);
  while(HIGH == digitalRead(rn4677_sw_btn)){
  }
  delay(50);
  digitalWrite(rn4677_sw_btn, HIGH);
  while(LOW == digitalRead(rn4677_sw_btn)){
  }
  delay(50);  
}

void rn4677_modeCommand()
{
    btSendStr("$$$\r\n");
    delay(10);
    //btSendStr("+\r\n");  //echo on
    delay(10);
}

void rn4677_modeData()
{
    btSendStr("---\r\n");  
    delay(10);
}

void rn4677_enableFastData(void)
{
    rn4677_modeCommand();
    rn4677_sendCmd("F,1");            //enable fast data mode
    rn4677_modeData();
    _delay_ms(10);
}
  
// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();

    return;
}

void btInit() 
{
  //wdt_disable();
  btEnableValue = analogRead(rn4677_sw_btn);  //0x1 - cold startup; 0x29C - reset; 0x114, 0x165 - ~1s power button off; ~2s - 0x7A
 
  pinMode(rn4677_sw_btn, OUTPUT);
  digitalWrite(rn4677_sw_btn, HIGH);
  //rn4677_reset();
  Serial.begin(38400);   

  //Serial.write("DONE");
}

#include "rn4677.h"
#include <stdint.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define rn4677_sw_btn_DDR DDRC
#define rn4677_sw_btn_PORT PORTC
#define rn4677_sw_btn_PIN PINC
#define rn4677_sw_btn_BIT PC2
#define rn4677_sw_btn_ADC _BV(MUX1)

#define rn4677_RX_DDR DDRD
#define rn4677_RX_PIN PIND
#define rn4677_RX_BIT PD0
#define rn4677_TX_DDR DDRD
#define rn4677_TX_PORT PORTD
#define rn4677_TX_BIT PD1

#define TX_HIGH rn4677_TX_PORT |= _BV(rn4677_TX_BIT)
#define TX_LOW  rn4677_TX_PORT &= ~_BV(rn4677_TX_BIT)

uint8_t tx_delay;  //66 - 38400baud;
uint8_t rx_delay;

#define TX_DELAY(X) _delay_loop_1(tx_delay + X)
#define RX_DELAY(X) _delay_loop_1(rx_delay + X)
#define RX_DELAY_HALF() _delay_loop_1(rx_delay >> 1)

void softuart_init(uint32_t baudrate)
{
  //calculate TX and RX delays: F_CPU / baudrate / 3
  tx_delay = (F_CPU + ((3 * baudrate) >> 2)) / (3 * baudrate) - 3;
  rx_delay = (F_CPU + ((3 * baudrate) >> 2)) / (3 * baudrate) - 3;

  //configure IO
  TX_HIGH;              
  rn4677_TX_DDR |= _BV(rn4677_TX_BIT);

  rn4677_RX_DDR &= ~_BV(rn4677_RX_BIT);
}

void softuart_putchar(char val)
{
  TX_LOW;  //start bit
  TX_DELAY(0);  
  for(uint8_t i = 0; i < 8; i++)
  {
    if(val & 0x01)
    {
      TX_HIGH;
    }
    else
    {
      TX_LOW;
    }
    val >>= 1;
    TX_DELAY(0);
  }

  TX_HIGH;
  TX_DELAY(0);   
}

void softuart_puts(char const *str)
{
  while(*str) {
    softuart_putchar(*str++);
  }
}

char softuart_getchar(uint32_t *timeout)
{
  if(!timeout)
  {
    //make sure that the state is high
    loop_until_bit_is_set(rn4677_RX_PIN, rn4677_RX_BIT);  
    //wait for start bit edge
    loop_until_bit_is_clear(rn4677_RX_PIN, rn4677_RX_BIT);  
  }
  else
  {
    //make sure that the state is high
    while(*timeout && !bit_is_set(rn4677_RX_PIN, rn4677_RX_BIT)){
      (*timeout)--;
    }
    //wait for start bit edge
    while(*timeout && !bit_is_clear(rn4677_RX_PIN, rn4677_RX_BIT)){
      (*timeout)--;
    }
    if(*timeout == 0)
    {
      return 0;
    }
  }

  RX_DELAY_HALF();
  uint8_t result = 0;
  for(uint8_t i = 0; i < 8; i++)
  {
    RX_DELAY(0);
    if(bit_is_set(rn4677_RX_PIN, rn4677_RX_BIT))
    {
      result = (result >> 1) | 0x80;
    }
    else
    {
      result = (result >> 1) & ~0x80;
    }
  }

  //wait for stop edge
  loop_until_bit_is_set(rn4677_RX_PIN, rn4677_RX_BIT);  
  
  return result;
}

void rn4677_sendStr(char const *str)
{
  while(*str) {
    softuart_putchar(*str++);
    _delay_ms(1);
  }
}

void rn4677_sendCmd(char const *str)
{
  rn4677_sendStr(str);
  rn4677_sendStr("\r\n");
  while(softuart_getchar(NULL) != '>')
  {
  }
}

uint8_t rn4677_find(char const *str)
{
  while(*str)
  {
    if (softuart_getchar(NULL) != *str++)
    {
      return 0;
    }
  }
  return (*str == 0);
}

void rn4677_reset(void) 
{
  // rn4677_sw_btn as output
  rn4677_sw_btn_DDR |= _BV(rn4677_sw_btn_BIT);
  // rn4677_sw_btn low
  rn4677_sw_btn_PORT &= ~_BV(rn4677_sw_btn_BIT);

  loop_until_bit_is_clear(rn4677_sw_btn_PIN, rn4677_sw_btn_BIT);
  _delay_ms(100);
  // rn4677_sw_btn high
  rn4677_sw_btn_PORT |= _BV(rn4677_sw_btn_BIT);
  loop_until_bit_is_set(rn4677_sw_btn_PIN, rn4677_sw_btn_BIT);
  //_delay_ms(10);  
}

void rn4677_enable(void) 
{
  // rn4677_sw_btn as output
  rn4677_sw_btn_DDR |= _BV(rn4677_sw_btn_BIT);
  // rn4677_sw_btn high
  rn4677_sw_btn_PORT |= _BV(rn4677_sw_btn_BIT);
  loop_until_bit_is_set(rn4677_sw_btn_PIN, rn4677_sw_btn_BIT);
}

void rn4677_modeCommand(void)
{
    rn4677_sendStr("$$$\r\n");
    _delay_ms(10);
    rn4677_sendStr("+\r\n");  //echo on
    _delay_ms(10);
}

void rn4677_modeData(void)
{
    rn4677_sendStr("---\r\n");  
    _delay_ms(10);
}

void rn4677_enableFastData(void)
{
    rn4677_modeCommand();
    rn4677_sendCmd("F,1");            //enable fast data mode
    rn4677_modeData();
    _delay_ms(10);
}

void rn4677_init(void) 
{
  uint16_t rn4677_enableLevel = 0;              //rn4677 chip enable voltage after restart

  ADMUX = _BV(REFS0) | rn4677_sw_btn_ADC;       //select AVCC as reference and ADC2 as input channel
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); //enable ADC, select ADC clock = F_CPU / 64 (i.e. 125 kHz)
  ADCSRA |= _BV(ADSC);                          //start conversion 
  loop_until_bit_is_clear(ADCSRA, ADSC);        //loop until conversion is complete 
  rn4677_enableLevel = (ADCH << 8) | ADCL;      //0x1 - cold startup; 0x29C - reset; 0x114, 0x165 - ~1s power button off; ~2s - 0x7A
  ADCSRA &= ~_BV(ADEN);                         //disable ADC 
 
  //rn4677_reset();
  rn4677_enable();
 
  softuart_init(38400);

#if 0
  softuart_putchar('0' + (rn4677_enableLevel / 1000)); 
  softuart_putchar('0' + (rn4677_enableLevel % 1000 / 100)); 
  softuart_putchar('0' + (rn4677_enableLevel % 100 / 10)); 
  softuart_putchar('0' + (rn4677_enableLevel % 10 / 1)); 
#endif

#if 0
  //softuart_putchar('O');
  uint8_t c1 = softuart_getchar();
  uint8_t c2 = softuart_getchar();
  uint8_t c3 = softuart_getchar();
  softuart_putchar('A' + (c1 & 0x0F)); 
  softuart_putchar('a' + (c1 >> 4)); 
  softuart_putchar(c2);
  softuart_putchar(c3);
  softuart_puts("test");
  return;
#endif
  
  uint8_t restore = 0;
  uint8_t forcerestore = (100 < rn4677_enableLevel) && (rn4677_enableLevel < 300); //restore rn4677 settings
  if ((rn4677_enableLevel < 100) && !rn4677_find("%REBOOT%"))
  {
    //cold start and %REBOOT% not found => wrong baudrate
    //softuart_puts("NOK");
    softuart_init(115200);
    restore = 1;
  }
  
  if (restore || forcerestore)
  {
    rn4677_modeCommand();
    rn4677_sendCmd("SF,1");            //restore factory default
    rn4677_sendCmd("S-,RCSnail");     //set device name
    rn4677_sendCmd("SU,38");          //set baudrate to 38400
    rn4677_sendCmd("SG,2");           //0 - Dual, 1 - BLE, 2 - Classic
    //rn4677_sendCmd("F,1");            //enable fast data mode
    rn4677_sendCmd("R,1");            //Reset

    /*
    rn4677_modeData();
    _delay_ms(10);
    rn4677_reset();
    */

    softuart_init(38400);
    //rn4677_sendStr("FIND\r\n");
    rn4677_find("%REBOOT%");
  }
//  else
  {
    //rn4677_enableFastData();
    rn4677_modeData();
#if 0
    while(softuart_getchar() != '%'); 
    while(softuart_getchar() != '%'); //%CONNECT,...%
    _delay_ms(1000);
    softuart_puts("connected ");
    _delay_ms(1000);
    softuart_putchar(0x5F);
    softuart_putchar(0x1F);
    softuart_putchar(0x4F);
    softuart_putchar(0x44);
    softuart_putchar(0x24);
    //softuart_putchar('O');
#endif
  }

  //rn4677_sendStr("WAIT\r\n");

  if(forcerestore)
  {
    //wait up to 20 seconds until firmware updater connects
    uint32_t timeout = 20 * (F_CPU>>4);
    char c;
    while (timeout)
    {
      c = softuart_getchar(&timeout);
      if ('0' == c) 
      {
        if (' ' == softuart_getchar(&timeout)) 
        {
          //AVR programmer protocol
          return;
        }
      } 
      else if (0xFE == c)
      {
        //btcar protocol start byte
        return;
      }
    }
    /*
    //make sure that the state is high
    while(timeout && !bit_is_set(rn4677_RX_PIN, rn4677_RX_BIT)){
      timeout--;
    }
    //wait for start bit edge
    while(count && !bit_is_clear(rn4677_RX_PIN, rn4677_RX_BIT)){
      timeout--;
    }
    */
  } 

  //rn4677_sendStr("DONE\r\n");
}

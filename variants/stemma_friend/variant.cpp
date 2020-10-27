/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"
#include "Arduino.h"
/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // D0 / D1 - UART TX & RX or I2C on SERCOM2 (shared on grove connector)
  { PORTA,  8, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel16, PWM1_CH2, TCC1_CH2, EXTERNAL_INT_NMI },  // SDA1
  { PORTA,  9, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel17, PWM1_CH3, TCC1_CH3, EXTERNAL_INT_9 },  // SCL1

  // D2 / D3 - I2C SDA & SCL on SERCOM1
  { PORTA, 16, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_0 }, // SDA
  { PORTA, 17, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH7, TCC0_CH7, EXTERNAL_INT_1 }, // SCL

  // D4, D5, D6, D7 - TFT control
  { PORTA,  3, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG), ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // TFT Reset
  { PORTA,  4, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel4, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_4 }, // TFT CS
  { PORTA,  6, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel6, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, // TFT DC
  { PORTA, 11, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel19, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_11 }, // TFT LIT

  // D8 - NeoPixel
  { PORTA, 10, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel18, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_10 }, // NeoPixel
  
  // D9 - Mode button
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Button

  // D10, D11, D12 - SPI on SERCOM 0
  { PORTA,  5, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel5, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_5 }, //SCK
  { PORTA,  7, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // MOSI
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused pin (MISO)
  
  // D13 fake pin
  { PORTA, 22, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH4, TCC0_CH4, EXTERNAL_INT_6 },   // D16 / MOSI1

  // D14 (unused DAC)
  { PORTA,  2, PIO_ANALOG, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel0, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_2 }, // A0 / D0 / DAC

  // USB pins
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;

Uart Serial1( &sercom2, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

// If wire.h isnt included, just do nothing
void WIRE_IT_HANDLER(void) __attribute__ ((weak));

void SERCOM2_Handler()
{
  Serial1.IrqHandler();
  WIRE_IT_HANDLER();
}

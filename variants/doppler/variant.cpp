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

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{


    // 0..5
    // ----------------------
    { PORTA,  13, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH7, TCC0_CH7, EXTERNAL_INT_13 },    // D0  SERCOM 2.1 SCL
    { PORTA,  12, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_12 },    // D1  SERCOM 2.0 SDA
    { PORTB,  11, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },                                      // D2  QSPI CS
    { PORTA,  14, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH2, TCC1_CH2, EXTERNAL_INT_14 },       // D3
    { PORTA,  15, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH3, TCC1_CH3, EXTERNAL_INT_15 },       // D4
    { PORTB,  10, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },                                      // D5  QSPI SCK

    // SWDIO,SWDCLK,RST Pins on Board beetween

    // 6..9
    { PORTA,  19, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH3, TCC1_CH3, EXTERNAL_INT_3 },  // D6  SHARED FPGA sercom5 SPI
    { PORTA,  20, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH4, TCC1_CH4, EXTERNAL_INT_4 },  // D7  SHARED FPGA sercom5 SPI
    { PORTA,  21, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH5, TCC1_CH5, EXTERNAL_INT_5 },  // D8  SHARED FPGA sercom5 SPI
    { PORTA,  22, PIO_SERCOM,     (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH6, TCC1_CH6, EXTERNAL_INT_6 },  // D9  SHARED FPGA sercom5 SPI


    // 10..20 - Analog pins
    // --------------------
    { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },                     // A0 DAC0
    { PORTB,  8, PIO_SERCOM_ALT, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },                     // A1
    { PORTB,  9, PIO_SERCOM_ALT, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },                     // A2
    { PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },                     // A3
    { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },                     // A4 DAC1
    { PORTA,  6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },                     // A5
    { PORTA,  7, PIO_ANALOG, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG), ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  // A6
    { PORTA,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },                   // A7  QSPI D0
    { PORTA,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },                     // A8  QSPI D1
    { PORTA, 10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },                   // A9  QSPI D2
    { PORTA, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },                   // A10 QSPI D3

    // ________________ Internal Connections ___________________

    // 21..26 Internal FPGA Connection
    { PORTB,   3, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },                                       // 21 ICE40_CLK
    { PORTB,  23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },                                     // 22 ICE40_MOSI
    { PORTB,   2, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },                                       // 23 ICE40_MISO
    { PORTB,  22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },                                    // 24 ICE40_CS
    { PORTA,  16, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_0 },     // 25 ICE40_CDONE
    { PORTA,  17, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_1 },     // 26 ICE40_RESET

    // 27..29 - USB
    // --------------------
    { PORTA, 27, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // USB Host enable
    { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // USB/DM
    { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // USB/DP

    // 30 (AREF)
    { PORTA,  3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // DAC/VREFP

    // 31..32 Crystal 32K
    { PORTA,  1, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // SCK  SERCOM 1.1
    { PORTA,  0, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // MOSI SERCOM 1.0

    // 33 LED
    { PORTA, 23, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH7, TCC1_CH7, EXTERNAL_INT_7 },

    // ----------------------
    // 34 .. 35 Alternate use of A0 and A4 (DAC output)
    { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT[0]
    { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // DAC/VOUT[1]

    // 36..37 not connected
    { PORTA,  7, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH2, TCC1_CH2, EXTERNAL_INT_2 }, // D7
    { PORTA, 18, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM1_CH2, TCC1_CH2, EXTERNAL_INT_2 }, // D7

} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;


Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM3_0_Handler()    {   Serial1.IrqHandler();         }
void SERCOM3_1_Handler()    {   Serial1.IrqHandler();         }
void SERCOM3_2_Handler()    {   Serial1.IrqHandler();         }
void SERCOM3_3_Handler()    {   Serial1.IrqHandler();         }
/*********************************************************************************************************
        some useful functions
*********************************************************************************************************/


// init 2 DACS
void dacInit(){

    // Do this in the Sketch first!
    //pinMode(34,OUTPUT);
    //pinMode(35,OUTPUT);

    while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

    GCLK->PCHCTRL[GCM_DAC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK4 );  // use 48MHz clock (100MHz max for DAC) from GCLK4, which was setup in startup.c
    while ( (GCLK->PCHCTRL[GCM_DAC].reg & GCLK_PCHCTRL_CHEN) == 0 );      // wait for sync
    while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );

    // SUPC->VREF.reg |= SUPC_VREF_SEL(mode - AR_INTREF_1V0);

    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPU;

    //ADC0->REFCTRL.bit.REFSEL = AR_EXTERNAL_REFB;  // or AR_EXTERNAL_REFB
    //ADC1->REFCTRL.bit.REFSEL = AR_EXTERNAL_REFB;

    DAC->CTRLA.bit.ENABLE = 0x00;
    DAC->DACCTRL[0].bit.ENABLE = 0x01;

    // delay(1);
    for (int i=0;i<60000;i++) __asm("nop");

    DAC->DACCTRL[1].bit.ENABLE = 0x01;
    DAC->CTRLA.bit.ENABLE = 0x01;

    //while (DAC->SYNCBUSY.bit.ENABLE == 1) {}
    while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );

}

void dacWrite(uint16_t  left ,uint16_t  right){
    DAC->DATA[0].reg = left >> 4;
    while (DAC->SYNCBUSY.bit.DATA0);
    DAC->DATA[1].reg = right >> 4;
    while (DAC->SYNCBUSY.bit.DATA1);
}

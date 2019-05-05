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

/////////////////////////////////////////////////////////////////////////////////////////////
///// MASTER SAMD PINS //////////////////////////////////////////////////////////////////////

    // MASTER SAM PORT A
    { PORTA,   0, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },     // --- (Quartz)
    { PORTA,   1, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },     // --- (Quartz)
    { PORTA,   2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },            // --- (DAC OUT)
    { PORTA,   3, PIO_ANALOG,  PIN_ATTR_ANALOG,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },        // AREF
    { PORTA,   4, PIO_SERCOM_ALT,         EXTERNAL_INT_4 },        // BUS_I2C_SDA    /SERCOM_ALT_0_P0
    { PORTA,   5, PIO_SERCOM_ALT,         EXTERNAL_INT_5 },        // BUS_I2C_SCL    /SERCOM_ALT_0_P1
    { PORTA,   6, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },        // ---
    { PORTA,   7, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },        // ---

    { PORTA,   8, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },       // FLASH_SD QSPI_D0   /SERCOM_ALT
    { PORTA,   9, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9  },        // FLASH_SD QSPI_D1   /SERCOM_ALT
    { PORTA,  10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },        // FLASH_SD QSPI_D2   /SERCOM_ALT
    { PORTA,  11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },        // FLASH_SD QSPI_D3   /SERCOM_ALT

 //   AUS Doppler Variant: was ist richtig, oben oder unten?
 //   { PORTA,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },           // FLASH_SD QSPI_D0   /SERCOM_ALT  /or EXTERNAL_INT_NMI
 //   { PORTA,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },              // FLASH_SD QSPI_D1   /SERCOM_ALT
 //   { PORTA, 10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },            // FLASH_SD QSPI_D2   /SERCOM_ALT
 //   { PORTA, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },            // FLASH_SD QSPI_D2   /SERCOM_ALT

    { PORTA,  12, PIO_SERCOM_ALT,        },        // BUS SPI > Compute Module MOSI     /SERCOM_2_P0 /SERCOM_ALT_4_P1
    { PORTA,  13, PIO_SERCOM_ALT,        },        // BUS SPI > Compute Module SKC      /SERCOM_2_P1 /SERCOM_ALT_4_P0
    { PORTA,  14, PIO_SERCOM_ALT,        },        // BUS SPI > Compute Module CS       /SERCOM_2_P2 /SERCOM_ALT_4_P2
    { PORTA,  15, PIO_SERCOM_ALT,        },        // BUS SPI > Compute Module MISO     /SERCOM_2_P3 /SERCOM_ALT_4_P3
    { PORTA,  16, PIO_SERCOM,        },        // BUS SPI > ICE 40 MOSI             /SERCOM_1_P0 /SERCOM_ALT_3_P1
    { PORTA,  17, PIO_SERCOM,        },        // BUS SPI > ICE 40 SCK              /SERCOM_1_P1 /SERCOM_ALT_3_P0
    { PORTA,  18, PIO_SERCOM,        },        // BUS SPI > ICE 40 CS               /SERCOM_1_P2 /SERCOM_ALT_3_P2
    { PORTA,  19, PIO_SERCOM,        },        // BUS SPI > ICE 40 MISO             /SERCOM_1_P3 /SERCOM_ALT_3_P3
    { PORTA,  20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },        // ---
    { PORTA,  21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },        // ---
    { PORTA,  22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },        // ---
    { PORTA,  23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },        // ---
    { PORTA,  24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },        // USB_C IN DATA D_N
    { PORTA,  25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },        // USB_C IN DATA D_P
    { PORTA,  26, PIO_DIGITAL,        },        // ---
    { PORTA,  27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },    // ---
    { PORTA,  28, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },     // --- 
    { PORTA,  29, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },     // ---
    { PORTA,  30, PIO_DIGITAL,        },        // SWCLK
    { PORTA,  31, PIO_DIGITAL,        },        // SWDIO

    // MASTER SAM PORT B
    { PORTB,   0, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },        // ---
    { PORTB,   1, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },        // ---
    { PORTB,   2, PIO_DIGITAL,        EXTERNAL_INT_2 },        // USB_C PD SDA
    { PORTB,   3, PIO_DIGITAL,        },        // USB_C PD SCL
    { PORTB,   4, PIO_DIGITAL,        },        // USB_C PD INT
    { PORTB,   5, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5  },        // ---
    { PORTB,   6, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6  },        // ---
    { PORTB,   7, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7  },        // ---
    { PORTB,   8, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8  },        // ---
    { PORTB,   9, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9  },        // ---
    { PORTB,  10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },        // FLASH_SD QSPI_SCK
    { PORTB,  11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },        // FLASH_SD QSPI_CS
    { PORTB,  12, PIO_DIGITAL,         EXTERNAL_INT_12 },        // FLASH_SD QSPI_DE
    { PORTB,  13, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },        // ---
    { PORTB,  14, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },        // ---
    { PORTB,  15, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },        // ---
    { PORTB,  16, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0  },        // ---
    { PORTB,  17, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1  },        // BUILTIN LED
    { PORTB,  18, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2  },        // ---
    { PORTB,  19, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3  },        // ---
    { PORTB,  20, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4  },        // ---
    { PORTB,  21, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5  },        // ---
    { PORTB,  22, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6  },        // ---
    { PORTB,  23, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7  },        // ---
    { PORTB,  24, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8  },        // ---
    { PORTB,  25, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9  },        // ---
    { PORTB,  26, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },        // ---
    { PORTB,  27, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },        // ---
    { PORTB,  28, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },        // ---
    { PORTB,  29, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },        // ---
    { PORTB,  30, PIO_DIGITAL,        },        // SWO
    { PORTB,  31, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },        // ---


/////////////////////////////////////////////////////////////////////////////////////////////
///// SLAVE SAMD PINS ///////////////////////////////////////////////////////////////////////

    // SLAVe SAM PORT A
    { PORTA,   0, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },     // --- (Quartz)
    { PORTA,   1, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },     // --- (Quartz)
    { PORTA,   2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },            // --- (DAC OUT)
    { PORTA,   3, PIO_ANALOG,  PIN_ATTR_ANALOG,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },        // AREF
    { PORTA,   4, PIO_SERCOM_ALT,         EXTERNAL_INT_4 },        // BUS_I2C_SDA    /SERCOM_ALT_0_P0
    { PORTA,   5, PIO_SERCOM_ALT,         EXTERNAL_INT_5 },        // BUS_I2C_SCL    /SERCOM_ALT_0_P1
    { PORTA,   6, PIO_DIGITAL,        },        // PA06_TOUCH
    { PORTA,   7, PIO_DIGITAL,        },        // PA07_TOUCH
    { PORTA,   8, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },     // SYNC_RX
    { PORTA,   9, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },        // SYNC_TX
    { PORTA,  10, PIO_DIGITAL,        },        // PA10_TOUCH
    { PORTA,  11, PIO_DIGITAL,        },        // PA11_TOUCH
    { PORTA,  12, PIO_SERCOM,        },        // MIDI_O RX     /SERCOM_2_P0 /SERCOM_ALT_4_P1
    { PORTA,  13, PIO_SERCOM,        },        // MIDI_0 TX     /SERCOM_2_P1 /SERCOM_ALT_4_P0
    { PORTA,  14, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },        // ---
    { PORTA,  15, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },        // ---
    { PORTA,  16, PIO_SERCOM,        },        // SPI_OLED MOSI   /SERCOM_1_P0 /SERCOM_ALT_3_P1
    { PORTA,  17, PIO_SERCOM,        },        // SPI_OLED SCK    /SERCOM_1_P1 /SERCOM_ALT_3_P0
    { PORTA,  18, PIO_SERCOM,        },        // SPI_OLED CS     /SERCOM_1_P2 /SERCOM_ALT_3_P2
    { PORTA,  19, PIO_SERCOM,        },        // SPI_OLED MISO   /SERCOM_1_P3 /SERCOM_ALT_3_P3
    { PORTA,  20, PIO_DIGITAL,        },        // PA20_TOUCH
    { PORTA,  21, PIO_DIGITAL,        },        // PA21_TOUCH
    { PORTA,  22, PIO_SERCOM,        EXTERNAL_INT_6 },        // I2C_2_SDA   /SERCOM_3_P0 /SERCOM_ALT_5_P1
    { PORTA,  23, PIO_SERCOM,        EXTERNAL_INT_7 },        // I2C_2_SCL   /SERCOM_3_P1 /SERCOM_ALT_5_P0
    { PORTA,  24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },               // USB_D_N
    { PORTA,  25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },               // USB_D_P
    { PORTA,  26, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },        // ---
    { PORTA,  27, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },          // ---
    { PORTA,  28, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },        // ---
    { PORTA,  29, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },        // ---
    { PORTA,  30, PIO_DIGITAL,        EXTERNAL_INT_14 },        // SWCLK
    { PORTA,  31, PIO_DIGITAL,        EXTERNAL_INT_15 },        // SWDIO

    // SLAVE SAM PORT B
    { PORTB,   0, PIO_DIGITAL,         EXTERNAL_INT_0 },        // PB00_TOUCH
    { PORTB,   1, PIO_DIGITAL,         EXTERNAL_INT_1 },        // PB01_TOUCH
    { PORTB,   2, PIO_SERCOM,         EXTERNAL_INT_2 },        // MIDI_1_TX
    { PORTB,   3, PIO_SERCOM,        EXTERNAL_INT_3 },        // MIDI_1_RX
    { PORTB,   4, PIO_DIGITAL,        EXTERNAL_INT_4 },        // PB04_TOUCH
    { PORTB,   5, PIO_DIGITAL,        EXTERNAL_INT_5 },        // PB05_TOUCH
    { PORTB,   6, PIO_DIGITAL,        EXTERNAL_INT_6 },        // PB06_TOUCH
    { PORTB,   7, PIO_DIGITAL,        EXTERNAL_INT_7 },        // PB07_TOUCH
    { PORTB,   8, PIO_DIGITAL,        EXTERNAL_INT_8 },        // PB08_TOUCH
    { PORTB,   9, PIO_DIGITAL,        EXTERNAL_INT_9 },        // PB09_TOUCH. -- TOUCHSTRIP TCA9555 INT PIN
    { PORTB,  10, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },        // ---
    { PORTB,  11, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },        // ---
    { PORTB,  12, PIO_DIGITAL,         EXTERNAL_INT_12 },        // PB12_TOUCH
    { PORTB,  13, PIO_DIGITAL,         EXTERNAL_INT_13 },        // PB13_TOUCH
    { PORTB,  14, PIO_DIGITAL,         EXTERNAL_INT_14 },        // PB14_TOUCH
    { PORTB,  15, PIO_DIGITAL,         EXTERNAL_INT_15 },        // PB15_TOUCH
    { PORTB,  16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0  },     // OLED_RESET
    { PORTB,  17, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1  },        // BUILTIN LED
    { PORTB,  18, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2  },        // ---
    { PORTB,  19, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3  },        // ---
    { PORTB,  20, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4  },        // ---
    { PORTB,  21, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5  },        // ---
    { PORTB,  22, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6  },        // ---
    { PORTB,  23, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7  },        // ---
    { PORTB,  24, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8  },        // ---
    { PORTB,  25, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9  },        // ---
    { PORTB,  26, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },        // ---
    { PORTB,  27, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },        // ---
    { PORTB,  28, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },        // ---
    { PORTB,  29, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },        // ---
    { PORTB,  30, PIO_DIGITAL,        EXTERNAL_INT_14 },        // SWO
    { PORTB,  31, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },        // ---

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

} ;

//const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5 } ;
//const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID } ;
const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1,TCC2 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

//Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

//void SERCOM3_0_Handler()    {   Serial1.IrqHandler();         }
//void SERCOM3_1_Handler()    {   Serial1.IrqHandler();         }
//void SERCOM3_2_Handler()    {   Serial1.IrqHandler();         }
//void SERCOM3_3_Handler()    {   Serial1.IrqHandler();         }

/*********************************************************************************************************
        some useful functions
*********************************************************************************************************/

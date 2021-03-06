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

#ifndef _VARIANT_COMPOSER_PRO_
#define _VARIANT_COMPOSER_PRO_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (120000000ul)

#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (128u)
#define NUM_DIGITAL_PINS     (20u)      // TO DO !! set to correct value
#define NUM_ANALOG_INPUTS    (0u)       // TO DO !! set to correct value
#define NUM_ANALOG_OUTPUTS   (1u)       // TO DO !! set to correct value
//#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + PIN_A0 : -1)     // TO DO !! set to correct value

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

    
/////////////////////////////////////////////////////////////////////////////////////////////
///// MASTER SAMD PINS //////////////////////////////////////////////////////////////////////
    
#define M_PA00      0
#define M_PA01      1
#define M_PA02      2
#define M_PA03      3           // AREF
#define M_PA04      4           // BUS_I2C_1_SDA 
#define M_PA05      5           // BUS_I2C_1_SCL 
#define M_PA06      6
#define M_PA07      7           // WS28 RGB LEDs
#define M_PA08      8           // FLASH_SD QSPI_D0     // MOSI for FRAM+SD
#define M_PA09      9           // FLASH_SD QSPI_D1     // SCK  for FRAM+SD
#define M_PA10     10           // FLASH_SD QSPI_D2     // CS/SS for FRAM
#define M_PA11     11           // FLASH_SD QSPI_D3     // MISO for FRAM+SD
#define M_PA12     12           // BUS SPI > Compute Module MOSI
#define M_PA13     13           // BUS SPI > Compute Module SKC
#define M_PA14     14           // BUS SPI > Compute Module CS
#define M_PA15     15           // BUS SPI > Compute Module MISO
#define M_PA16     16           // BUS SPI > ICE 40 MOSI 
#define M_PA17     17           // BUS SPI > ICE 40 SCK
#define M_PA18     18           // BUS SPI > ICE 40 CS
#define M_PA19     19           // BUS SPI > ICE 40 MISO
#define M_PA20     20
#define M_PA21     21
#define M_PA22     22
#define M_PA23     23           
#define M_PA24     24           // USB_C IN DATA D_N
#define M_PA25     25           // USB_C IN DATA D_P
#define M_PA26     26
#define M_PA27     27           // RESET SLAVE SAMD 
#define M_PA28     28
#define M_PA29     29
#define M_PA30     30           // SWCLK
#define M_PA31     31           // SWDIO

#define M_PB00     ( 0+32)      // BUS_I2C_2_INT  MatrixTCA955 INT PIN
#define M_PB01     ( 1+32)      // BUS_I2C_2_INT  USB_C PD INT
#define M_PB02     ( 2+32)      // BUS_I2C_2_SDA  USB_C PD + MatrixTCA955
#define M_PB03     ( 3+32)      // BUS_I2C_2_SCL  USB_C PD + MatrixTCA955
#define M_PB04     ( 4+32)
#define M_PB05     ( 5+32)
#define M_PB06     ( 6+32)      // FOOTSWITCH 1
#define M_PB07     ( 7+32)      // FOOTSWITCH 2
#define M_PB08     ( 8+32)      // BUS SPI > ICE 40 CDONE
#define M_PB09     ( 9+32)      // BUS SPI > ICE 40 RESET
#define M_PB10     (10+32)      // FLASH_SD QSPI_SCK    // FRAM
#define M_PB11     (11+32)      // FLASH_SD QSPI_CS     // CS SD
#define M_PB12     (12+32)      // FLASH_SD QSPI_DE     // SD enable
#define M_PB13     (13+32)
#define M_PB14     (14+32)
#define M_PB15     (15+32)
#define M_PB16     (16+32)
#define M_PB17     (17+32)      // BUILTIN LED
#define M_PB18     (18+32)
#define M_PB19     (19+32)
#define M_PB20     (20+32)
#define M_PB21     (21+32)
#define M_PB22     (22+32)      // SWDIO from SLAVE SAMD
#define M_PB23     (23+32)      // SWCLK from SLAVE SAMD
#define M_PB24     (24+32)
#define M_PB25     (25+32)
#define M_PB26     (26+32)
#define M_PB27     (27+32)
#define M_PB28     (28+32)
#define M_PB29     (29+32)
#define M_PB30     (30+32)      // SWO
#define M_PB31     (31+32)


/////////////////////////////////////////////////////////////////////////////////////////////
///// SLAVe SAMD PINS ///////////////////////////////////////////////////////////////////////

#define S_PA00     ( 0+64)
#define S_PA01     ( 1+64)
#define S_PA02     ( 2+64)      // DAC OUT
#define S_PA03     ( 3+64)      // AREF
#define S_PA04     ( 4+64)      // BUS_I2C_1_SDA
#define S_PA05     ( 5+64)      // BUS_I2C_1_SCL
#define S_PA06     ( 6+64)      // PA06_TOUCH     X2
#define S_PA07     ( 7+64)      // PA07_TOUCH     X1
#define S_PA08     ( 8+64)      // SYNC_IN_1
#define S_PA09     ( 9+64)      // SYNC_IN_2
#define S_PA10     (10+64)      // PA10_TOUCH     X0
#define S_PA11     (11+64)      // PA11_TOUCH     Y13
#define S_PA12     (12+64)      // MIDI_O RX
#define S_PA13     (13+64)      // MIDI_O TX
#define S_PA14     (14+64)
#define S_PA15     (15+64)      // OLED DISPLAY D/C
#define S_PA16     (16+64)      // SPI_OLED MOSI
#define S_PA17     (17+64)      // SPI_OLED SCK
#define S_PA18     (18+64)      // SPI_OLED CS
#define S_PA19     (19+64)      // SPI_OLED MISO
#define S_PA20     (20+64)      // PA20_TOUCH     Y8
#define S_PA21     (21+64)      // PA21_TOUCH     Y7
#define S_PA22     (22+64)      // I2C_2_SDA 
#define S_PA23     (23+64)      // I2C_2_SCL 
#define S_PA24     (24+64)      // USB_D_N
#define S_PA25     (25+64)      // USB_D_P
#define S_PA26     (26+64)
#define S_PA27     (27+64)
#define S_PA28     (28+64)
#define S_PA29     (29+64)
#define S_PA30     (30+64)      // SWCLK
#define S_PA31     (31+64)      // SWDIO

#define S_PB00     ( 0+96)      // PB00_TOUCH     Y6
#define S_PB01     ( 1+96)      // PB01_TOUCH     Y5
#define S_PB02     ( 2+96)      // MIDI_1_TX
#define S_PB03     ( 3+96)      // MIDI_1_RX
#define S_PB04     ( 4+96)      // PB04_TOUCH     Y4
#define S_PB05     ( 5+96)      // PB05_TOUCH     Y3
#define S_PB06     ( 6+96)      // PB06_TOUCH     Y2
#define S_PB07     ( 7+96)      // PB07_TOUCH     Y1
#define S_PB08     ( 8+96)      // PB08_TOUCH     Y0
#define S_PB09     ( 9+96)      // TCA9555 TOUCHStrip LEDs INT Pin --> NOT NEEDED
#define S_PB10     (10+96)
#define S_PB11     (11+96)
#define S_PB12     (12+96)      // PB12_TOUCH     Y12
#define S_PB13     (13+96)      // PB13_TOUCH     Y11
#define S_PB14     (14+96)      // PB14_TOUCH     Y10
#define S_PB15     (15+96)      // PB15_TOUCH     Y9
#define S_PB16     (16+96)      // OLED_RESET
#define S_PB17     (17+96)      // BUILTIN LED
#define S_PB18     (18+96)
#define S_PB19     (19+96)
#define S_PB20     (20+96)
#define S_PB21     (21+96)
#define S_PB22     (22+96)      // SYNC_OUT_1
#define S_PB23     (23+96)      // SYNC_OUT_2
#define S_PB24     (24+96)
#define S_PB25     (25+96)
#define S_PB26     (26+96)
#define S_PB27     (27+96)
#define S_PB28     (28+96)
#define S_PB29     (29+96)
#define S_PB30     (30+96)      // SWO
#define S_PB31     (31+96)


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


// ONLY AVAILABLE ON MASTER ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// QSPI Pins
#define PIN_QSPI_SCK    M_PB10    // FLASH_SD QSPI_SCK    // WP for FRAM enable
#define PIN_QSPI_CS     M_PB11    // FLASH_SD QSPI_CS     // CS for SD
#define PIN_QSPI_IO0    M_PA08    // FLASH_SD QSPI_D0     // MOSI  for FRAM+SD
#define PIN_QSPI_IO1    M_PA09    // FLASH_SD QSPI_D1     // SCK   for FRAM+SD
#define PIN_QSPI_IO2    M_PA10    // FLASH_SD QSPI_D2     // CS/SS for FRAM
#define PIN_QSPI_IO3    M_PA11    // FLASH_SD QSPI_D3     // MISO  for FRAM+SD
#define PIN_QSPI_DE     M_PB12     // FLASH_SD QSPI_DE.   // SD DE for SD
#define VARIANT_QSPI_BAUD_DEFAULT 5000000   //TODO: meaningful value for this


#define PIN_SD_MOSI     M_PA08
#define PIN_SD_SCK      M_PA09
#define PIN_SD_CS       M_PB11
#define PIN_SD_MISO     M_PA11
#define PIN_SD_DE       M_PB12      // SD CARD Device enable

#define PIN_FRAM_MOSI   M_PA08
#define PIN_FRAM_SCK    M_PA09
#define PIN_FRAM_CS     M_PA10
#define PIN_FRAM_MISO   M_PA11
#define PIN_FRAM_DE     M_PB10      // FRAM Device enable




// SPI to Compute Module Pins
#define COMPUTEMODULE_SPI_MOSI_M    M_PA12          // BUS SPI > Compute Module MOSI
#define COMPUTEMODULE_SPI_SCK_M     M_PA13          // BUS SPI > Compute Module SKC
#define COMPUTEMODULE_SPI_CS_M      M_PA14          // BUS SPI > Compute Module CS
#define COMPUTEMODULE_SPI_MISO_M    M_PA15          // BUS SPI > Compute Module MISO


// FPGA internal SPI Bus    
#define ICE_MOSI   M_PA16   // BUS SPI > ICE 40 MOSI 
#define ICE_CLK    M_PA17   // BUS SPI > ICE 40 SCK
#define ICE_CS     M_PA18   // BUS SPI > ICE 40 CS
#define ICE_MISO   M_PA19   // BUS SPI > ICE 40 MISO
#define ICE_CDONE  M_PB08   // BUS SPI > ICE 40 CDONE
#define ICE_RESET  M_PB09   // BUS SPI > ICE 40 RESET
#define ICE_CRESET  M_PB09   // BUS SPI > ICE 40 ICE_RESET. for compatibility with doppler sketches where it is called CRESET


// WS28 RGB LEDs
#define WS28_PIN    M_PA07      // WS28 RGB LEDs


// SLAVE PROGRAMMER (from Master SAMD)
#define RESET_SLAVE_SAMD_M   M_PA27     // RESET SLAVE SAMD 
#define SWDIO_SLAVE_SAMD_M   M_PB22     // SWDIO from SLAVE SAMD
#define SWCLK_SLAVE_SAMD_M   M_PB23     // SWCLK from SLAVE SAMD


// FOOTSWITCH PINS
#define FOOTSWITCH_1  M_PB06     // FOOTSWITCH 1
#define FOOTSWITCH_2  M_PB07     // FOOTSWITCH 2

// Wire 1 
#define BUS_I2C_1_SDA_M   M_PA04    // BUS_I2C_1_SDA  MASTER
#define BUS_I2C_1_SCL_M   M_PA05    // BUS_I2C_1_SCL  MASTER


// BUS_I2C_2 to TCA9555 on MatrixPCB and Power Distribution Chip for USB C
#define BUS_I2C_2_SDA_M       M_PB02     // BUS_I2C_2_SDA  USB_C PD + MatrixTCA955
#define BUS_I2C_2_SCL_M       M_PB03     // BUS_I2C_2_SCL  USB_C PD + MatrixTCA955
#define BUS_I2C_2_INT_TCA_M   M_PB00     // BUS_I2C_2_INT  MatrixTCA955
#define BUS_I2C_2_INT_USB_M   M_PB01     // BUS_I2C_2_INT  USB_C PD



// ONLY AVAILABLE ON SLAVE ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// ANANLOG MIDI I/O
#define MIDI_1_RX       S_PA12       // MIDI_O RX
#define MIDI_1_TX       S_PA13       // MIDI_O TX
#define MIDI_2_RX       S_PB03       // MIDI_1_RX
#define MIDI_2_TX       S_PB02       // MIDI_1_TX


// ANANLOG SYNC I/O
#define SYNC_IN_1       S_PA08      // SYNC_IN_1
#define SYNC_IN_2       S_PA09      // SYNC_IN_2
#define SYNC_OUT_1      S_PB22      // SYNC_OUT_1
#define SYNC_OUT_2      S_PB23      // SYNC_OUT_2


// TOUCHSTRIP TOUCH PINS
#define TOUCHPIN_X0     S_PA10     // PA10_TOUCH     X0
#define TOUCHPIN_X1     S_PA07     // PA07_TOUCH     X1
#define TOUCHPIN_X2     S_PA06     // PA06_TOUCH     X2
#define TOUCHPIN_Y0     S_PB08     // PB08_TOUCH     Y0
#define TOUCHPIN_Y1     S_PB07     // PB07_TOUCH     Y1
#define TOUCHPIN_Y2     S_PB06     // PB06_TOUCH     Y2
#define TOUCHPIN_Y3     S_PB05     // PB05_TOUCH     Y3
#define TOUCHPIN_Y4     S_PB04     // PB04_TOUCH     Y4
#define TOUCHPIN_Y5     S_PB01     // PB01_TOUCH     Y5
#define TOUCHPIN_Y6     S_PB00     // PB00_TOUCH     Y6
#define TOUCHPIN_Y7     S_PA21     // PA21_TOUCH     Y7
#define TOUCHPIN_Y8     S_PA20     // PA20_TOUCH     Y8
#define TOUCHPIN_Y9     S_PB15     // PB15_TOUCH     Y9
#define TOUCHPIN_Y10    S_PB14     // PB14_TOUCH     Y10
#define TOUCHPIN_Y11    S_PB13     // PB13_TOUCH     Y11
#define TOUCHPIN_Y12    S_PB12     // PB12_TOUCH     Y12
#define TOUCHPIN_Y13    S_PA11     // PA11_TOUCH     Y13


// OLED SPI BUS
#define OLED_SPI_DC     S_PA15  // OLED DISPLAY D/C
#define OLED_SPI_MOSI   S_PA16  // SPI_OLED MOSI
#define OLED_SPI_SCK    S_PA17  // SPI_OLED SCK
#define OLED_SPI_CS     S_PA18  // SPI_OLED CS
#define OLED_SPI_MISO   S_PA19  // SPI_OLED MISO
#define OLED_SPI_RESET  S_PB16  // OLED_RESET


// Wire 1
#define BUS_I2C_1_SDA_S   S_PA04    // BUS_I2C_1_SDA  Slave
#define BUS_I2C_1_SCL_S   S_PA05    // BUS_I2C_1_SCL  Slave

// Wire 2
#define BUS_I2C_2_SDA_S   S_PA22    // I2C_2_SDA    to Touchstrip TCA9555
#define BUS_I2C_2_SCL_S   S_PA23    // I2C_2_SCL    to Touchstrip TCA9555
#define BUS_I2C_2_INT_S   S_PB09    // TCA9555 TOUCHStrip LEDs INT Pin. NOT NEEDED



// AVIALABLE ON BOTH CHIPS ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// LEDs
#define PIN_LED           M_PB17   
#define LED_BUILTIN       M_PB17

#define PIN_LED_M         M_PB17    
#define LED_BUILTIN_M     M_PB17
#define PIN_LED_S         S_PB17    
#define LED_BUILTIN_S     S_PB17



/*
 * USB
 */
#define PIN_USB_DM          M_PA24    // better levea this out?
#define PIN_USB_DP          M_PA25
#define PIN_USB_HOST_ENABLE (M_PA28)  // hmm, wir haben keinen ENABLE PIN!?  habe M_PA28 als Dummy genommen

#define PIN_USB_DM_M      M_PA24
#define PIN_USB_DP_S      M_PA25

#define PIN_USB_DM_S      S_PA24
#define PIN_USB_DP_S      S_PA25




 /*
 * Wire Interfaces
 */
  
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         M_PA04 
#define PIN_WIRE_SCL         M_PA05
#define PERIPH_WIRE          sercom0
#define WIRE_IT_HANDLER      SERCOM0_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;





// SPI Interfaces

#define SPI_INTERFACES_COUNT     1

#define PIN_SPI_CS           M_PA18                   
#define PIN_SPI_MISO         M_PA19                  
#define PIN_SPI_SCK          M_PA17                  
#define PIN_SPI_MOSI         M_PA16                  
#define PERIPH_SPI           sercom1

#define PAD_SPI_TX           SPI_PAD_0_SCK_1     // not sure if correct
#define PAD_SPI_RX           SERCOM_RX_PAD_3        // not sure if correct


static const uint8_t SS   = M_PA18 ;                  // not sure if correct
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;










/*
// Serial interfaces


// Serial1
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

 */





// Analog pins

#define PIN_A0               (S_PA10)
#define PIN_A1               (S_PA07)
#define PIN_A2               (S_PA06)
#define PIN_A3               (S_PB08)
//#define PIN_A4               (PIN_A0 + 4)
//#define PIN_A5               (PIN_A0 + 5)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
//static const uint8_t A4  = PIN_A4;
//static const uint8_t A5  = PIN_A5;

//#define PIN_DAC0             S_PA02
//#define PIN_DAC1             35

//static const uint8_t DAC0 = PIN_DAC0;
//static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12



/*
// Other pins
#define PIN_ATN              (26ul)
static const uint8_t ATN = PIN_ATN;
 /*




/*
 * I2S Interfaces
 */
//#define I2S_INTERFACES_COUNT 0
//#define I2S_DEVICE          0
//// no I2S on G19!
//
//
//void dacInit();
//void dacWrite(uint16_t  left ,uint16_t  right);
    



#ifdef __cplusplus
}
#endif


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1




#endif /* _VARIANT_COOMPOSER_PRO_ */

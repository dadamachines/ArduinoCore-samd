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

#ifndef _VARIANT_DOPPLER_
#define _VARIANT_DOPPLER_

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
#define PINS_COUNT           (37u)
#define NUM_DIGITAL_PINS     (27u)
#define NUM_ANALOG_INPUTS    (10u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + PIN_A0 : -1)

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

    /*          DOPPLER-Board-Layout:
     *                                                                                    ---------------- FPGA Pins ------------------
     *                                                     DAC1      SCK  MOSI DAC0      LedR LedG LedB
     * DIL Pin 48   47   46   45   44   43   42   41   40   39   38   37   36   35   34   33   32   31   30   29   28   27   26   25
     *       |--O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O---|
     * name  | VIN  5V  3.3V  A10  A9   A8   A7   A6   A5   A4   A3   A2   A1   A0   GND  R2   R1   R0   F14  F13  F12  F11  F10  F9   |
     * alt   | VIN  5V  3.3V PA11 PA10 PA09 PA08 PA07 PA06 PA05 PA04 PB09 PB08 PA02  GND  41   40   39   38   37   36   35   34   32   |
     *       |                                                                                            ö  ö  ö  ö                   |
     *      |                                                                                             ö  ö  ö  ö         |BTN:S1|  |
     *     | USB                           DOPPLER: SamD51 <- SPI -> icE40        |BTN:RESET|             ö  ö  ö  ö                   |
     *      |                                                                                             ö  ö  ö  ö         |BTN:S2|  |
     *       |                                                                                                                         |
     * alt   | GND PA13 PA12 PB11 PA14 PA15 PB10 PA31 PA30  RES PA19 PA20 PA21 PA22 3.3V   11   12   13   18   19   20   21   23   25  |
     * name  | GND   0    1    2    3    4    5                   6    7    8    9  3.3V  F0   F1   F2   F3   F4   F5   F6   F7   F8   |
     *       L--O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O---|
     * DIL Pin  1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24
     *             SCL  SDA   MISO           SS   SWD  SWC RES
     *             -- I2C--                       --- SWD  ---   ----- Shared  -----      ---------------- FPGA Pins ------------------
     */



// LEDs
#define PIN_LED_33          (33u)
#define PIN_LED              PIN_LED_33
#define LED_BUILTIN          PIN_LED_33

// FPGA internal SPI Bus
#define ICE_CLK     21     // PB03
#define ICE_MOSI    22    // PB23
#define ICE_MISO    23     // PB02
#define ICE_CS      24     // PB22
#define ICE_CDONE   25     // PA16
#define ICE_CRESET  26     // PA17


/*
 * Analog pins
 */
#define PIN_A0               (10ul)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_A4               (PIN_A0 + 4)
#define PIN_A5               (PIN_A0 + 5)
#define PIN_A6               (PIN_A0 + 6)
#define PIN_A7               (PIN_A0 + 7)
#define PIN_A8               (PIN_A0 + 8)
#define PIN_A9               (PIN_A0 + 9)
#define PIN_A10              (PIN_A0 + 10)

#define PIN_DAC0             34
#define PIN_DAC1             35

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
static const uint8_t A8  = PIN_A8 ;
static const uint8_t A9  = PIN_A9 ;
static const uint8_t A10 = PIN_A10 ;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12

// Other pins
#define PIN_ATN              (26ul)
static const uint8_t ATN = PIN_ATN;

/*
 * Serial interfaces
 */

// Serial1 TODO: CHECK
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT     1

#define PIN_SPI_CS           (5u)
#define PIN_SPI_MISO         (2u)
#define PIN_SPI_SCK          (12u)
#define PIN_SPI_MOSI         (11u)
#define PERIPH_SPI           sercom4
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3

static const uint8_t SS	  = PIN_A2 ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;



 /*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (1u)
#define PIN_WIRE_SCL         (0u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (27ul)
#define PIN_USB_DM          (28ul)
#define PIN_USB_DP          (29ul)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

#define I2S_DEVICE          0
// no I2S on G19!

//QSPI Pins
#define PIN_QSPI_SCK	(5u)
#define PIN_QSPI_CS	    (2u)
#define PIN_QSPI_IO0	(17u)
#define PIN_QSPI_IO1	(18u)
#define PIN_QSPI_IO2	(19u)
#define PIN_QSPI_IO3	(20u)

/*
//PCC Pins
#define PIN_PCC_DEN1    (PIN_SPI_MOSI)
#define PIN_PCC_DEN2    (PIN_SPI_SCK)
#define PIN_PCC_CLK     (PIN_SPI_MISO)
#define PIN_PCC_D0      (13u)
#define PIN_PCC_D1      (12u)
#define PIN_PCC_D2      (10u)
#define PIN_PCC_D3      (11u)
#define PIN_PCC_D4      (9u)
#define PIN_PCC_D5      (8u)
#define PIN_PCC_D6      (1u)
#define PIN_PCC_D7      (0u)
#define PIN_PCC_D8      (5u)
#define PIN_PCC_D9      (6u)
*/

//TODO: meaningful value for this
#define VARIANT_QSPI_BAUD_DEFAULT 5000000



void dacInit();
void dacWrite(uint16_t  left ,uint16_t  right);

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











#endif /* _VARIANT_DOPPLER_ */

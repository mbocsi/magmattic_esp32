////////////////////////////////////////////////////////////////////////////////////////////
//    Arduino library for the ADS1262 32-bit ADC breakout board from ProtoCentral
//    Copyright (c) 2020 ProtoCentral Electronics
//
//    This example measures raw capacitance across CHANNEL0 and Gnd and
//    prints on serial terminal
//    
//    this example gives differential voltage across the AN0 And AN1 in mV
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//    NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//     WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//  
//     For information on how to use, visit https://github.com/Protocentral/ProtoCentral_ads1262
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef ads1262_h
#define ads1262_h

#include "Arduino.h"
#include <map>

#define CONFIG_SPI_MASTER_DUMMY   0xFF

// Register Read Commands
#define  RREG    0x20;		//Read n nnnn registers starting at address r rrrr
                                //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG    0x40;		//Write n nnnn registers starting at address r rrrr
                                //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define START		0x08		//Start/restart (synchronize) conversions
#define STOP		0x0A		//Stop conversion
#define RDATAC      0x10		//Enable Read Data Continuous mode. 

//This mode is the default mode at power-up.
#define SDATAC		0x11		//Stop Read Data Continuously mode
#define RDATA		0x12		//Read data by command; supports multiple read back.

//Pin declartion the other you need are controlled by the SPI library
const int ADS1262_DRDY_PIN = 25;
const int ADS1262_CS_PIN = 5;
const int ADS1262_START_PIN = 27;
const int ADS1262_PWDN_PIN = 26;

//register address
#define	POWER		0x01
#define	INTERFACE	0x02
#define	MODE0		0x03
#define	MODE1		0x04
#define	MODE2		0x05
#define	INPMUX		0x06
#define	OFCAL0		0x07
#define	OFCAL1		0x08
#define	OFCAL2		0x09
#define	FSCAL0		0x0A
#define	FSCAL1		0x0B
#define	FSCAL2		0x0C
#define	IDACMUX		0x0D
#define	IDACMAG		0x0E
#define	REFMUX		0x0F
#define	TDACP		0x10
#define	TDACN		0x11
#define	GPIOCON		0x12
#define	GPIODIR		0x13
#define	GPIODAT		0x14
#define	ADC2CFG		0x15
#define	ADC2MUX		0x16
#define	ADC2OFC0	0x17
#define	ADC2OFC1	0x18
#define	ADC2FSC0	0x19
#define	ADC2FSC1	0x1A


class Ads1262
{
  public:
    static void init(void);
    static void reset(void);
    static void reg_write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA);
    static void reg_read (unsigned char READ_WRITE_ADDRESS);
    static void SPI_command_data(unsigned char data_in);
    static void disable_start(void);
    static void enable_start(void);
    static void hard_stop (void);
    static void start_data_conv_command (void);
    static void soft_stop (void);
    // static void start_read_data_continuous (void);
    // static void stop_read_data_continuous (void);
    static char* read_data(void);
    static bool set_sample_rate(const unsigned int rate);
};

#endif


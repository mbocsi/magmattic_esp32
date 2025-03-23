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

#include <Arduino.h>
#include <ads1262.h>
#include <SPI.h>

std::map<int, int> SAMPLE_RATE{{100, 0x07}, {400, 0x08}, {1200, 0x09}, {2400, 0x0A}, {4800, 0x0B}, {7200, 0x0C}};

char* Ads1262::read_data()
{
  static char SPI_Dummy_Buff[6];
  digitalWrite(ADS1262_CS_PIN, LOW);
   
	for (int i = 0; i < 6; ++i)
	{
		SPI_Dummy_Buff[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
	}
	
  digitalWrite(ADS1262_CS_PIN, HIGH);
	
	return SPI_Dummy_Buff;
}

bool Ads1262::set_sample_rate(const unsigned int rate) {
  if(SAMPLE_RATE.find(rate) == SAMPLE_RATE.end()) {
    return false;
  }
  
  int code = SAMPLE_RATE[rate];
  hard_stop();
  delay(350);
  reg_write(MODE2, code); 
  delay(10);
  enable_start();
  return true;
}

void Ads1262::init()
{
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  //CPOL = 0, CPHA = 1
  SPI.setDataMode(SPI_MODE1);
  // Selecting 1Mhz clock for SPI
  SPI.setClockDivider(SPI_CLOCK_DIV8); // DIV16

  reset();
  delay(100);
  
  hard_stop();
  delay(350);
  
  reg_write(POWER, 0x11);
  delay(10);
  reg_write(INTERFACE, 0x05);	//Lead-off comp off, test signal disabled
  delay(10);
  reg_write(MODE0, 0x00);
  delay(10);
  reg_write(MODE1, 0x80);	//Ch 1 enabled, gain 6, connected to electrode in
  delay(10);
  reg_write(MODE2, 0x09); // Set sampling rate to 1200 SPS	
  delay(10);
  reg_write(INPMUX, 0x01);	
  delay(10);  
  reg_write(OFCAL0, 0x00);	
  delay(10);  
  reg_write(OFCAL1, 0x00);	
  delay(10);  
  reg_write(OFCAL2, 0x00);	
  delay(10);  
  reg_write(FSCAL0, 0x00);	
  delay(10);  
  reg_write(FSCAL1, 0x00);	
  delay(10);  
  reg_write(FSCAL2, 0x40);	
  delay(10);  
  reg_write(IDACMUX, 0xBB);	
  delay(10);  
   reg_write(IDACMAG, 0x00);	
  delay(10);  
  reg_write(REFMUX, 0x00);	
  delay(10);    
  reg_write(TDACP, 0x00);	
  delay(10);    
  reg_write(TDACN, 0x00);	
  delay(10);    
  reg_write(GPIOCON, 0x00);	
  delay(10);    
  reg_write(GPIODIR, 0x00);	
  delay(10);    
  reg_write(GPIODAT, 0x00);	
  delay(10);    
  reg_write(ADC2CFG, 0x00);	
  delay(10);    
  reg_write(ADC2MUX, 0x01);	
  delay(10);    
  reg_write(ADC2OFC0, 0x00);	
  delay(10);    
  reg_write(ADC2OFC1, 0x00);	
  delay(10);    
  reg_write(ADC2FSC0, 0x00);	
  delay(10);    
  reg_write(ADC2FSC1, 0x40);	
  delay(10);
 // start_read_data_continuous();
  delay(10);
  enable_start();
}

void Ads1262::reset()
{
  digitalWrite(ADS1262_PWDN_PIN, HIGH);
  delay(100);					// Wait 100 mSec
  digitalWrite(ADS1262_PWDN_PIN, LOW);
  delay(100);
  digitalWrite(ADS1262_PWDN_PIN, HIGH);
  delay(100);
}

void Ads1262::disable_start()
{
  digitalWrite(ADS1262_START_PIN, LOW);
  delay(20);
}

void Ads1262::enable_start()
{
  digitalWrite(ADS1262_START_PIN, HIGH);
  delay(20);
}

void Ads1262::hard_stop (void)
{
  digitalWrite(ADS1262_START_PIN, LOW);
  delay(100);
}


void Ads1262::start_data_conv_command (void)
{
  SPI_command_data(START);					// Send 0x08 to the ADS1x9x
}

void Ads1262::soft_stop (void)
{
  SPI_command_data(STOP);                   // Send 0x0A to the ADS1x9x
}

// void Ads1262::start_read_data_continuous (void)
// {
//   //SPI_command_data(RDATAC);					// Send 0x10 to the ADS1x9x
// }

// void Ads1262::stop_read_data_continuous (void)
// {
//   //SPI_command_data(SDATAC);					// Send 0x11 to the ADS1x9x
// }

void Ads1262::SPI_command_data(unsigned char data_in)
{
  byte data[1];
  //data[0] = data_in;
  digitalWrite(ADS1262_CS_PIN, LOW);
  delay(2);
  digitalWrite(ADS1262_CS_PIN, HIGH);
  delay(2);
  digitalWrite(ADS1262_CS_PIN, LOW);
  delay(2);
  SPI.transfer(data_in);
  delay(2);
  digitalWrite(ADS1262_CS_PIN, HIGH);
}

//Sends a write command to SCP1000
void Ads1262::reg_write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
  // now combine the register address and the command into one byte:
  byte dataToSend = READ_WRITE_ADDRESS | WREG;
  
   digitalWrite(ADS1262_CS_PIN, LOW);
   delay(2);
   digitalWrite(ADS1262_CS_PIN, HIGH);
   delay(2);
  // take the chip select low to select the device:
  digitalWrite(ADS1262_CS_PIN, LOW);
  delay(2);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(0x00);		//number of register to wr
  SPI.transfer(DATA);		//Send value to record into register
  
  delay(2);
  // take the chip select high to de-select:
  digitalWrite(ADS1262_CS_PIN, HIGH);
}
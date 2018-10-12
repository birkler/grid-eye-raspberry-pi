
#ifndef LIB_ADAFRUIT_AMG88XX_H
#define LIB_ADAFRUIT_AMG88XX_H

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define AMG88xx_ADDRESS                (0x69)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
        AMG88xx_PCTL = 0x00,
		AMG88xx_RST = 0x01,
		AMG88xx_FPSC = 0x02,
		AMG88xx_INTC = 0x03,
		AMG88xx_STAT = 0x04,
		AMG88xx_SCLR = 0x05,
		//0x06 reserved
		AMG88xx_AVE = 0x07,
		AMG88xx_INTHL = 0x08,
		AMG88xx_INTHH = 0x09,
		AMG88xx_INTLL = 0x0A,
		AMG88xx_INTLH = 0x0B,
		AMG88xx_IHYSL = 0x0C,
		AMG88xx_IHYSH = 0x0D,
		AMG88xx_TTHL = 0x0E,
		AMG88xx_TTHH = 0x0F,
		AMG88xx_INT_OFFSET = 0x010,
		AMG88xx_PIXEL_OFFSET = 0x80
    };
	
	enum power_modes{
		AMG88xx_NORMAL_MODE = 0x00,
		AMG88xx_SLEEP_MODE = 0x01,
		AMG88xx_STAND_BY_60 = 0x20,
		AMG88xx_STAND_BY_10 = 0x21
	};
	
	enum sw_resets {
		AMG88xx_FLAG_RESET = 0x30,
		AMG88xx_INITIAL_RESET = 0x3F
	};
	
	enum frame_rates {
		AMG88xx_FPS_10 = 0x00,
		AMG88xx_FPS_1 = 0x01
	};
	
	enum int_enables{
		AMG88xx_INT_DISABLED = 0x00,
		AMG88xx_INT_ENABLED = 0x01
	};
	
	enum int_modes {
		AMG88xx_DIFFERENCE = 0x00,
		AMG88xx_ABSOLUTE_VALUE = 0x01
	};
	
/*=========================================================================*/

#define AMG88xx_PIXEL_ARRAY_SIZE 64
#define AMG88xx_PIXEL_TEMP_CONVERSION .25
#define AMG88xx_THERMISTOR_CONVERSION .0625

/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with AMG88xx IR sensor chips
*/
/**************************************************************************/
class Adafruit_AMG88xx {
    protected:
    	int filehnd;
    	const char* dev_filename;

		void delay(int ms) {return;}

		int constrain(int val,int minval, int maxval) {
			return std::min(maxval,std::max(val,minval));
		}

		int min(int val0,int val1) {return std::min(val0,val1);}
    
	public:
		using byte = uint8_t;
		//constructors
		Adafruit_AMG88xx();
		~Adafruit_AMG88xx();
		
		bool init(uint8_t addr = AMG88xx_ADDRESS);
		
		void readPixels(float *buf, uint8_t size = AMG88xx_PIXEL_ARRAY_SIZE);
		float readThermistor();
		
		void setMovingAverageMode(bool mode);
		
		void	  enableInterrupt();
		void	  disableInterrupt();
		void	  setInterruptMode(uint8_t mode);
		void	  getInterrupt(uint8_t *buf, uint8_t size = 8);
		void	  clearInterrupt();
				
		//this will automatically set hysteresis to 95% of the high value
		void	  setInterruptLevels(float high, float low);
				
		//this will manually set hysteresis
		void	  setInterruptLevels(float high, float low, float hysteresis);
		
	private:
		uint8_t _i2caddr;
		
		void      write8(byte reg, byte value);
		void      write16(byte reg, uint16_t value);
        uint8_t   read8(byte reg);
		
		void read(uint8_t reg, uint8_t *buf, uint8_t num);
		void _i2c_init();
		
		float signedMag12ToFloat(uint16_t val);
		
		 // The power control register
        struct pctl {
            // 0x00 = Normal Mode
			// 0x01 = Sleep Mode
			// 0x20 = Stand-by mode (60 sec intermittence)
			// 0x21 = Stand-by mode (10 sec intermittence)
           
            uint8_t PCTL : 8;

            uint8_t get() {
                return PCTL;
            }
        };
        pctl _pctl;
		
		//reset register
		struct rst {
			//0x30 = flag reset (all clear status reg 0x04, interrupt flag and interrupt table)
			//0x3F = initial reset (brings flag reset and returns to initial setting)
			
			uint8_t RST : 8;
			
			uint8_t get() {
				return RST;
			}
		};
		rst _rst;
		
		//frame rate register
		struct fpsc {
			
			//0 = 10FPS
			//1 = 1FPS
			uint8_t FPS : 1;
			
			uint8_t get() {
				return FPS & 0x01;
			}
		};
		fpsc _fpsc;
		
		//interrupt control register
		struct intc {
			
			// 0 = INT output reactive (Hi-Z)
			// 1 = INT output active
			uint8_t INTEN : 1;
			
			// 0 = Difference interrupt mode
			// 1 = absolute value interrupt mode
			uint8_t INTMOD : 1;
			
			uint8_t get(){
				return (INTMOD << 1 | INTEN) & 0x03;
			}
		};
		intc _intc;
		
		//status register
		struct stat {
			uint8_t unused : 1;
			//interrupt outbreak (val of interrupt table reg)
			uint8_t INTF : 1;
			
			//temperature output overflow (val of temperature reg)
			uint8_t OVF_IRS : 1;
			
			//thermistor temperature output overflow (value of thermistor)
			uint8_t OVF_THS : 1;
			
			uint8_t get(){
				return ( (OVF_THS << 3) | (OVF_IRS << 2) | (INTF << 1) ) & 0x07;
			}
		};
		stat _stat;
		
		//status clear register
		//write to clear overflow flag and interrupt flag
		//after writing automatically turns to 0x00
		struct sclr {
			uint8_t unused : 1;
			//interrupt flag clear
			uint8_t INTCLR : 1;
			//temp output overflow flag clear
			uint8_t OVS_CLR : 1;
			//thermistor temp output overflow flag clear
			uint8_t OVT_CLR : 1;
			
			uint8_t get(){
				return ((OVT_CLR << 3) | (OVS_CLR << 2) | (INTCLR << 1)) & 0x07;
			}
		};
		sclr _sclr;
		
		//average register
		//for setting moving average output mode
		struct ave {
			uint8_t unused : 5;
			//1 = twice moving average mode
			uint8_t MAMOD : 1;
			
			uint8_t get(){
				return (MAMOD << 5);
			}
		};
		struct ave _ave;
		
		//interrupt level registers
		//for setting upper / lower limit hysteresis on interrupt level
		
		//interrupt level upper limit setting. Interrupt output
		// and interrupt pixel table are set when value exceeds set value
		struct inthl {
			uint8_t INT_LVL_H : 8;
			
			uint8_t get(){
				return INT_LVL_H;
			}
		};
		struct inthl _inthl;
		
		struct inthh {
			uint8_t INT_LVL_H : 4;
			
			uint8_t get(){
				return INT_LVL_H;
			}
		};
		struct inthh _inthh;
		
		//interrupt level lower limit. Interrupt output
		//and interrupt pixel table are set when value is lower than set value
		struct intll {
			uint8_t INT_LVL_L : 8;
			
			uint8_t get(){
				return INT_LVL_L;
			}
		};
		struct intll _intll;
		
		struct intlh {
			uint8_t INT_LVL_L : 4;
			
			uint8_t get(){
				return (INT_LVL_L & 0xF);
			}
		};
		struct intlh _intlh;
		
		//setting of interrupt hysteresis level when interrupt is generated.
		//should not be higher than interrupt level
		struct ihysl {
			uint8_t INT_HYS : 8;
			
			uint8_t get(){
				return INT_HYS;
			}
		};
		struct ihysl _ihysl;
		
		struct ihysh {
			uint8_t INT_HYS : 4;
			
			uint8_t get(){
				return (INT_HYS & 0xF);
			}
		};
		struct ihysh _ihysh;
		
		//thermistor register
		//SIGNED MAGNITUDE FORMAT
		struct tthl {
			uint8_t TEMP : 8;
			
			uint8_t get(){
				return TEMP;
			}
		};
		struct tthl _tthl;
		
		struct tthh {
			uint8_t TEMP : 3;
			uint8_t SIGN : 1;
			
			uint8_t get(){
				return ( (SIGN << 3) | TEMP) & 0xF;
			}
		};
		struct tthh _tthh;
		
		//temperature registers 0x80 - 0xFF
		/*
		//read to indicate temperature data per 1 pixel
		//SIGNED MAGNITUDE FORMAT
		struct t01l {
			uint8_t TEMP : 8;
			
			uint8_t get(){
				return TEMP;
			}
		};
		struct t01l _t01l;
		
		struct t01h {
			uint8_t TEMP : 3;
			uint8_t SIGN : 1;
			
			uint8_t get(){
				return ( (SIGN << 3) | TEMP) & 0xF;
			}
		};
		struct t01h _t01h;
		*/
		
		
};

#define AMG_I2C_CHUNKSIZE 32


Adafruit_AMG88xx::Adafruit_AMG88xx() : filehnd(-1), dev_filename("/dev/i2c-1") {
}
Adafruit_AMG88xx::~Adafruit_AMG88xx() {
	if (filehnd >=0) close(filehnd);
	filehnd = 0;
}
		

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
    @param  addr Optional I2C address the sensor can be found on. Default is 0x69
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool Adafruit_AMG88xx::init(uint8_t addr)
{
	_i2caddr = addr;

	if (filehnd < 0) {
		filehnd = open(dev_filename,O_RDWR);
		if (filehnd < 0) throw std::ios_base::failure("Cannot open /dev/i2c-x");
		printf("%s opened!\n",dev_filename);
	}

	


	_i2c_init();
	
	//enter normal mode
	_pctl.PCTL = AMG88xx_NORMAL_MODE;
	write8(AMG88xx_PCTL, _pctl.get());
	
	//software reset
	_rst.RST = AMG88xx_INITIAL_RESET;
	write8(AMG88xx_RST, _rst.get());
	
	//disable interrupts by default
	disableInterrupt();
	
	//set to 10 FPS
	_fpsc.FPS = AMG88xx_FPS_10;
	write8(AMG88xx_FPSC, _fpsc.get());

	delay(100);

	return true;
}

/**************************************************************************/
/*! 
    @brief  Set the moving average mode.
    @param  mode if True is passed, output will be twice the moving average
*/
/**************************************************************************/
void Adafruit_AMG88xx::setMovingAverageMode(bool mode)
{
	_ave.MAMOD = mode;
	write8(AMG88xx_AVE, _ave.get());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels. The hysteresis value defaults to .95 * high
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
*/
/**************************************************************************/
void Adafruit_AMG88xx::setInterruptLevels(float high, float low)
{
	setInterruptLevels(high, low, high * .95);
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
    @param hysteresis the hysteresis value for interrupt detection
*/
/**************************************************************************/
void Adafruit_AMG88xx::setInterruptLevels(float high, float low, float hysteresis)
{
	int highConv = high / AMG88xx_PIXEL_TEMP_CONVERSION;
	highConv = constrain(highConv, -4095, 4095);
	_inthl.INT_LVL_H = highConv & 0xFF;
	_inthh.INT_LVL_H = (highConv & 0xF) >> 4;
	this->write8(AMG88xx_INTHL, _inthl.get());
	this->write8(AMG88xx_INTHH, _inthh.get());
	
	int lowConv = low / AMG88xx_PIXEL_TEMP_CONVERSION;
	lowConv = constrain(lowConv, -4095, 4095);
	_intll.INT_LVL_L = lowConv & 0xFF;
	_intlh.INT_LVL_L = (lowConv & 0xF) >> 4;
	this->write8(AMG88xx_INTLL, _intll.get());
	this->write8(AMG88xx_INTLH, _intlh.get());
	
	int hysConv = hysteresis / AMG88xx_PIXEL_TEMP_CONVERSION;
	hysConv = constrain(hysConv, -4095, 4095);
	_ihysl.INT_HYS = hysConv & 0xFF;
	_ihysh.INT_HYS = (hysConv & 0xF) >> 4;
	this->write8(AMG88xx_IHYSL, _ihysl.get());
	this->write8(AMG88xx_IHYSH, _ihysh.get());
}

/**************************************************************************/
/*! 
    @brief  enable the interrupt pin on the device.
*/
/**************************************************************************/
void Adafruit_AMG88xx::enableInterrupt()
{
	_intc.INTEN = 1;
	this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*! 
    @brief  disable the interrupt pin on the device
*/
/**************************************************************************/
void Adafruit_AMG88xx::disableInterrupt()
{
	_intc.INTEN = 0;
	this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt to either absolute value or difference mode
    @param  mode passing AMG88xx_DIFFERENCE sets the device to difference mode, AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
*/
/**************************************************************************/
void Adafruit_AMG88xx::setInterruptMode(uint8_t mode)
{
	_intc.INTMOD = mode;
	this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*! 
    @brief  Read the state of the triggered interrupts on the device. The full interrupt register is 8 bytes in length.
    @param  buf the pointer to where the returned data will be stored
    @param  size Optional number of bytes to read. Default is 8 bytes.
    @returns up to 8 bytes of data in buf
*/
/**************************************************************************/
void Adafruit_AMG88xx::getInterrupt(uint8_t *buf, uint8_t size)
{
	uint8_t bytesToRead = min(size, (uint8_t)8);
	
	this->read(AMG88xx_INT_OFFSET, buf, bytesToRead);
}

/**************************************************************************/
/*! 
    @brief  Clear any triggered interrupts
*/
/**************************************************************************/
void Adafruit_AMG88xx::clearInterrupt()
{
	_rst.RST = AMG88xx_FLAG_RESET;
	write8(AMG88xx_RST, _rst.get());
}

/**************************************************************************/
/*! 
    @brief  read the onboard thermistor
    @returns a the floating point temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_AMG88xx::readThermistor()
{
	uint8_t raw[2];
	this->read(AMG88xx_TTHL, raw, 2);
	uint16_t recast = ((uint16_t)raw[1] << 8) | ((uint16_t)raw[0]);

	return signedMag12ToFloat(recast) * AMG88xx_THERMISTOR_CONVERSION;
}

/**************************************************************************/
/*! 
    @brief  Read Infrared sensor values
    @param  buf the array to place the pixels in
    @param  size Optionsl number of bytes to read (up to 64). Default is 64 bytes.
    @return up to 64 bytes of pixel data in buf
*/
/**************************************************************************/
void Adafruit_AMG88xx::readPixels(float *buf, uint8_t size)
{
	uint16_t recast;
	float converted;
	uint8_t bytesToRead = min((uint8_t)(size << 1), (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1));
	uint8_t rawArray[bytesToRead];
	this->read(AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
	
	for(int i=0; i<size; i++){
		uint8_t pos = i << 1;
		recast = ((uint16_t)rawArray[pos + 1] << 8) | ((uint16_t)rawArray[pos]);
		
		converted = signedMag12ToFloat(recast) * AMG88xx_PIXEL_TEMP_CONVERSION;
		buf[i] = converted;
	}
}

/**************************************************************************/
/*! 
    @brief  write one byte of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
void Adafruit_AMG88xx::write8(byte reg, byte value)
{
	uint8_t buf [2]= {reg,value};
	int res = ::write(filehnd,buf,2);
	if (res != 2) throw std::ios_base::failure("Failed to write byte to slave");
}

/**************************************************************************/
/*! 
    @brief  read one byte of data from the specified register
    @param  reg the register to read
    @returns one byte of register data
*/
/**************************************************************************/
uint8_t Adafruit_AMG88xx::read8(byte reg)
{
	uint8_t ret;
	this->read(reg, &ret, 1);
	
	return ret;
}

void Adafruit_AMG88xx::_i2c_init()
{
	if (ioctl(filehnd,I2C_SLAVE,_i2caddr) < 0) {
		throw std::ios_base::failure("Failed to acquire bus access and/or talk to slave");
    } else {
		printf("ioctl ok!\n");
	}

#if __ARDUINO__
	Wire.begin();
#endif
}

void Adafruit_AMG88xx::read(uint8_t reg, uint8_t *buf, uint8_t num)
{
	uint8_t value;
	uint8_t pos = 0;
	uint8_t* bufptr = buf;
	//on arduino we need to read in AMG_I2C_CHUNKSIZE byte chunks
	while(pos < num){
		*bufptr = reg + pos;
		int res = ::read(filehnd,bufptr,1);
		if (res != 1) throw std::ios_base::failure("Failed to read byte from slave at pos %d at address %d");
		pos++;
		bufptr++;
	}
}


/**************************************************************************/
/*! 
    @brief  convert a 12-bit signed magnitude value to a floating point number
    @param  val the 12-bit signed magnitude value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
float Adafruit_AMG88xx::signedMag12ToFloat(uint16_t val)
{
	//take first 11 bits as absolute val
	uint16_t absVal = (val & 0x7FF);
	
	return (val & 0x8000) ? 0 - (float)absVal : (float)absVal ;
}





#endif

#if 0


void
sensors_ADC_init(void) {
    int file;
    char filename[40];
    const gchar *buffer;
    int addr = 0b00101001;        // The I2C address of the ADC

    sprintf(filename,"/dev/i2c-2");
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    char buf[10] = {0};
    float data;
    char channel;

    for(int i = 0; i<4; i++) {
        // Using I2C Read
        if (read(file,buf,2) != 2) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            buffer = g_strerror(errno);
            printf(buffer);
            printf("\n\n");
        } else {
            data = (float)((buf[0] & 0b00001111)<<8)+buf[1];
            data = data/4096*5;
            channel = ((buf[0] & 0b00110000)>>4);
            printf("Channel %02d Data:  %04f\n",channel,data);
        }
    }

    //unsigned char reg = 0x10; // Device register to access
    //buf[0] = reg;
    buf[0] = 0b11110000;

    if (write(file,buf,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
        buffer = g_strerror(errno);
        printf(buffer);
        printf("\n\n");
    }
}

#endif

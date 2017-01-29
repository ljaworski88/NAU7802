/*
NAU7802 Library
.h file

This library is used to simplify the control scheme of the NAU7802 brigde sensor chip for use with a bioreactor controller.
this chip has many registers and options not all of which are explained well in the datasheet. As far as I am aware all register settings are expressed here,
 (The direct OTP read setting is hidden from the user though) if any are missing feel free to log it.

**IMPORTANT**
To start the chip up a specfic sequence of commands needs to be used to ensure that the IC starts up properly and is ready to use.
After the IC recieves power the recomended start up sequence is:	->Can use bootCycle function instead
NAU7802::registerReset(true);
NAU7802::registerReset(false); // This register value needs to be turned off for normal chip operation
NAU7802::powerDigital(); //the default value is true so it should turn on
delay(1); //the IC needs at least 200usec to boot up its internal circuitry
NAU7802::powerAnalog(); //Powers up the ADC
{Set Gain, Offsets, and all other desired settings}
{Calibrate}
NAU7802::cycleStart(); //***This triggers the first conversion and needs to be activated NOTE: It can take a few milliseconds for the first conversion to initialize

NOTE: The IC has 50kOhm I2C pullups enabled internally by default, and can use 1.6kOhm ones, but this must be set in the registers.

Lukas Jaworski

University of Miami
2016
*/

//**NOTE: the function Wire.begin() will need to be called somewhere in the main body of your program, BEFORE you call any of the library functions


#ifndef NAU7802_H
#define NAU7802_H
#include "arduino.h"
#include <Wire.h>

//#define LEAN true //This library is pretty large and if space is tight it cuts out a lot of functions and data structures to the bare minimum to save space (down to 3 functions and 1 data structure) (so no register state tracking either)

#ifndef LEAN

	typedef union Offsets{ //IC reading offsets, can run calibration or eneter manually
		uint32_t Offset;
		uint8_t OffBytes[3];
	} Offsets;
	
	typedef union GOffsets{ //Gain offeset values, can run a calibration or set manually
		uint32_t GOffset;
		uint8_t GOffBytes[4];
	} GOffsets;
	
	typedef union OTPread{ //Internal temperature sensor values
		uint32_t OTPvalue;
		uint8_t OTPbytes[4];
	} OTPread;
	
	typedef enum gainSelect { //used to select the desired Gain into the ADC
		x1 = 0,
		x2,
		x4,
		x8,
		x16,
		x32,
		x64,
		x128
	} gainSelect;
	
	typedef enum LDOSelect { //used to select the internal IC LDO output
		V45 = 0, //4.5V
		V42,     //4.2V
		V39,
		V36,
		V33,
		V30,
		V27,
		V24      //2.4V
	} LDOSelect;
	
	typedef enum sampleRate {//Used in setting the sample rate (samples per second)
		SPS10 = 0b000,
		SPS20,
		SPS40,
		SPS80,
		SPS320 = 0b111
	} sampleRate;
	
	typedef enum calType { //used to set which type of calibration will be run
		INTERN	 = 0b00,
		OFFSET	 = 0b10,
		GAIN 	 = 0b11
	} calType;
	
	typedef enum ComMode { //used to set the common mode
		DISABLED	= 0b00,
		REFN	 	= 0b10,
		REFP 	 	= 0b11
	} ComMode;
	
typedef enum I2C_level { //used to set the common mode
		NOPULLUP	= 0b00,
		WEAK	 	= 0b10,
		STRONG 	 	= 0b11
	} I2C_level;
	
	typedef enum registerAddress { //Register address map, used in selecting which address is being worked with. NOTE: ADC_REG and OTP_READ share the 0x15 register and to read the state and the proper mode must be selected, any writes always go to the ADC_REG
		PU_CTRL 	= 0x00,
		CTRL1 		= 0x01,
		CTRL2 		= 0x02,
		CH1_OFFSET 	= 0x03,
		CH1_GOFFSET = 0x06,
		CH2_OFFSET 	= 0x0A,
		CH2_GOFFSET = 0x0D,
		I2C_CTRL 	= 0x11,
		ADC_RESULT 	= 0x12,
		ADC_REG 	= 0x15,
		OTP_READ 	= 0x15,
		PGA_REG 	= 0x1B,
		PWR_CTRL 	= 0x1C
	} registerAddress;
	
	typedef struct registerState { //intializes the variables to their initial conditions on chip startup
		uint8_t PU_CTRL 		=	 0;
		uint8_t CTRL1 			=	 0;
		uint8_t CTRL2 			=	 0;
		uint8_t I2C_CTRL 		=	 0;
		uint8_t ADC_REG 		=	 0;
		uint8_t PGA_REG 		=	 0;
		uint8_t PWR_CTRL 		=	 0;
	} registerState;
	
	typedef struct calibrationOffsets{ //stores the IC offsets
		Offsets CH1_OFFSET;
		GOffsets CH1_GOFFSET;
		Offsets CH2_OFFSET;
		GOffsets CH2_GOFFSET;
	} calibrationOffsets;

#endif 

	typedef union loadReading{ //the load reading taken from the IC
		uint32_t Load;
		uint8_t bytes[4];
	} loadReading;
	
	
class NAU7802 {
	public:
	
	NAU7802();
	
#ifndef LEAN

	void selectAVDDsource(bool internal_source = false); //select the AVdd source (external = 0 (default) internal LDO =1)
	void selectClockSource(bool internal_source = true); //select the IC clock source (internal = 0 (default) external = 1)
	void cycleStart(bool start = true);//starts the next conversion at the rising edge of this trigger. Needed to start the first conversion.
	bool powerAnalog(bool powerUp = true); //power up the analog circuit
	bool powerDigital(bool powerUp = true); //power up all digital circuitry in the IC
	bool registerReset(bool reset = true); //resets all registers to their default value. NOTE: This bit must be manually cleared otherwise the chip will reset itself indefinitely.
	bool checkPowerUp(); //checks the powerup status of the IC to see if it is ready to use
	bool checkDataReady(); //checks the register to see if new data is ready
	void setRDYpinPolarity(bool activeHigh = true); //sets the data conversion pin polarity (default HIGH = conversion ready)
	uint8_t setGain(gainSelect gain);//Set the ADC gain from 1-128
	void setVLDO(LDOSelect VLDO);//Set the AVDD/LDO voltage from 4.5V-2.4V 
	void setChannel(uint8_t channel);//select channel 1 (0) or channel 2 (1)
	void setConversionRate(sampleRate conversionRate);//Set the Samples per second 10-320
	void setCalibrationMode(calType calibrationSelect);//selects what type of calibration will be run during the calibrate function
	bool calibrate();//runs a calibration, what is being calibrated is set by setCalibrationMode()
	void setSDAconvRDY(bool enabled = true); //special nonstandard I2C protocol which allows the IC to use the SDA line to be pulled low to signal conversion completion, if the data line is not in use
	void setFastRead(bool enabled = true);//special I2C protocol which allows the IC to convert out all 24 to 32 bits of the desired data in a "burst mode" with no ACKs between bytes
	void I2CstrPU(bool enabled = true); //Use the strong pullups on the I2C line (they are 1.6k)
	void I2CwkPU(bool enabled = true); //Use the weak I2C pullups (50K). These pullups are enabled by default and must be disabled to be turned off
	void shortInput(); //internally shorts the input lines together and measures the offset (where this value is stored or used is unclear in the datasheet)
	void setBurnoutCurrent(bool enabled = true); //enables the burnoutcurrent on the PGA positive input
	void setTempSensor(bool enabled = true); //sets the PGA to read from the temp sensor vs input
	void setBandgapChopper(bool enabled = true); //enables or disables the bandgap chopper
	void setCLKfreq(uint8_t freq); // if set to 0b11 will be turned off otherwise on (not explained in datasheet)
	void setCommonMode(ComMode mode);//For single ended opporation, sets which pin will be the common mode.
	void setChopClockDelay(uint8_t select);
	uint32_t readOTP(); //read the IC temperature sensor
	void setLDOmode(bool highESR);
	void setPGAoutputBuffer(bool enabled = true);
	void setPGAbypass(bool enabled = true);
	void setPGAinputInversion(bool enabled = true);
	void setPGAchopper(bool enabled = false);
	void setPGAbypassCAP(bool enabled = true);
	void setMasterBiasCurrent(uint8_t current);
	void setADCcurrent(uint8_t current);
	void setPGAcurrent(uint8_t current);
	void bootCycle(void);//runs through the boot sequence needed to start the NAU7802
	
	uint8_t readRegister(registerAddress address); // read the current value of any register
	void setRegister(registerAddress address, uint8_t regValue); //set the value of any register directly (use the datasheet)
#endif

	uint32_t readLoad(); //reads the load and returns the value as a 32-bit unsigned integer (the result only has 24bits)
	uint8_t readRegister(uint8_t address); // read the current value of any register
	void setRegister(uint8_t address, uint8_t regValue); //set the value of any register directly (use the datasheet)
	
	private:
	
	const uint8_t deviceAddress = 0x2A;
	
	loadReading currentReading;
	
	
#ifndef LEAN
	
	OTPread		currentTemp;
	registerState currentRegisterState;
	registerAddress currentRegister = PU_CTRL;
	
#endif
};
#endif
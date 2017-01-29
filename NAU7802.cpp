/*
NAU7802 Library
.cpp file

This library is used to simplify the controll scheme of the NAU7802 brigde sensor chip for use with a bioreactor controller.
This library holds the current register state in one structure meaning it is relatively large, this is done so that the current register state does not have to be read before changing it increasing communication speed.
As such if space is a concern the library can be cut down by defining the macro LEAN to true (honestly any value will do).

Lukas Jaworski

University of Miami
2016
*/

#include <NAU7802.h>

NAU7802::NAU7802(){
	currentReading.Load = 0;
}

#ifndef LEAN

void NAU7802::selectAVDDsource(bool internal_source){//Allows the selection of the analog voltage source
	currentRegister = PU_CTRL;
	currentRegisterState.PU_CTRL = ((currentRegisterState.PU_CTRL & 0b01111111) | ((~internal_source & 1) << 7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PU_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::selectClockSource(bool internal_source){ //Decides whether the internal or an external clock is used
	currentRegister = PU_CTRL;
	currentRegisterState.PU_CTRL = ((currentRegisterState.PU_CTRL & 0b10111111) | (internal_source << 6));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PU_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::cycleStart(bool start){
	currentRegister = PU_CTRL;
	currentRegisterState.PU_CTRL = ((currentRegisterState.PU_CTRL & 0b11101111) | (start << 4));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PU_CTRL);
	Wire.endTransmission();
	return;
}

bool NAU7802::powerAnalog(bool powerUp){
	currentRegister = PU_CTRL;
	currentRegisterState.PU_CTRL = ((currentRegisterState.PU_CTRL & 0b11111011) | (powerUp << 2));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PU_CTRL);
	Wire.endTransmission();
	return powerUp;
}

bool NAU7802::powerDigital(bool powerUp){
	currentRegister = PU_CTRL;
	currentRegisterState.PU_CTRL = ((currentRegisterState.PU_CTRL & 0b11111101) | (powerUp << 1));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PU_CTRL);
	Wire.endTransmission();
	return powerUp;
}

bool NAU7802::registerReset(bool reset){
	currentRegister = PU_CTRL;
	currentRegisterState.PU_CTRL = ((currentRegisterState.PU_CTRL & 0b11111110) | reset);
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PU_CTRL);
	Wire.endTransmission();
	return reset;
}

bool NAU7802::checkPowerUp(){
	currentRegister = PU_CTRL;
	uint8_t registerValue;
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.endTransmission(false);
	Wire.requestFrom(deviceAddress, uint8_t(1));
	registerValue = Wire.read();
	Wire.endTransmission();
	if (((registerValue >> 3) & 1)){
		return true;
	}
	else{
		return false;
	}
}

bool NAU7802::checkDataReady(){
	currentRegister = PU_CTRL;
	int8_t registerValue;
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.endTransmission(false);
	Wire.requestFrom(deviceAddress, uint8_t(1));
	registerValue = Wire.read();
	// Serial.print("regVal: ");
	// Serial.println(registerValue);
	Wire.endTransmission();
	if (((registerValue >> 5) & 1)){
		return true;
	}
	else{
		return false;
	}
}

uint8_t NAU7802::setGain(gainSelect gain){
	currentRegister = CTRL1;
	currentRegisterState.CTRL1 = ((currentRegisterState.CTRL1 & 0b11111000) | gain);
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.CTRL1);
	Wire.endTransmission();
	return uint8_t(gain);
}

void NAU7802::setVLDO(LDOSelect VLDO){
	currentRegister = CTRL1;
	currentRegisterState.CTRL1 = ((currentRegisterState.CTRL1 & 0b11000111) | (VLDO << 3));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.CTRL1);
	Wire.endTransmission();
	return;
}

void NAU7802::setChannel(uint8_t channel){
	currentRegister = CTRL2;
	currentRegisterState.CTRL2 = ((currentRegisterState.CTRL2 & 0b01111111) | (channel << 7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.CTRL2);
	Wire.endTransmission();
	return;
}

void NAU7802::setConversionRate(sampleRate conversionRate){
	currentRegister = CTRL2;
	currentRegisterState.CTRL2 = ((currentRegisterState.CTRL2 & 0b10001111) | (conversionRate << 4));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.CTRL2);
	Wire.endTransmission();
	return;
}

void NAU7802::setCalibrationMode(calType calibrationSelect){
	currentRegister = CTRL2;
	currentRegisterState.CTRL2 = ((currentRegisterState.CTRL2 & 0b11111100) | (calibrationSelect));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.CTRL2);
	Wire.endTransmission();
	return;
}

bool NAU7802::calibrate(){//starts a calibration, waits until it is complete and returns true if there were no errors, throws false on a calibration error
	currentRegister = CTRL2;
    uint8_t registerValue;
	currentRegisterState.CTRL2 = ((currentRegisterState.CTRL2 & 0b11111011) | (1 << 2));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.CTRL2);
	Wire.endTransmission();
    do{
        Wire.beginTransmission(deviceAddress);
        Wire.write(currentRegister);
        Wire.endTransmission(false);
        Wire.requestFrom(deviceAddress, uint8_t(1));
        registerValue = Wire.read();
		Wire.endTransmission();
    } while( 1 & (registerValue>>2) );
    if ( 1 & (registerValue>>3) ){return false;}
    else {return true;}
}

void NAU7802::setSDAconvRDY(bool enabled){ //allow the SDA to become a conversion ready interrupt when not in use. (not standard I2C)
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b01111111) | (enabled << 7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setFastRead(bool enabled){ //allow fast read mode of ADC data, ADC_REG bit 7 must also be 1  (not standard I2C)
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b10111111) | (enabled << 6));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	currentRegister = ADC_REG;
	currentRegisterState.ADC_REG = ((currentRegisterState.ADC_REG & 0b01111111) | (enabled << 7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.ADC_REG);
	Wire.endTransmission();
	return;
}

// void NAU7802:: I2CpullUps(I2C_level I2Clevel){
	// currentRegister = I2C_CTRL;
	// switch (I2Clevel){
		// case NOPULLUP:
		// break;
		// case WEAK:
		// currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11101111) | ((~1) << 4));
		// break;
		// case STRONG:
		// currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11001111) | (1 << 5));
		// break;
	// }
	// Wire.beginTransmission(deviceAddress);
	// Wire.write(currentRegister);
	// Wire.write(currentRegisterState.I2C_CTRL);
	// Wire.endTransmission();
	// return;
// }

void NAU7802::I2CstrPU(bool enabled){ //use strong pullup resistors on the i2c (1.6k)
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11011111) | (enabled << 5));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::I2CwkPU(bool enabled){ //use weak pullup resistors on the i2c (50k) (These are enabled by default)
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11101111) | ((~enabled & 1) << 4));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::shortInput(){ //short the input and measure the offset
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11110111) | (1 << 3));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setBurnoutCurrent(bool enabled){ //I really don't know what the burnout current is or does, but the setting is in the data sheet so you can set it here.
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11111011) | (enabled << 2));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setTempSensor(bool enabled){ //sets the temp sensor to go to the PGA
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11111101) | (enabled << 1));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setBandgapChopper(bool enabled){ //enables the bandgap chopper (this is enabled by default)
	currentRegister = I2C_CTRL;
	currentRegisterState.I2C_CTRL = ((currentRegisterState.I2C_CTRL & 0b11111110) | (~enabled & 1));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.I2C_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setCLKfreq(uint8_t freq){ 
	currentRegister = ADC_REG;
	currentRegisterState.ADC_REG = ((currentRegisterState.ADC_REG & 0b11001111) | (freq<<4));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.ADC_REG);
	Wire.endTransmission();
	return;
}

void NAU7802::setCommonMode(ComMode mode){ 
	currentRegister = ADC_REG;
	currentRegisterState.ADC_REG = ((currentRegisterState.ADC_REG & 0b11110011) | (mode<<2));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.ADC_REG);
	Wire.endTransmission();
	return;
}

void NAU7802::setChopClockDelay(uint8_t select){ //the datasheet has nothing listed about these modes 
	currentRegister = ADC_REG;
	currentRegisterState.ADC_REG = ((currentRegisterState.ADC_REG & 0b11111100) | (select));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.ADC_REG);
	Wire.endTransmission();
	return;
}

uint32_t NAU7802::readOTP(){ //ADC_REG and the MSB of the OTP_READ share a register address and a setting needs changed to read the OTP, this function sets that setting and then reverts back to reading the ADC_REG value
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = (currentRegisterState.PGA_REG  | (1<<7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	currentRegister = OTP_READ;
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.endTransmission(false);
	Wire.requestFrom(deviceAddress, uint8_t(4));
	for (int8_t i = 3; i >= 0; i--){
		currentTemp.OTPbytes[i] = Wire.read();
	}
	Wire.endTransmission();
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = (currentRegisterState.PGA_REG  & ~(1<<7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	return currentTemp.OTPvalue;
}

void NAU7802::setLDOmode(bool highESR){
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = ((currentRegisterState.PGA_REG & 0b10111111)  | (highESR<<6));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	return;
}

void NAU7802::setPGAoutputBuffer(bool enabled){
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = ((currentRegisterState.PGA_REG & 0b11011111)  | (enabled<<5));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	return;
}
	
void NAU7802::setPGAbypass(bool enabled){
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = ((currentRegisterState.PGA_REG & 0b11101111)  | (enabled<<4));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	return;
}
	
void NAU7802::setPGAinputInversion(bool enabled){
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = ((currentRegisterState.PGA_REG & 0b11110111)  | (enabled<<3));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	return;
}

void NAU7802::setPGAchopper(bool enabled){
	currentRegister = PGA_REG;
	currentRegisterState.PGA_REG = ((currentRegisterState.PGA_REG & 0b111111110)  | (~enabled & 1));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PGA_REG);
	Wire.endTransmission();
	return;
}

void NAU7802::setPGAbypassCAP(bool enabled){
	currentRegister = PWR_CTRL;
	currentRegisterState.PWR_CTRL = ((currentRegisterState.PWR_CTRL & 0b01111111)  | (enabled << 7));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PWR_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setMasterBiasCurrent(uint8_t current){
	currentRegister = PWR_CTRL;
	currentRegisterState.PWR_CTRL = ((currentRegisterState.PWR_CTRL & 0b10001111)  | (current << 4));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PWR_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setADCcurrent(uint8_t current){
	currentRegister = PWR_CTRL;
	currentRegisterState.PWR_CTRL = ((currentRegisterState.PWR_CTRL & 0b11110011)  | (current << 2));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PWR_CTRL);
	Wire.endTransmission();
	return;
}

void NAU7802::setPGAcurrent(uint8_t current){
	currentRegister = PWR_CTRL;
	currentRegisterState.PWR_CTRL = ((currentRegisterState.PWR_CTRL & 0b01111100)  | (current));
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(currentRegisterState.PWR_CTRL);
	Wire.endTransmission();
	return;
}
uint8_t NAU7802::readRegister(registerAddress address){
	currentRegister = address;
	uint8_t registerValue;
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.endTransmission(false);
	Wire.requestFrom(deviceAddress, uint8_t(1));
	registerValue = Wire.read();
	Wire.endTransmission();
	return registerValue;
}

void NAU7802::setRegister(registerAddress address, uint8_t regValue){
	currentRegister = address;
	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.write(regValue);
	Wire.endTransmission();
	return;
}

void NAU7802::bootCycle(void){
	NAU7802::registerReset(true);
	delay(10);
	NAU7802::registerReset(false);
	NAU7802::powerDigital();
	delay(5);
	NAU7802::powerAnalog();
}
#endif

uint32_t NAU7802::readLoad(){//This function reads the results register and returns it as an unsigned 32-bit integer of which only 24-bits are used. It requests and reads three words at a time.
	currentRegister = ADC_RESULT;

	Wire.beginTransmission(deviceAddress);
	Wire.write(currentRegister);
	Wire.endTransmission(false);
	Wire.requestFrom(deviceAddress, uint8_t(3));
	for (int8_t i = 2; i >= 0; i--){
		currentReading.bytes[i] = Wire.read();
		// Serial.println(currentReading.bytes[i]);
	}
	Wire.endTransmission();
	return currentReading.Load;
}

uint8_t NAU7802::readRegister(uint8_t address){//This function outputs the value stored in one register.
	uint8_t registerValue;
	Wire.beginTransmission(deviceAddress);
	Wire.write(address);
	Wire.endTransmission(false);
	Wire.requestFrom(deviceAddress, uint8_t(1));
	registerValue = Wire.read();
	Wire.endTransmission();
	return registerValue;
}

void NAU7802::setRegister(uint8_t address, uint8_t regValue){//This is a generic set register function which will input any 8-bit value into a register
	Wire.beginTransmission(deviceAddress);						//The user must take care to only put in valid values as there is no error checking
	Wire.write(address);										//This function is most useful of those running the lean version of this library.
	Wire.write(regValue);
	Wire.endTransmission();
	return;
}
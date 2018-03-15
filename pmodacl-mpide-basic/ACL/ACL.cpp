/************************************************************************/
/*																		*/
/*	ACL.cpp		--		Definition for ACL library 	    				*/
/*																		*/
/************************************************************************/
/*	Author:		Cristian Fatu											*/
/*	Copyright 2011, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*		This file defines functions for ACL								*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	09/09/2011(CristianF): created										*/
/*	26/06/2012(CristianF): lite version of ACL							*/
/*		- solved bug in GetOffsetG										*/
/*		- implement delay in CalibrateOneAxisGravitational				*/
/*		- remove error processing										*/
/*	02/29/2016(JamesC): fixed ReadAccelG function to account for:		*/
/*		-2's compliment nature of the module							*/
/*		-rearranging the read bytes to be processed in the correct		*/
/*			order (MSB then LSB as opposed to the reverse)				*/
/*		-expanded returned values to included negative signs			*/
/*	-Commented out I2C dependent functions to save program space		*/
/*	-Corrected CalibrateOneAxisGravitational to account for:			*/
/*		-two's compliment nature of measured data						*/
/*		-math on stored offset value calcuation							*/
/*	-added tap detection with interrupt functionality					*/
/*																		*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */
#include "ACL.h"
#include <Dspi.h>
//#include <Wire.h>

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*        ACL::ACLWriteBytesSPI
**
**        Synopsis:
**				ACLWriteBytesSPI(bRegisterAddress, bCntBytes,  rgbValues);
**        Parameters:
**				uint8_t bAddress – the address where the values will be written
**				uint8_t bCntBytes – the number of bytes that will be written
**				uint8_t *rgbValues - the array of values to be written
**
**
**        Return Values:
**                void 
**
**        Errors:
**			If module is not initialized (using begin), the function does nothing
**
**        Description:
**			This function writes the values from the buffer to the specified number of registers starting from a specified address value. 
**				It performs the SPI write cycle for the specified array of values to the specified address.
**
**
*/
void ACL::WriteBytesSPI(uint8_t bAddress, uint8_t bCntBytes, uint8_t *rgbValues)
{
	if(pdspi != NULL)
	{
		// make SS active
		digitalWrite(m_SSPin, LOW);		
		// As requested by documentation, first byte contains:
		//	bit 7 = 0 because is a write operation
		//	bit 6 = 1 if more than one bytes is written, 0 if a single byte is written
		// 	bits 5-0 - the address
		uint8_t bFirstByte = ((bCntBytes > 1) ? 0x40: 0) | (bAddress & 0x3F);
		pdspi->transfer(bFirstByte);
		// transfer the rest of the bytes
		int nIdxBytes;
		for(nIdxBytes = 0; nIdxBytes < bCntBytes; nIdxBytes++)
		{
			pdspi->transfer(rgbValues[nIdxBytes]);
		}
		
		// make SS inactive
		digitalWrite(m_SSPin, HIGH);
	}
}

/* ------------------------------------------------------------ */
/*        ACL::ReadBytesSPI
**
**        Synopsis:
**				ACLReadBytesSPI(bRegisterAddress, bCntBytes, rgbValues);
**        Parameters:
**				uint8_t bAddress – the address from where the values will be read
**				uint8_t bCntBytes – the number of bytes that will be read
**				uint8_t *rgbValues - the array where values will be read
**
**
**        Return Values:
**                void 
**
**        Errors:
**			If module is not initialized (using begin), the function does nothing
**
**        Description:
**				This function will read the specified number of registers starting from a specified address and store their values in the buffer. 
**					It performs the SPI read cycle from the specified address into the specified array of values.
**
**
*/
void ACL::ReadBytesSPI(uint8_t bAddress, uint8_t bCntBytes, uint8_t *rgbValues)
{
	if(pdspi != NULL)
	{
		uint8_t blank = 0;
		// make SS active
		digitalWrite(m_SSPin, LOW);

		// As requested by documentation, first byte contains:
		//	bit 7 = 1 because is a read operation
		//	bit 6 = 1 if more than one bytes is written, 0 if a single byte is written
		// 	bits 5-0 - the address
		uint8_t bFirstByte = (bCntBytes > 1 ? 0xE0: 0x80) | (bAddress & 0x3F);
		pdspi->transfer(bFirstByte);

		// transfer the rest of the bytes
		int nIdxBytes;
		for(nIdxBytes = 0; nIdxBytes < bCntBytes; nIdxBytes++)
		{
			rgbValues[nIdxBytes] = pdspi->transfer(blank);
		}
		
		// make SS inactive
		digitalWrite(m_SSPin, HIGH);
	}
}



/* ------------------------------------------------------------ */
/*        ACL::WriteBytesI2C
**
**        Synopsis:
**				ACLWriteBytesI2C(bRegisterAddress, bCntBytes,  rgbValues);
**        Parameters:
**				uint8_t bAddress – the address where the values will be written
**				uint8_t bCntBytes – the number of bytes that will be written
**				uint8_t *rgbValues - the array of values to be written
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**			This function writes the values from the buffer to the specified number of registers starting from a specified address value. 
**				It performs the I2C write cycle for the specified array of values to the specified address.
**
**
*/
/*
void ACL::WriteBytesI2C(uint8_t bAddress, uint8_t bCntBytes, uint8_t *rgbValues)
{
    Wire.beginTransmission(ACL_I2C_ADDR); //start transmission to device 
    Wire.send(bAddress);        // send register address
	int nIdxBytes;
	for(nIdxBytes = 0; nIdxBytes < bCntBytes; nIdxBytes++)
	{
		Wire.send(rgbValues[nIdxBytes]); // send value to write
	}	
	Wire.endTransmission(); //end transmission
}
*/




/* ------------------------------------------------------------ */
/*        ACL::ReadBytesI2C
**
**        Synopsis:
**				ACLReadBytesI2C(bRegisterAddress, bCntBytes, rgbValues);
**        Parameters:
**				uint8_t bAddress – the address from where the values will be read
**				uint8_t bCntBytes – the number of bytes that will be read
**				uint8_t *rgbValues - the array where values will be read
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function will read the specified number of registers starting from a specified address and store their values in the buffer. 
**					It performs the I2C read cycle from the specified address into the specified array of values.
**
**
*/
/*
void ACL::ReadBytesI2C(uint8_t bAddress, uint8_t bCntBytes, uint8_t *rgbValues)
{
	Wire.beginTransmission(ACL_I2C_ADDR); 	//start transmission to idDevice 
	Wire.send(bAddress);        			//send address to read from
	Wire.endTransmission(); 				//end transmission
                 		
	Wire.beginTransmission(ACL_I2C_ADDR); 	//start transmission to idDevice (initiate again)
	Wire.requestFrom(ACL_I2C_ADDR, (int)bCntBytes);    // request bCntBytes bytes from idDevice
                	
	int nIdxBytes = 0;
	while(Wire.available())    //idDevice may send less than requested (abnormal)
	{ 
		rgbValues[nIdxBytes] = Wire.receive(); // receive a byte
		nIdxBytes++;
	}
	Wire.endTransmission(); //end transmission
}
*/



/* ------------------------------------------------------------ */
/*        ACL::WriteRegisters
**
**        Synopsis:
**				ACLWriteRegisters(bRegisterAddress, 1, &bRegValue);
**        Parameters:
**				uint8_t bRegisterAddress – the address where the values will be written
**				uint8_t bCntBytes – the number of bytes that will be written
**				uint8_t *rgbValues - the array of values to be written**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**			This function writes the values from the buffer to the specified number of registers starting from a specified address value. 
**			It is a low level function and will be used by a lot of higher level functions. According to the chosen access platform (SPI or I2C), 
**			it will direct the call to the specific function (ACLWriteBytesSPI or ACLWriteBytesI2C).
**
**
*/
void ACL::WriteRegisters(uint8_t bRegisterAddress, uint8_t bCntBytes, uint8_t *rgbValues)
{
	if(m_bAccessType == PAR_ACCESS_I2C)
	{
//		WriteBytesI2C(bRegisterAddress, bCntBytes,  rgbValues);
	}
	else
	{
		WriteBytesSPI(bRegisterAddress, bCntBytes,  rgbValues);
	}
}


/* ------------------------------------------------------------ */
/*        ACL::ReadRegisters
**
**        Synopsis:
**				ReadRegisters(bRegisterAddress, 1, &bRegValue);
**        Parameters:
**				uint8_t bRegisterAddress – the address from where the values will be read
**				uint8_t bCntBytes – the number of bytes that will be read
**				uint8_t *rgbValues - the array where values will be read
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**			This function will read the specified number of registers starting from a specified address and store their values in the buffer. 
**			It is a low level function and will be used by a lot of higher level functions. 
**			According to the chosen access platform (SPI or I2C), it will direct the call to the specific function (ReadBytesSPI or ReadBytesI2C).
**
**
*/
void ACL::ReadRegisters(uint8_t bRegisterAddress, uint8_t bCntBytes, uint8_t *rgbValues)
{
	if(m_bAccessType == PAR_ACCESS_I2C)
	{
		rgbValues[0] = 0x12;
//		ReadBytesI2C(bRegisterAddress, bCntBytes, rgbValues);
	}
	else
	{
		ReadBytesSPI(bRegisterAddress, bCntBytes, rgbValues);
	}
}

/* ------------------------------------------------------------ */
/*        ACL::SetRegisterBits
**
**        Synopsis:
**				ACLSetRegisterBits(ACL_ADR_POWER_CTL, bPowerControlMask, fValue);
**        Parameters:
**				uint8_t bRegisterAddress 	- the address of the register whose bits are set
**				uint8_t bMask				- the mask indicating which bits are affected
**				bool fValue					- 1 if the bits are set or 0 if their bits are reset
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function sets the value of some bits (corresponding to the bMask) of a register (indicated by bRegisterAddress) to 1 or 0 (indicated by fValue).
**
**
*/
void ACL::SetRegisterBits(uint8_t bRegisterAddress, uint8_t bMask, bool fValue)
{
	uint8_t bRegValue;
	ReadRegisters(bRegisterAddress, 1, &bRegValue);
	if(fValue)
	{
		// set 1 value to the values that are 1 in the mask
		bRegValue |= bMask;
	}
	else
	{
		// set 0 value to the values that are 1 in the mask
		bRegValue &= ~bMask;
	}
	WriteRegisters(bRegisterAddress, 1, &bRegValue);
}


/* ------------------------------------------------------------ */
/*        ACL::GetRegisterBits
**
**        Synopsis:
**				return ACLGetRegisterBits(ACL_ADR_BW_RATE, MSK_BW_RATE_RATE);
**        Parameters:
**				uint8_t bRegisterAddress 	- the address of the register whose bits are read
**				uint8_t bMask				- the mask indicating which bits are read
**
**
**        Return Values:
**                uint8_t 					- a byte containing only the bits correspoding to the mask.
**
**        Errors:
**
**
**        Description:
**				Returns a byte containing only the bits from a register (indicated by bRegisterAddress), correspoding to the bMask mask.
**
**
*/
uint8_t ACL::GetRegisterBits(uint8_t bRegisterAddress, uint8_t bMask)
{
	uint8_t bRegValue;
	ReadRegisters(bRegisterAddress, 1, &bRegValue);
	return bRegValue & bMask;
}

/* ------------------------------------------------------------ */
/*        ACL::ConvertReadingToValueG
**
**        Synopsis:
**				*pdAclXg = ACLConvertReadingToValueG(rgwRegVals[0]);
**        Parameters:
**				int16_t uiReading	- the 2 bytes containing the reading (in fact only 10 bits are used).
**
**
**        Return Values:
**                float - the value of the acceleration in "g" corresponding to the 10 bits reading and the current g range
**
**        Errors:
**
**
**        Description:
**				Converts the value from the 10 bits reading to the float value (in g) corresponding to the acceleration, considering the current selected g range.
**	
**
**
*/
float ACL::ConvertReadingToValueG(int16_t uiReading)
{
	//Convert the accelerometer value to G's. 
  //With 10 (ACL_NO_BITS) bits measuring over a +/- ng range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10))
  // m_dGRangeLSB is pre-computed in ACL::SetGRange
  float dResult = ((float)uiReading) * m_dGRangeLSB;
  //Serial.print("9 bit reading is is: "); Serial.println(uiReading);
  //Serial.print("LSB value is: "); Serial.println(m_dGRangeLSB);
  //Serial.print("result is: "); Serial.println(dResult);
  return dResult;
}

/* ------------------------------------------------------------ */
/*        ACL::GetGRangeLSB
**
**        Synopsis:
**				m_dGRangeLSB = GetGRangeLSB(bGRange);
**        Parameters:
**				uint8_t bGRange	- the parameter specifying the g range. Can be one of the parameters in the following list
**					0	PAR_GRANGE_PM2G	Parameter g range : +/- 2g
**					1	PAR_GRANGE_PM4G	Parameter g range : +/- 4g
**					2	PAR_GRANGE_PM8G	Parameter g range : +/- 8g
**					3	PAR_GRANGE_PM16G Parameter g range : +/- 16g
**
**        Return Values:
**                float - the value in "g" corresponding to the G range parameter, that corresponds to 1 accelerometer LSB 
**
**        Errors:
**
**
**        Description:
**				Converts the parameter indicating the G range into the value that corresponds to 1 accelerometer LSB. 
**					For ex PAR_GRANGE_PM8G: Range is 16g, accelerometer is on 10 bits, that corresponds to 16/(2^10). This constant is later used in converting readings to acceleration values in g.  
**					(for ex converts PAR_GRANGE_PM8G into 8).
**
**
*/
float ACL::GetGRangeLSB(uint8_t bGRange)
{
	float dGMaxValue = 0;
	float dResult;
	switch(bGRange)
	{
		case PAR_GRANGE_PM2G:
			dGMaxValue = 2;
			break;
		case PAR_GRANGE_PM4G:
			dGMaxValue = 4;
			break;
		case PAR_GRANGE_PM8G:
			dGMaxValue = 8;
			break;
		case PAR_GRANGE_PM16G:
			dGMaxValue = 16;
			break;
	}
	dResult = 2 * dGMaxValue / (float)(1 << ACL_NO_BITS);
	//Serial.print("LSB is: "); Serial.println(dResult);
	//Serial.print("scaled LSB is: "); Serial.println(dResult * 1000.0);
	return dResult;
}

/* ------------------------------------------------------------ */
/*        ACL::ACL
**
**        Synopsis:
**				
**        Parameters:
**
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**			Class constructor. Performs variables initialization tasks
**
**
*/
ACL::ACL()
{
	pdspi = NULL;
	m_bAccessType = -1;
	m_dGRangeLSB = 0;
}





/* ------------------------------------------------------------ */
/*        ACL::begin
**
**        Synopsis:
**				myACL.begin(PAR_ACCESS_I2C);
**        Parameters:
**				uint8_t bAccessType	- the type of the communication with the PMod. Can be one of:
**					0	PAR_ACCESS_SPI0	- indicates SPI access, port corresponding to DSPI0
**					1	PAR_ACCESS_SPI1	- indicates SPI access, port corresponding to DSPI1
**					2	PAR_ACCESS_SPI1	- indicates SPI access, port corresponding to DSPI2
**					10	PAR_ACCESS_I2C	- indicates I2C access, port corresponding to default Wire interface
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function performs the required ACL initialization tasks:
**					-	Initializes and configures the SPI or I2C communication instance. It sets the default SPI frequency to 1 MHz.
**					-	Configures the required signals as outputs
**					-	Other initialization tasks 
**
**
*/
void ACL::begin(uint8_t bAccessType)
{
	m_bAccessType = bAccessType;
	m_dGRangeLSB = GetGRangeLSB(PAR_GRANGE_PM2G);	// the startup range for the ADXL345 accelerometer is +/- 2g
	if(m_bAccessType == PAR_ACCESS_I2C)
	{
//		Wire.begin();
	}
	else
	{
#if (NUM_DTWI_PORTS > 0)
		if(m_bAccessType == PAR_ACCESS_DSPI0)
		{
			pdspi = new DSPI0();
			m_SSPin = PIN_DSPI0_SS;	// default SS pin for DSPI0
		}
#endif	
#if (NUM_DTWI_PORTS > 1)
		if(m_bAccessType == PAR_ACCESS_DSPI1)
		{
			pdspi = new DSPI1();
			m_SSPin = PIN_DSPI1_SS;	// default SS pin for DSPI1
		}
#endif	
#if (NUM_DTWI_PORTS > 2)
		if(m_bAccessType == PAR_ACCESS_DSPI2)
		{
			pdspi = new DSPI2();
			m_SSPin = PIN_DSPI2_SS;	// default SS pin for DSPI2
		}
#endif	
		pdspi->begin(m_SSPin);	// this defines SS pin as output, sets SS as high
		pdspi->setMode(DSPI_MODE3);
		pdspi->setSpeed(ACL_SPI_FREQ);
	}
}


/* ------------------------------------------------------------ */
/*        ACL::end
**
**        Synopsis:
**				myACL.end();
**        Parameters:
**
**
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function performs the required ACL deinitialization tasks.
**
**
*/
void ACL::end()
{
	if(pdspi != NULL)
	{
		delete pdspi;
		pdspi = NULL;
	}
	m_bAccessType = -1;
	m_dGRangeLSB = 0;
}



/* ------------------------------------------------------------ */
/*        ACL::ReadAccelG
**
**        Synopsis:
**				myACL.ReadAccelG(dX, xNeg, dY, yNeg, dZ, zNeg);
**        Parameters:
**				float &dAclXg	- the output parameter that will receive acceleration on X axis (in "g")
**				float &dAclYg	- the output parameter that will receive acceleration on Y axis (in "g")
**				float &dAclZg	- the output parameter that will receive acceleration on Z axis (in "g")
**				bool  &xNeg		- the output parameter that indicates if the x-axis value is negative
**				bool  &yNeg		- the output parameter that indicates if the y-axis value is negative
**				bool  &zNeg		- the output parameter that indicates if the z-axis value is negative
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function is the main function used for acceleration reading, providing the 3 current acceleration values in “g”. 
**					-	It reads simultaneously the acceleration on three axes in a buffer of 6 bytes using the ReadRegister function
**					-	For each of the three axes, combines the two bytes in order to get a 10-bit value
**					-	For each of the three axes, converts the 10-bit value to the value expressed in “g”, considering the currently selected g range

**
**
*/
void ACL::ReadAccelG(float &dAclXg, bool &xNeg, float &dAclYg, bool &yNeg, float &dAclZg, bool &zNeg) 
{
	uint16_t rgwRegVals[3];
	bool coordSigns[3] = {0,0,0};
	uint8_t tempLSB = 0;
	uint8_t tempMSB = 0;
	uint16_t temp = 0;
	ReadRegisters(ACL_ADR_DATAX0, 6, (uint8_t *)rgwRegVals);
	for(int i=0; i<3; i++){
		//Serial.print("i is: "); Serial.println(i);
		//Serial.print("measured value is first: "); Serial.println(rgwRegVals[i], DEC);
		tempLSB = (rgwRegVals[i] >> 8);
		tempMSB = rgwRegVals[i];
		//Serial.print("tempMSB is: "); Serial.println(tempMSB, DEC);
		//Serial.print("tempLSB is: "); Serial.println(tempLSB, DEC);
		//tempMSB = tempMSB | 0b1111111111111111;
		rgwRegVals[i] = tempMSB;
		//Serial.print("measured msb value is now: "); Serial.println(rgwRegVals[i]);
		rgwRegVals[i] = (rgwRegVals[i]<<8); 
		//Serial.print("measured shifted msb value is now: "); Serial.println(rgwRegVals[i]);
		temp = tempLSB;
		//Serial.print("temp is: "); Serial.println(temp, DEC);
		rgwRegVals[i] = rgwRegVals[i] | temp;
		
		if(rgwRegVals[i] > 0x8000){
			coordSigns[i] = 1;
			//if we have a negative value, flip the 9 relevant bits to be positive
			rgwRegVals[i] = rgwRegVals[i] ^ 0x01ff;
			//subtract 1 to get binary
			rgwRegVals[i] = rgwRegVals[i] - 1;
		}
		//now that we have set the negative bool (or not) set the upper 7 bits to zeroes
		//This sets the sign bit and extensions to match positive value to ensure accurate calcuation conversions
		rgwRegVals[i] = rgwRegVals[i] & 0x01FF;
		
		
		//Serial.print("measured combined value is now: "); Serial.println(rgwRegVals[i]);
	}
	//things that need to be changed
	//need to adjust the function below so that they multiply by the appropriate LSB
	//also need to change this function so that it returns if the answer is negative
	//after determining sign, need to clear upper 
	dAclXg = ConvertReadingToValueG(rgwRegVals[0]);
	dAclYg = ConvertReadingToValueG(rgwRegVals[1]);
	dAclZg = ConvertReadingToValueG(rgwRegVals[2]);
	xNeg = coordSigns[0];
	yNeg = coordSigns[1];
	zNeg = coordSigns[2];
}

/* ------------------------------------------------------------ */
/*        ACL::ReadAccelG
**
**        Synopsis:
**				myACL.ReadAccelG(dX, dY, dZ);
**        Parameters:
**				float &dAclXg	- the output parameter that will receive acceleration on X axis (in "g")
**				float &dAclYg	- the output parameter that will receive acceleration on Y axis (in "g")
**				float &dAclZg	- the output parameter that will receive acceleration on Z axis (in "g")
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function is the overloaded version of the main function (above) used for acceleration reading, providing the 3 current acceleration values in “g”. 
**					-	It reads simultaneously the acceleration on three axes in a buffer of 6 bytes using the ReadRegister function
**					-	For each of the three axes, combines the two bytes in order to get a 10-bit value
**					-	For each of the three axes, converts the 10-bit value to the value expressed in “g”, considering the currently selected g range

**
**
*/
void ACL::ReadAccelG(float &dAclXg, float &dAclYg, float &dAclZg) 
{
	uint16_t rgwRegVals[3];
	uint8_t tempLSB = 0;
	uint8_t tempMSB = 0;
	uint16_t temp = 0;
	ReadRegisters(ACL_ADR_DATAX0, 6, (uint8_t *)rgwRegVals);
	for(int i=0; i<3; i++){
		tempLSB = (rgwRegVals[i] >> 8);
		tempMSB = rgwRegVals[i];
		rgwRegVals[i] = tempMSB;
		rgwRegVals[i] = (rgwRegVals[i]<<8); 
		temp = tempLSB;
		rgwRegVals[i] = rgwRegVals[i] | temp;
		
		if(rgwRegVals[i] > 0x8000){
			//if we have a negative value, flip the 9 relevant bits to be positive
			rgwRegVals[i] = rgwRegVals[i] ^ 0x01ff;
			//subtract 1 to get binary
			rgwRegVals[i] = rgwRegVals[i] - 1;
		}
		//now that we have set the negative bool (or not) set the upper 7 bits to zeroes
		//This sets the sign bit and extensions to match positive value to ensure accurate calcuation conversions
		rgwRegVals[i] = rgwRegVals[i] & 0x01FF;
	}
	//things that need to be changed
	//need to adjust the function below so that they multiply by the appropriate LSB
	//also need to change this function so that it returns if the answer is negative
	//after determining sign, need to clear upper 
	dAclXg = ConvertReadingToValueG(rgwRegVals[0]);
	dAclYg = ConvertReadingToValueG(rgwRegVals[1]);
	dAclZg = ConvertReadingToValueG(rgwRegVals[2]);
}

/* ------------------------------------------------------------ */
/*        ACL::ReadAccel
**
**        Synopsis:
**				myACL.ReadAccel(iX, iY, iZ);
**        Parameters:
**				int16_t &iAclX	- the output parameter that will receive acceleration on X axis - 10 bits signed value
**				int16_t &iAclY	- the output parameter that will receive acceleration on Y axis - 10 bits signed value
**				int16_t &iAclZ	- the output parameter that will receive acceleration on Z axis - 10 bits signed value
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				This function provides the 3 "raw" 10-bit values read from the accelerometer. 
**					-	It reads simultaneously the acceleration on three axes in a buffer of 6 bytes using the ReadRegister function
**					-	For each of the three axes, combines the two bytes in order to get a 10-bit value
**
**
*/
void ACL::ReadAccel(int16_t &iAclX, int16_t &iAclY, int16_t &iAclZ) 
{
	
	uint16_t rgwRegVals[3];
	ReadRegisters(ACL_ADR_DATAX0, 6, (uint8_t *)rgwRegVals);
	iAclX = rgwRegVals[0];
	iAclY = rgwRegVals[1];
	iAclZ = rgwRegVals[2];
}

/* ------------------------------------------------------------ */
/*        ACL::SetMeasure
**
**        Synopsis:
**				myACL.SetMeasure(true);
**
**        Parameters:
**				bool fMeasure	– the value to be set for MEASURE bit of POWER_CTL register
**
**
**
**        Return Values:
**
**
**        Description:
**				This function sets the MEASURE bit of POWER_CTL register. This toggles between measurement and standby mode.
**
*/
void ACL::SetMeasure(bool fMeasure)
{
	SetRegisterBits(ACL_ADR_POWER_CTL, MSK_POWER_CTL_MEASURE, fMeasure);
}

/* ------------------------------------------------------------ */
/*        ACL::SetMeasure
**
**        Synopsis:
**				fMeasure = myACL.GetMeasure();
**
**        Parameters:
**
**
**
**        Return Values:
**				bool – the value of the MEASURE bit of POWER_CTL register
**
**
**        Description:
**				This function returns the value of MEASURE bit of POWER_CTL register.  
**
*/
bool ACL::GetMeasure()
{
	return (GetRegisterBits(ACL_ADR_POWER_CTL, MSK_POWER_CTL_MEASURE) != 0);
}

/* ------------------------------------------------------------ */
/*        ACL::SetGRange
**
**        Synopsis:
**					myACL.SetGRange(PAR_GRANGE_PM2G);
**        Parameters:
**				uint8_t bGRangePar	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_GRANGE_PM2G	Parameter g range : +/- 2g
**					1	PAR_GRANGE_PM4G	Parameter g range : +/- 4g
**					2	PAR_GRANGE_PM8G	Parameter g range : +/- 8g
**					3	PAR_GRANGE_PM16G Parameter g range : +/- 16g
**
**
**        Return Value:
**
**
**        Description:
**				The function sets the appropriate g range bits in the DATA_FORMAT register. The accepted argument values are between 0 and 3.
**				If the argument is within the accepted values range, it sets the g range bits in DATA_FORMAT register and ACL_ERR_SUCCESS status is returned.
**				If value is outside this range no value is set.
**
*/
 void ACL::SetGRange(uint8_t bGRangePar)
{
	uint8_t bResult;
	m_dGRangeLSB = GetGRangeLSB(bGRangePar);
	
	SetRegisterBits(ACL_ADR_DATA_FORMAT, MSK_DATA_FORMAT_RANGE0, (bGRangePar & 1));
	SetRegisterBits(ACL_ADR_DATA_FORMAT, MSK_DATA_FORMAT_RANGE1, (bGRangePar & 2) >> 1);
}

/* ------------------------------------------------------------ */
/*        ACL::GetGRange
**
**        Synopsis:
**
**        Parameters:
**
**
**        Return Values:
**                uint8_t - the value specifying the g Range parameter. Can be one of the values from the following list
**					0	PAR_GRANGE_PM2G	Parameter g range : +/- 2g
**					1	PAR_GRANGE_PM4G	Parameter g range : +/- 4g
**					2	PAR_GRANGE_PM8G	Parameter g range : +/- 8g
**					3	PAR_GRANGE_PM16G Parameter g range : +/- 16g
**
**        Errors:
**
**
**        Description:
**				The function returns the value specifying the g range parameter. It relies on the data in DATA_FORMAT register. 
`**
*/
uint8_t ACL::GetGRange()
{
	uint8_t bVal = (GetRegisterBits(ACL_ADR_DATA_FORMAT, MSK_DATA_FORMAT_RANGE1) << 1) + GetRegisterBits(ACL_ADR_DATA_FORMAT, MSK_DATA_FORMAT_RANGE0);
	return bVal;
}

/* ------------------------------------------------------------ */
/*        ACL::SetOffsetG
**
**        Synopsis:
**				myACL.SetOffsetG(PAR_AXIS_Z, 0.5);
**
**        Parameters:
**				uint8_t bAxisParam - byte indicating the axis whose offset will be set. Can be one of:
**					PAR_AXIS_X		- indicating X axis				
**                  PAR_AXIS_Y 		- indicating Y axis
**                  PAR_AXIS_Z 		- indicating Z axis
**
**				float dOffsetX	– the offset for X axis in “g”
**
**        Return Values:
**
**
**        Description:
**				This function sets the specified axis offset, the value being given in “g”. The accepted argument values are between -2g and +2g.  
**				If argument is within the accepted values range, its value is quantified in the 8-bit offset register using a scale factor of 15.6 mg/LSB and ACL_ERR_SUCCESS is returned.
**				If value is outside this range or if bAxisParam parameter is outside 0 - 3 range, the function does nothing.
**
*/
void ACL::SetOffsetG(uint8_t bAxisParam, float dOffset)
{
	int8_t bOffsetVal = (uint8_t)(dOffset/(float)ACL_CONV_OFFSET_G_LSB);
	switch (bAxisParam)
	{
		case PAR_AXIS_X:
			WriteRegisters(ACL_ADR_OFSX, 1, (uint8_t *)&bOffsetVal);
			break;
		case PAR_AXIS_Y:
			WriteRegisters(ACL_ADR_OFSY, 1, (uint8_t *)&bOffsetVal);
			break;
		case PAR_AXIS_Z:
			WriteRegisters(ACL_ADR_OFSZ, 1, (uint8_t *)&bOffsetVal);
			break;				
	}
}


/* ------------------------------------------------------------ */
/*        ACL::GetOffsetG
**
**        Synopsis:
**				dOffsetG = myACL.GetOffsetG(PAR_AXIS_X);
**
**        Parameters:
**				uint8_t bAxisParam - byte indicating the axis whose acceleration will be read. Can be one of:
**					PAR_AXIS_X		- indicating X axis				
**                  PAR_AXIS_Y 		- indicating Y axis
**                  PAR_AXIS_Z 		- indicating Z axis
**
**
**
**        Return Values:
**                float 	- the offset for X axis in “g”.
**
**        Errors:
**
**
**        Description:
**				This function returns the offset, in “g”, for the specified axis.  
**				It converts the 8-bit value quantified in the offset register into a value expressed in “g”, using a scale factor of 15.6 mg/LSB.
**
**
*/
float ACL::GetOffsetG(uint8_t bAxisParam)
{
	int8_t bOffsetVal; 
	float dResult;
	switch (bAxisParam)
	{
		case PAR_AXIS_X:
			ReadRegisters(ACL_ADR_OFSX, 1, (uint8_t *)&bOffsetVal);
			break;
		case PAR_AXIS_Y:
			ReadRegisters(ACL_ADR_OFSY, 1, (uint8_t *)&bOffsetVal);
			break;
		case PAR_AXIS_Z:
			ReadRegisters(ACL_ADR_OFSZ, 1, (uint8_t *)&bOffsetVal);
			break;				
	}
	dResult = (float)bOffsetVal * (float)ACL_CONV_OFFSET_G_LSB;
	return dResult;
}

/* ------------------------------------------------------------ */
/*        ACL::CalibrateOneAxisGravitational
**
**        Synopsis:
**				myACL.CalibrateOneAxisGravitational(PAR_AXIS_ZP);
**        Parameters:
**				uint8_t bAxisInfo - Parameter specifying axes orientation. Can be one of the following:
**					0	PAR_AXIS_XP - X axis is oriented in the gravitational direction
**					1	PAR_AXIS_XN - X axis is oriented in the opposite gravitational direction
**					2	PAR_AXIS_YP - Y axis is oriented in the gravitational direction
**					3	PAR_AXIS_YN - Y axis is oriented in the opposite gravitational direction
**					4	PAR_AXIS_ZP - Z axis is oriented in the gravitational direction
**					5	PAR_AXIS_ZN - Z axis is oriented in the opposite gravitational direction
**
**        Return Value:
**
**        Errors:
**
**
**        Description:
**				The accepted argument values are between 0 and +5.
**				This function performs the calibration of the accelerometer by setting the offset registers in the following manner: 
**				 computes the correction factor that must be loaded in the offset registers so that the acceleration readings are:
**					1 for the gravitational axis, if positive orientation
**					-1 for the gravitational axis, if negative orientation
**					0 for the other axes
**				The accepted argument values are between 0 and 5.
**				If the argument value is outside this range, the function does nothing.
**
*/
void ACL::CalibrateOneAxisGravitational(uint8_t bAxisInfo)
{
		// perform calibration
		float dX, dSumX = 0, dY, dSumY = 0, dZ, dSumZ = 0;
		bool xNeg=0, yNeg=0, zNeg=0;
		// set the offset registers to 0
		//Put the device into standby mode to configure it.
		SetMeasure(false);
		SetOffsetG(PAR_AXIS_X, 0);
		SetOffsetG(PAR_AXIS_Y, 0);
		SetOffsetG(PAR_AXIS_Z, 0);
		SetMeasure(true);


		// read average acceleration on the three axes
		int idxAvg;

		int nCntMeasurements = 128;
		// consume some readings
		for(idxAvg = 0; idxAvg < nCntMeasurements; idxAvg++)
		{
			ReadAccelG(dX, dY, dZ);
		}
		//taking advantange of the fact the module is likely not moving
		// determine the sign orientation of each axis with a single reading
		ReadAccelG(dX, xNeg, dY, yNeg, dZ, zNeg);
		// compute average values
		for(idxAvg = 0; idxAvg < nCntMeasurements; idxAvg++)
		{
			ReadAccelG(dX, dY, dZ);
			dSumX = dSumX + dX;
			dSumY = dSumY + dY;
			dSumZ = dSumZ + dZ;
		}
		dX = dSumX/nCntMeasurements;
		dY = dSumY/nCntMeasurements;
		dZ = dSumZ/nCntMeasurements;
		//set the values to + or - as appropriate
		if(xNeg){dX = dX *(-1 * xNeg);}
		if(yNeg){dY = dY *(-1 * yNeg);}	
		if(zNeg){dZ = dZ *(-1 * zNeg);}
		// computes the correction that must be put in the offset registers so that the acceleration readings are:
		//	1 (for the gravitational axis, if positive
		//	-1 (for the gravitational axis, if negative
		// 0 (for the other axes)
		
		//all calculations below switched from (ideal - measured) to (measured - ideal)
		//as described in the Offset Calibration section of the ADXL345 datasheet (pg. 30)
		switch (bAxisInfo)
		{
			case PAR_AXIS_XP:
				dX = dX - 1.0;		//1.0 - dX;
				dY = dY - 0.0;		//0.0 - dY;
				dZ = dY - 0.0;		//0.0 - dZ;
				break;
			case PAR_AXIS_XN:
				dX = dX - (-1.0);	//-1.0 - dX;
				dY = dY - 0.0;		//0.0 - dY;
				dZ = dZ - 0.0;		//0.0 - dZ;
				break;
			case PAR_AXIS_YP:
				dY = dY - 1.0;		//1.0 - dY;
				dX = dX - 0.0;		//0.0 - dX;
				dZ = dZ - 0.0;		//0.0 - dZ;
				break;
			case PAR_AXIS_YN:
				dY = dY - (-1.0);	//-1.0 - dY;
				dX = dX - 0.0;		//0.0 - dX;
				dZ = dZ - 0.0;		//0.0 - dZ;
				break;
			case PAR_AXIS_ZP:
				dZ = dZ - 1.0;		//1.0 - dZ;
				dY = dY - 0.0;		//0.0 - dY;
				dX = dX - 0.0;		//0.0 - dX;
				break;
			case PAR_AXIS_ZN:
				dZ = dZ - (-1.0);	//-1.0 - dZ;
				dY = dY - 0.0;		//0.0 - dY;
				dX = dX - 0.0;		//0.0 - dX;
				break;
		}
		//Put the device into standby mode to configure it.
		SetMeasure(false);
		// set the offset data to registers
		SetOffsetG(PAR_AXIS_X, dX);
		SetOffsetG(PAR_AXIS_Y, dY);
		SetOffsetG(PAR_AXIS_Z, dZ);
		SetMeasure(true);
		
		// some delay is needed
		delay(100);

}

/* ------------------------------------------------------------ */
/*        ACL::ConfigureInterrupt
**
**        Synopsis: copied from ACLFULL.cpp
**				
**        Parameters:
**				uint8_t bParACLIntNo	- The parameter indicating the ACL interrupt number. Can be one of the parameters from the following list
**					0		PAR_ACL_INT1
**					1		PAR_ACL_INT2
**
**				uint8_t bParExtIntNo	- The parameter indicating the external interrupt number. Can be one of the parameters from the following list
**					0		PAR_EXT_INT0
**					1		PAR_EXT_INT1
**					2		PAR_EXT_INT2
**					3		PAR_EXT_INT3
**					4		PAR_EXT_INT4
**
**				uint8_t bEventMask	- the events that trigger the interrupt. Can be one or more (OR-ed) parameters from the following list
**					1<<7	MSK_INT_DATA_READY	DATA_READY 
**					1<<6	MSK_INT_SINGLE_TAP	SINGLE_TAP 
**					1<<5	MSK_INT_float_TAP	float_TAP 
**					1<<4	MSK_INT_ACTIVITY	Activity
**					1<<3	MSK_INT_INACTIVITY	Inactivity 
**					1<<2	MSK_INT_FREE_FALL	FREE_FALL 
**					1<<1	MSK_INT_WATERMARK	Watermark 
**					1<<0	MSK_INT_OVERRUN		Overrun
**
**				void (*pfIntHandler)() - pointer to a function that will serve as interrupt handler.
**
**				uint8_t bActiveType	– The parameter indicating the interrupt pin is active high or low. Can be one of the parameters from the following list
**					0		PAR_INT_ACTIVEHIGH
**					1		PAR_INT_ACTIVELOW
**
**        Return Value:
**                uint8_t 
**					- ACL_ERR_SUCCESS (0) 	- The action completed successfully 
**					- a combination (ORed) of the following errors:
**						- ACL_ERR_ARG_AINTNO02_BIT (32) - ACL Interrupt no is not within 0-1 range
**						- ACL_ERR_ARG_EINTNO04_BIT (64) - External Interrupt no is not within 0-4 range 
**						- ACL_ERR_ARG_ACT_BIT (128) - Active is different than 0 or 1 
**
**        Errors:
**
**
**        Description:
**				The function configures the interrupt by 
**					- associating it to a set of events (INT_ENABLE  & INT_MAP registers) 
**					- associating it to an ACL interrupt number (1, 2)
**					- associating it to an external interrupt number (0-4)
**					- associating it to an interrupt handler  
**					- defining the behaviour for the interrupt pin
**				Make sure that interrupt pin of the PmodACL corresponding to the bParACLIntNo parameter is physically connected to external interrupt pin number corresponding to bParExtIntNo parameter.
**				The function expects the parameters bParACLIntNo, bParExtIntNo and bActiveType to be in the specified range.
**				For each parameter outside the range, a specific error is set and a combination (OR-ed) of the errors is returned. 
**				If all parameters are within their range, ACL_ERR_SUCCESS is returned. 
**
*/
uint8_t ACL::ConfigureInterrupt(uint8_t bParACLIntNo, uint8_t bParExtIntNo, uint8_t bEventMask, void (*pfIntHandler)(), uint8_t bActiveType)
{
	uint8_t bResult = ACL_ERR_SUCCESS;
	if(bParACLIntNo < PAR_ACL_INT1 || bParACLIntNo > PAR_ACL_INT2)
	{
		bResult |= ACL_ERR_ARG_AINTNO01_BIT;
	}
	if(bParExtIntNo < PAR_EXT_INT0 || bParExtIntNo > PAR_EXT_INT4)
	{
		bResult |= ACL_ERR_ARG_EINTNO04_BIT;
	}
	if(bActiveType < PAR_INT_ACTIVEHIGH || bActiveType > PAR_INT_ACTIVELOW)
	{
		bResult |= ACL_ERR_ARG_ACT_BIT;
	}
	if(bResult == ACL_ERR_SUCCESS)
	{	attachInterrupt(bParExtIntNo, pfIntHandler, RISING);
		SetRegisterBits(ACL_ADR_INT_MAP, bEventMask, bParACLIntNo == PAR_ACL_INT2);
		SetRegisterBits(ACL_ADR_INT_ENABLE, bEventMask, true);
		SetRegisterBits(ACL_ADR_DATA_FORMAT, MSK_DATA_FORMAT_INTINVERT, bActiveType == PAR_INT_ACTIVELOW);
		uint8_t bVal;
		ReadRegisters(ACL_ADR_INT_SOURCE, 1, &bVal);	// reading the INT_SOURCE register causes interrupt flags to be cleared
	}
	return bResult;
}

/* ------------------------------------------------------------ */
/*        ACL::GetInterruptSourceBits
**
**        Synopsis: taken from ACLFULL.cpp
**
**        Parameters:
**				uint8_t bEventMask	- the events that are checked if they triggered the interrupt. Can be one or more (OR-ed) parameters from the following list
**					1<<7	MSK_INT_DATA_READY	DATA_READY 
**					1<<6	MSK_INT_SINGLE_TAP	SINGLE_TAP 
**					1<<5	MSK_INT_float_TAP	float_TAP 
**					1<<4	MSK_INT_ACTIVITY		Activity
**					1<<3	MSK_INT_INACTIVITY	Inactivity 
**					1<<2	MSK_INT_FREE_FALL		FREE_FALL 
**					1<<1	MSK_INT_WATERMARK		Watermark 
**					1<<0	MSK_INT_OVERRUN		Overrun
**
**
**        Return Values:
**                uint8_t - the value of the register masked using the specified mask
**
**        Errors:
**
**
**        Description:
**				The function returns the value of INT_SOURCE register masked using the provided mask.
**					When defining more events for an interrupt, this function can be used to determine which is causing the interrupt.
**					Note that, according to datasheet, reading the INT_SOURCE register causes interrupt flags to be cleared.
**
*/
uint8_t ACL::GetInterruptSourceBits(uint8_t bEventMask)
{
	return GetRegisterBits(ACL_ADR_INT_SOURCE, bEventMask);
}

/* ------------------------------------------------------------ */
/*        ACL::SetThresholdG
**
**        Synopsis: taken from ACLFULL.cpp
**
**        Parameters:
**				uint8_t bThreshParam - byte indicating the threshold that will be set. Can be one of:
**					PAR_THRESH_TAP			- indicating tap treshold			
**                  PAR_THRESH_ACT	 		- indicating activity treshold
**                  PAR_THRESH_INACT 		- indicating inactivity treshold
**                  PAR_THRESH_FF			- indicating free fall treshold
**
**				float dTreshVal	– parameter containing Threshold in “g”.
**
**
**        Return Values:
**                uint8_t 
**					- ACL_ERR_SUCCESS (0) 	- The action completed successfully 
**					- a combination (ORed) of the following errors:
**						- ACL_ERR_ARG_RANGE_016G (2) - The dTreshVal argument is not within 0 - 16g range
**						- ACL_ERR_INVALID_THRESH (8)- The bThreshParam argument is not within 0 - 3 range
**
**        Errors:
**				see return value
**
**        Description:
**				This function sets the Tap, Activity, Inactivity or Free Fall Threshold, the value being given in “g”. The bThreshParam parameter specify which of the threshold values will be set.
**				The accepted threshold values are between 0 and 16g. If value is outside this range, ACL_ERR_ARG_RANGE_016G is returned and no value is set.
**				If argument is within the accepted values range, its value is quantified in 8-bit threshold register using a scale factor of 62.5 mg/LSB and ACL_ERR_SUCCESS is returned.
**				
**				
**
**
*/
uint8_t ACL::SetThresholdG(uint8_t bThreshParam, float dTreshVal)
{
	uint8_t bErr = ACL_ERR_SUCCESS;
	if(dTreshVal < 0 || dTreshVal > 16)
	{
		bErr |= ACL_ERR_ARG_RANGE_016G;
	}
	if(bThreshParam < PAR_THRESH_TAP || bThreshParam > PAR_THRESH_FF)
	{
		bErr |= ACL_ERR_INVALID_THRESH;
	}
	if(bErr == ACL_ERR_SUCCESS)
	{
		uint8_t bTreshVal = (uint8_t)(dTreshVal/(float)ACL_CONV_TRESH_G_LSB);
		switch(bThreshParam)
		{
			case PAR_THRESH_TAP:
				WriteRegisters(ACL_ADR_THRESH_TAP, 1, &bTreshVal);
				break;
			case PAR_THRESH_ACT:
				WriteRegisters(ACL_ADR_THRESH_ACT, 1, &bTreshVal);
				break;
			case PAR_THRESH_INACT:
				WriteRegisters(ACL_ADR_THRESH_INACT, 1, &bTreshVal);
				break;
			case PAR_THRESH_FF:
				WriteRegisters(ACL_ADR_THRESH_FF, 1, &bTreshVal);
				break;
		}
	}
	return bErr;		
}

/* ------------------------------------------------------------ */
/*        ACL::SetTimeS
**
**        Synopsis:
**				myACL.SetTimeS(PAR_TIME_DUR, 0.01); // duration 10 ms
**        Parameters:
**				uint8_t bTimeParam - byte indicating the time values that will be set. Can be one of:
**					PAR_TIME_DUR		- indicating duration time			
**                  PAR_TIME_LATENT		- indicating latent time
**                  PAR_TIME_WINDOW		- indicating window time
**                  PAR_TIME_FF			- indicating free fall time
**				float dTime			- parameter containing time expressed in seconds. 
**
**
**        Return Value:
**                uint8_t 
**					- ACL_ERR_SUCCESS (0) 	- The action completed successfully 
**					- ACL_ERR_INVALID_TIME (14) 	- The bTimeParam is not in 0 - 3 range	
**					- Depending on bTimeParam, one of:
**						- ACL_ERR_ARG_RANGE_0160MS (2)	- The argument is not within 0 - 160ms range (for PAR_TIME_DUR)
**						- ACL_ERR_ARG_RANGE_DF_0320MS (4)	- The argument is not within 0 - 320ms range (for PAR_TIME_LATENT, PAR_TIME_WINDOW)
**						- ACL_ERR_ARG_RANGE_0128S (5) - The argument is not within 0- 1.28s range (for PAR_TIME_FF)
**
**        Errors:
**				see return value
**
**        Description:
**				This function sets the Duration, Latent, Window and Free Fall time, the value being given in seconds. The bTimeParam parameter specify which of the time values will be set.
**				If bTimeParam is outside the four time values, ACL_ERR_INVALID_TIME will be returned.
**				The accepted dTime argument values depend on the different times:
**					- if Duration time is not between 0 and 0.16s, ACL_ERR_ARG_RANGE_0160MS is returned. Otherwise a scale factor of 625 μs/LSB is used.
**					- if Latent time is not between 0 and 0.32s, ACL_ERR_ARG_RANGE_DF_0320MS is returned. Otherwise a scale factor of 1.25 ms/LSB is used.
**					- if Window time is not between 0 and 0.32s, ACL_ERR_ARG_RANGE_DF_0320MS is returned. Otherwise a scale factor of 1.25 ms/LSB is used.
**					- if Free Fall time is not between 0 and 1.28s, ACL_ERR_ARG_RANGE_0128S is returned. Otherwise a scale factor of 5 ms/LSB is used.
**				If argument is within the accepted values range, its value is quantified in specific 8-bit register using the scale factor described before and ACL_ERR_SUCCESS is returned.
**				If value is outside this range, specific errors are returned and no value is set.
**
**
*/
uint8_t ACL::SetTimeS(uint8_t bTimeParam, float dTime)
{
	uint8_t bResult;
	uint8_t bRegVal;
	switch (bTimeParam)
	{
		case PAR_TIME_DUR:
			if(dTime >= 0 && dTime <= 0.16)
			{
				bRegVal = (uint8_t)(dTime/(float)ACL_CONV_DURATION_S_LSB);
				WriteRegisters(ACL_ADR_DUR, 1, &bRegVal);
				bResult = ACL_ERR_SUCCESS;
			}
			else
			{
				bResult = ACL_ERR_ARG_RANGE_0160MS;
			}		
			break;
		case PAR_TIME_LATENT:
			if(dTime >= 0 && dTime <= 0.32)
			{
				bRegVal = (uint8_t)(dTime/(float)ACL_CONV_LATENT_S_LSB);
				WriteRegisters(ACL_ADR_LATENT, 1, &bRegVal);
				bResult = ACL_ERR_SUCCESS;
			}
			else
			{
				bResult = ACL_ERR_ARG_RANGE_0320MS;
			}
			break;
		case PAR_TIME_WINDOW:
			if(dTime >= 0 && dTime <= 0.32)
			{
				bRegVal = (uint8_t)(dTime/(float)ACL_CONV_WINDOW_S_LSB);
				WriteRegisters(ACL_ADR_WINDOW, 1, &bRegVal);
				bResult = ACL_ERR_SUCCESS;
			}
			else
			{
				bResult = ACL_ERR_ARG_RANGE_0320MS;
			}			
			break;
	/*	case PAR_TIME_FF:
			if(dTime >= 0 && dTime <= 1.28)
			{
				bRegVal = (uint8_t)(dTime/(float)ACL_CONV_TIMEFREEFALL_S_LSB);
				WriteRegisters(ACL_ADR_TIME_FF, 1, &bRegVal);
				bResult = ACL_ERR_SUCCESS;
			}
			else
			{
				bResult = ACL_ERR_ARG_RANGE_0128S;
			}
			break;  */
		default:
			bResult = ACL_ERR_INVALID_TIME;
	}
	return bResult;	
}

/* ------------------------------------------------------------ */
/*        ACL::SetTapAxesBits
**
**        Synopsis:
**
**        Parameters:
**				uint8_t bTapAxesMask	- the mask containing the tap axes bits. There can be one or more (OR-ed) parameters from the list below:
**					1<<3	MSK_TAP_AXES_SUPPRESS	This mask is used in TAP_AXES register for Suppress bit.
**					1<<2	MSK_TAP_AXES_TAPXENABLE	This mask is used in TAP_AXES register for TAP_X bit.
**					1<<1	MSK_TAP_AXES_TAPYENABLE	This mask is used in TAP_AXES register for TAP_Y bit.
**					1<<0	MSK_TAP_AXES_TAPZENABLE	This mask is used in TAP_AXES register for TAP_Z bit.
**				bool fValue
**					-	if true, the bits corresponding to 1 values in the mask will get 1 value 
**					-	if false, the bits corresponding to 1 values in the mask will get 0 value
**
**
**        Return Values:
**                void 
**
**        Errors:
**
**
**        Description:
**				The function sets the appropriate (corresponding to the mask) bits in TAP_AXES register to 0 or 1 value.
**
**
*/
void ACL::SetTapAxesBits(uint8_t bTapAxesMask, bool fValue)
{
	SetRegisterBits(ACL_ADR_TAP_AXES, bTapAxesMask, fValue);
}


uint8_t ACL::GetDevID()
{
	uint8_t bRegValue;
	bool fResult;
	ReadRegisters(ACL_ADR_DEVID, 1, &bRegValue);
	return bRegValue;
}
/* SPDX-License-Identifier: LGPL-2.1 */
#include "MFRC522DriverSPI.h"

#include <avr/io.h>

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522DriverSPI
/////////////////////////////////////////////////////////////////////////////////////

bool MFRC522DriverSPI::init() {
	//4000000u /* 4MHz */, MSBFIRST, SPI_MODE0

	//MOSI, SCK, CS as output
	DDRB = (1 << 7) | (1 << 5);

	//CS high
	DDRD |= (1 << 6);
	PORTD |= (1 << 6);

	//2x operation
	SPSR = (1 << SPI2X);
	//enable, master, F_CPU / 2, mode0, msb first
	SPCR = (1 << SPE) | (1 << MSTR);

	return true;
}

/**
 * Writes a byte to the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverSPI::PCD_WriteRegister(const PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
		const byte value            ///< The value to write.
		) {
	PORTD &= ~(1 << 6);    // Select slave
	// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
	SPDR = (reg << 1);
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF
	uint8_t result = SPDR;
	SPDR = (value);
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF, "| or result" is to stop compiler warning
	result = SPDR | result;
	PORTD |= (1 << 6); // Release slave again
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverSPI::PCD_WriteRegister(const PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
		const byte count,     ///< The number of bytes to write to the register.
		byte *const values        ///< The values to write. Byte array.
		) {
	PORTD &= ~(1 << 6);    // Select slave
	// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
	SPDR = (reg << 1);
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF
	uint8_t result = SPDR;
	for (byte index = 0; index < count; index++) {
		SPDR = (values[index]);
		//wait for complete
		while (!(SPSR & _BV(SPIF))) {
			;
		}
		//read to clear SPIF, "| or result" is to stop compiler warning
		result = SPDR | result;
	}
	PORTD |= (1 << 6); // Release slave again
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
byte MFRC522DriverSPI::PCD_ReadRegister(const PCD_Register reg ///< The register to read from. One of the PCD_Register enums.
		) {
	byte value;

	PORTD &= ~(1 << 6);    // Select slave
	// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
	SPDR = ((byte) 0x80 | (reg << 1));
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF
	value = SPDR;

	SPDR = (0);
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF
	value = SPDR;
	PORTD |= (1 << 6); // Release slave again
	return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522DriverSPI chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522DriverSPI::PCD_ReadRegister(const PCD_Register reg, ///< The register to read from. One of the PCD_Register enums.
		const byte count,            ///< The number of bytes to read.
		byte *const values,        ///< Byte array to store the values in.
		const byte rxAlign ///< Only bit positions rxAlign..7 in values[0] are updated.
		) {
	if (count == 0) {
		return;
	}
	byte address = (byte) 0x80 | (reg << 1); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	byte index = 0;              // Index in values array.

	//count--;								// One read is performed outside of the loop // TODO is this correct?

	PORTD &= ~(1 << 6);    // Select slave
	// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
	SPDR = address;
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF
	byte value = SPDR;

	if (rxAlign) {    // Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		byte mask = (byte) (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.

		SPDR = address;
		//wait for complete
		while (!(SPSR & _BV(SPIF))) {
			;
		}
		//read to clear SPIF
		value = SPDR;
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	//while (index < count) { // changed because count changed to const
	while (index < count - 1) {
		SPDR = address;
		//wait for complete
		while (!(SPSR & _BV(SPIF))) {
			;
		}
		//read to clear SPIF
		values[index] = SPDR; // Read value and tell that we want to read the same address again.
		index++;
	}
	SPDR = 0;
	//wait for complete
	while (!(SPSR & _BV(SPIF))) {
		;
	}
	//read to clear SPIF
	values[index] = SPDR;
} // End PCD_ReadRegister()


byte read_reg(byte reg_addr)
{
	// send adress of the register, with MSBit = 0 to indicate it's a read
	SPI.transfer(mouseSS, reg_addr & 0x7f, SPI_CONTINUE);
	delayMicroseconds(160); // tSRAD
	byte data = SPI.transfer(mouseSS, 0, SPI_CONTINUE);
	delayMicroseconds(20); //tSRR
	return data;
}

void write_reg(byte reg_addr, byte data)
{
	SPI.transfer(mouseSS, reg_addr | 0x80, SPI_CONTINUE);
	SPI.transfer(mouseSS, data, SPI_CONTINUE);
	delayMicroseconds(180); // tSWW/tSWR
}
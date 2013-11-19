#include "TFTLIB.h"

void tft::goTo(int x, int y) {
  writeRegister(0x0020, x);
  writeRegister(0x0021, y);
  writeCommand(0x0022);
}

void tft::fillScreen(uint16_t color) {
	goTo(0, 0);
	uint32_t i;

	i = 76800;
	COMPORT &= ~CS;
	COMPORT |= CD_RD_WR;

  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xff;

	while (i--) {
		write_8(hi);
		write_8(lo);
  }

	COMPORT |= CS;
}

void tft::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
  writeRegister(TFTLCD_GRAM_HOR_AD, x);
  writeRegister(TFTLCD_GRAM_VER_AD, y);
  writeCommand(TFTLCD_RW_GRAM);
  writeData(color);
}

void tft::initDisplay(void) {
	DATADDR = 0xFF;
	COMDDR |= COMMASK
	uint16_t a, d;

	reset();

	for (uint8_t i = 0; i < sizeof(_regValues) / 4; i++) {
		a = pgm_read_word(_regValues + i*2);
		d = pgm_read_word(_regValues + i*2 + 1);

		if (a == TFTLCD_DELAYCMD) {
			_delay_ms(50);
    } else {
			writeRegister(a, d);
    }
  }
}

void tft::reset(void) {
  COMPORT &= ~RT;
  _delay_ms(2);
  COMPORT |= RT;

  writeData(0);
  writeData(0);
  writeData(0);
  writeData(0);
}

void tft::writeData(uint16_t data) {
	COMPORT &= ~CS;
	COMPORT |= CD_RD_WR;

	DATAPORT = (data>>8);
  
	COMPORT &= ~WR;
	COMPORT |= WR;

	DATAPORT = data;

	COMPORT &= ~WR;
	COMPORT |= WR_CS;
}


void tft::writeCommand(uint16_t cmd) {
	COMPORT &= ~CD_CS;
	COMPORT |= RD_WR;

	DATAPORT = (cmd>>8);

	COMPORT &= ~WR;
	COMPORT |= WR;

	DATAPORT = (cmd);

	COMPORT &= ~WR;
	COMPORT |= WR_CS;
}

void tft::writeRegister(uint16_t addr, uint16_t data) {
	writeCommand(addr);
	writeData(data);
}


#include "TFTLIB.h"

#include <util/delay.h>
#include <avr/pgmspace.h>

const uint16_t _regValues[] PROGMEM = {
  TFTLCD_START_OSC, 0x0001,
  TFTLCD_DELAYCMD, 50,
  TFTLCD_DRIV_OUT_CTRL, 0x0100,
  TFTLCD_DRIV_WAV_CTRL, 0x0700,
  TFTLCD_ENTRY_MOD, 0x1030,
  TFTLCD_RESIZE_CTRL, 0x0000,
  TFTLCD_DISP_CTRL2, 0x0202,
  TFTLCD_DISP_CTRL3, 0x0000,
  TFTLCD_DISP_CTRL4, 0x0000,
  TFTLCD_RGB_DISP_IF_CTRL1, 0x0,
  TFTLCD_FRM_MARKER_POS, 0x0,
  TFTLCD_RGB_DISP_IF_CTRL2, 0x0,
  TFTLCD_POW_CTRL1, 0x0000,
  TFTLCD_POW_CTRL2, 0x0007,
  TFTLCD_POW_CTRL3, 0x0000,
  TFTLCD_POW_CTRL4, 0x0000,
  TFTLCD_DELAYCMD, 200,
  TFTLCD_POW_CTRL1, 0x1690,
  TFTLCD_POW_CTRL2, 0x0227,
  TFTLCD_DELAYCMD, 50,
  TFTLCD_POW_CTRL3, 0x001A,
  TFTLCD_DELAYCMD, 50,
  TFTLCD_POW_CTRL4, 0x1800,
  TFTLCD_POW_CTRL7, 0x002A,
  TFTLCD_DELAYCMD,50,
  TFTLCD_GAMMA_CTRL1, 0x0000,
  TFTLCD_GAMMA_CTRL2, 0x0000,
  TFTLCD_GAMMA_CTRL3, 0x0000,
  TFTLCD_GAMMA_CTRL4, 0x0206,
  TFTLCD_GAMMA_CTRL5, 0x0808,
  TFTLCD_GAMMA_CTRL6, 0x0007,
  TFTLCD_GAMMA_CTRL7, 0x0201,
  TFTLCD_GAMMA_CTRL8, 0x0000,
  TFTLCD_GAMMA_CTRL9, 0x0000,
  TFTLCD_GAMMA_CTRL10, 0x0000,
  TFTLCD_GRAM_HOR_AD, 0x0000,
  TFTLCD_GRAM_VER_AD, 0x0000,
  TFTLCD_HOR_START_AD, 0x0000,
  TFTLCD_HOR_END_AD, 0x00EF,
  TFTLCD_VER_START_AD, 0X0000,
  TFTLCD_VER_END_AD, 0x013F,
  TFTLCD_GATE_SCAN_CTRL1, 0xA700,
  TFTLCD_GATE_SCAN_CTRL2, 0x0003,
  TFTLCD_GATE_SCAN_CTRL3, 0x0000,
  TFTLCD_PANEL_IF_CTRL1, 0X0010,
  TFTLCD_PANEL_IF_CTRL2, 0X0000,
  TFTLCD_PANEL_IF_CTRL3, 0X0003,
  TFTLCD_PANEL_IF_CTRL4, 0X1100,
  TFTLCD_PANEL_IF_CTRL5, 0X0000,
  TFTLCD_PANEL_IF_CTRL6, 0X0000,
  TFTLCD_DISP_CTRL1, 0x0133,
};


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


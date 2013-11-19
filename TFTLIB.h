#ifndef __tft__h_
#define __tft__h_

#define F_CPU 16000000UL

#include <avr/io.h>

#define TFTLCD_START_OSC 0x00
#define TFTLCD_DRIV_OUT_CTRL 0x01
#define TFTLCD_DRIV_WAV_CTRL 0x02
#define TFTLCD_ENTRY_MOD 0x03
#define TFTLCD_RESIZE_CTRL 0x04
#define TFTLCD_DISP_CTRL1 0x07
#define TFTLCD_DISP_CTRL2 0x08
#define TFTLCD_DISP_CTRL3 0x09
#define TFTLCD_DISP_CTRL4 0x0A
#define TFTLCD_RGB_DISP_IF_CTRL1 0x0C
#define TFTLCD_FRM_MARKER_POS 0x0D
#define TFTLCD_RGB_DISP_IF_CTRL2 0x0F
#define TFTLCD_POW_CTRL1 0x10
#define TFTLCD_POW_CTRL2 0x11
#define TFTLCD_POW_CTRL3 0x12
#define TFTLCD_POW_CTRL4 0x13
#define TFTLCD_GRAM_HOR_AD 0x20
#define TFTLCD_GRAM_VER_AD 0x21
#define TFTLCD_RW_GRAM 0x22
#define TFTLCD_POW_CTRL7 0x29
#define TFTLCD_FRM_RATE_COL_CTRL 0x2B
#define TFTLCD_GAMMA_CTRL1 0x30
#define TFTLCD_GAMMA_CTRL2 0x31
#define TFTLCD_GAMMA_CTRL3 0x32
#define TFTLCD_GAMMA_CTRL4 0x35 
#define TFTLCD_GAMMA_CTRL5 0x36
#define TFTLCD_GAMMA_CTRL6 0x37
#define TFTLCD_GAMMA_CTRL7 0x38
#define TFTLCD_GAMMA_CTRL8 0x39
#define TFTLCD_GAMMA_CTRL9 0x3C
#define TFTLCD_GAMMA_CTRL10 0x3D
#define TFTLCD_HOR_START_AD 0x50
#define TFTLCD_HOR_END_AD 0x51
#define TFTLCD_VER_START_AD 0x52
#define TFTLCD_VER_END_AD 0x53
#define TFTLCD_GATE_SCAN_CTRL1 0x60
#define TFTLCD_GATE_SCAN_CTRL2 0x61
#define TFTLCD_GATE_SCAN_CTRL3 0x6A
#define TFTLCD_PART_IMG1_DISP_POS 0x80
#define TFTLCD_PART_IMG1_START_AD 0x81
#define TFTLCD_PART_IMG1_END_AD 0x82
#define TFTLCD_PART_IMG2_DISP_POS 0x83
#define TFTLCD_PART_IMG2_START_AD 0x84
#define TFTLCD_PART_IMG2_END_AD 0x85
#define TFTLCD_PANEL_IF_CTRL1 0x90
#define TFTLCD_PANEL_IF_CTRL2 0x92
#define TFTLCD_PANEL_IF_CTRL3 0x93
#define TFTLCD_PANEL_IF_CTRL4 0x95
#define TFTLCD_PANEL_IF_CTRL5 0x97
#define TFTLCD_PANEL_IF_CTRL6 0x98
#define TFTLCD_DELAYCMD 0xFF

#define DATAPORT PORTD
#define DATAPIN  PIND
#define DATADDR  DDRD

#define COMDDR DDRB
#define COMPORT PORTB
#define COMMASK 0b11111;

#define RT 0b10000
#define CS 0b01000
#define CD 0b00100
#define WR 0b00010
#define RD 0b00001

#define n_WR 0b11111101

#define CD_RD_WR 0b0111
#define CD_CS 0b01100
#define RD_WR 0b00011
#define WR_CS 0b01010

#define write_16(data) \
  write_8(data>>8);\
  write_8(data);

#define write_8(data) \
	DATAPORT = (data);\
\
	COMPORT &= n_WR;\
	COMPORT |= WR;


namespace tft {
  void drawPixel(uint16_t x, uint16_t y, uint16_t color);
  void fillScreen(uint16_t color);

  void initDisplay(void);
  void goTo(int x, int y);
  void reset(void);

  void writeData(uint16_t d);
  void writeCommand(uint16_t c);
  void writeRegister(uint16_t addr, uint16_t data);
}

#endif

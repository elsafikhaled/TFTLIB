#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define TFTLCD_START_OSC			0x00
#define TFTLCD_DRIV_OUT_CTRL		0x01
#define TFTLCD_DRIV_WAV_CTRL		0x02
#define TFTLCD_ENTRY_MOD			0x03
#define TFTLCD_RESIZE_CTRL			0x04
#define TFTLCD_DISP_CTRL1			0x07
#define TFTLCD_DISP_CTRL2			0x08
#define TFTLCD_DISP_CTRL3			0x09
#define TFTLCD_DISP_CTRL4			0x0A
#define TFTLCD_RGB_DISP_IF_CTRL1	0x0C
#define TFTLCD_FRM_MARKER_POS		0x0D
#define TFTLCD_RGB_DISP_IF_CTRL2	0x0F
#define TFTLCD_POW_CTRL1			0x10
#define TFTLCD_POW_CTRL2			0x11
#define TFTLCD_POW_CTRL3			0x12
#define TFTLCD_POW_CTRL4			0x13
#define TFTLCD_GRAM_HOR_AD			0x20
#define TFTLCD_GRAM_VER_AD			0x21
#define TFTLCD_RW_GRAM				0x22
#define TFTLCD_POW_CTRL7			0x29
#define TFTLCD_FRM_RATE_COL_CTRL	0x2B
#define TFTLCD_GAMMA_CTRL1			0x30
#define TFTLCD_GAMMA_CTRL2			0x31
#define TFTLCD_GAMMA_CTRL3			0x32
#define TFTLCD_GAMMA_CTRL4			0x35 
#define TFTLCD_GAMMA_CTRL5			0x36
#define TFTLCD_GAMMA_CTRL6			0x37
#define TFTLCD_GAMMA_CTRL7			0x38
#define TFTLCD_GAMMA_CTRL8			0x39
#define TFTLCD_GAMMA_CTRL9			0x3C
#define TFTLCD_GAMMA_CTRL10			0x3D
#define TFTLCD_HOR_START_AD			0x50
#define TFTLCD_HOR_END_AD			0x51
#define TFTLCD_VER_START_AD			0x52
#define TFTLCD_VER_END_AD			0x53
#define TFTLCD_GATE_SCAN_CTRL1		0x60
#define TFTLCD_GATE_SCAN_CTRL2		0x61
#define TFTLCD_GATE_SCAN_CTRL3		0x6A
#define TFTLCD_PART_IMG1_DISP_POS	0x80
#define TFTLCD_PART_IMG1_START_AD	0x81
#define TFTLCD_PART_IMG1_END_AD		0x82
#define TFTLCD_PART_IMG2_DISP_POS	0x83
#define TFTLCD_PART_IMG2_START_AD	0x84
#define TFTLCD_PART_IMG2_END_AD		0x85
#define TFTLCD_PANEL_IF_CTRL1		0x90
#define TFTLCD_PANEL_IF_CTRL2		0x92
#define TFTLCD_PANEL_IF_CTRL3		0x93
#define TFTLCD_PANEL_IF_CTRL4		0x95
#define TFTLCD_PANEL_IF_CTRL5		0x97
#define TFTLCD_PANEL_IF_CTRL6		0x98
#define TFTLCD_DELAYCMD             0xFF

#define DATAPORT PORTD
#define DATAPIN  PIND
#define DATADDR  DDRD

#define COMDDR DDRC
#define COMPORT PORTC
#define COMMASK 0b11111;

#define RT 0b10000
#define CS 0b01000
#define CD 0b00100
#define WR 0b00010
#define RD 0b00001

#define CD_RD_WR 0b0111
#define CD_CS 0b01100
#define RD_WR 0b011
#define WR_CS 0b01010

void drawPixel(uint16_t x, uint16_t y, uint16_t color);
void fillScreen(uint16_t color);

void initDisplay(void);
void goTo(int x, int y);
void reset(void);

void writeData(uint16_t d);
void writeCommand(uint16_t c);
void writeRegister(uint16_t addr, uint16_t data);
void writeData_unsafe(uint16_t d);

void goTo(int x, int y)
{
	writeRegister(0x0020, x);
	writeRegister(0x0021, y);
	writeCommand(0x0022);
}

void fillScreen(uint16_t color)
{
	goTo(0, 0);
	uint32_t i;

	i = 76800;
	COMPORT &= ~CS;
	COMPORT |= CD_RD_WR;

	while (i--)
		writeData_unsafe(color);

	COMPORT |= CS;
}

void drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	writeRegister(TFTLCD_GRAM_HOR_AD, x);
    writeRegister(TFTLCD_GRAM_VER_AD, y);
    writeCommand(TFTLCD_RW_GRAM);
    writeData(color);
}

uint16_t _regValues[] PROGMEM = {
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

void initDisplay(void)
{

	DATADDR = 0xFF;
	COMDDR |= COMMASK
	uint16_t a, d;

	reset();

	for (uint8_t i = 0; i < sizeof(_regValues) / 4; i++)
	{
		a = pgm_read_word(_regValues + i*2);
		d = pgm_read_word(_regValues + i*2 + 1);

		if (a == 0xFF)
			_delay_ms(d);

		else
			writeRegister(a, d);
  }
}

void reset(void)
{
  COMPORT &= ~RT;
  _delay_ms(2);
  COMPORT |= RT;

  writeData(0);
  writeData(0);
  writeData(0);
  writeData(0);
}

void writeData(uint16_t data)
{
	COMPORT &= ~CS;
	COMPORT |= CD_RD_WR;

	DATAPORT = (data>>8);
  
	COMPORT &= ~WR;
	COMPORT |= WR;

	DATAPORT = data;

	COMPORT &= ~WR;
	COMPORT |= WR_CS;
}

inline void writeData_unsafe(uint16_t data)
{
	DATAPORT = (data >> 8);

	COMPORT &= ~WR;
	COMPORT |= WR;

	DATAPORT = (data);

	COMPORT &= ~WR;
	COMPORT |= WR;
}

void writeCommand(uint16_t cmd)
{
	COMPORT &= ~CD_CS;
	COMPORT |= RD_WR;

	DATAPORT = (cmd>>8);

	COMPORT &= ~WR;
	COMPORT |= WR;

	DATAPORT = (cmd);

	COMPORT &= ~WR;
	COMPORT |= WR_CS;
}

void writeRegister(uint16_t addr, uint16_t data)
{
	writeCommand(addr);
	writeData(data);
}


#define F_CPU 16000000UL
#include <avr/io.h>
#include "TFTLIB.h"

int main()
{
  tft::initDisplay();
  tft::fillScreen(0x33a0);
	return 0;
}

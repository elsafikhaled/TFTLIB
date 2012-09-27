#include <avr/io.h>
#include "myTFTLCD.h"

int main()
{
	initDisplay();
	fillScreen(0x0050);
	return 0;
}

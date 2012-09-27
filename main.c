#include <avr/io.h>
#include "TFTLIB.h"

int main()
{
	initDisplay();
	fillScreen(0x0050);
	return 0;
}

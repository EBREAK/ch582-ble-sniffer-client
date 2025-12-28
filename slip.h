#pragma once

#include <stdint.h>

enum {
	SLIP_END = 0xC0,
	SLIP_ESC = 0xDB,
	SLIP_ESC_END = 0xDC,
	SLIP_ESC_ESC = 0xDE,
};

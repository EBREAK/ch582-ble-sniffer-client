#pragma once

#include <stdint.h>

extern uint8_t crc8_ccitt_lut[256];

static inline uint8_t crc8_ccitt_byte(uint8_t crc, uint8_t c)
{
	return crc8_ccitt_lut[(crc ^ c)];
}

static inline uint8_t crc8_ccitt_buf(uint8_t crc, uint8_t *buf,
				     unsigned int len)
{
	while (len > 0) {
		crc = crc8_ccitt_byte(crc, *buf);
		buf += 1;
		len -= 1;
	}
	return crc;
}

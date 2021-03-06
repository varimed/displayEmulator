#include <stdio.h>
#include "libbmp.h"
#include <stdint.h>
//#include <inttypes.h.>

#include <string.h>
#include "dispbuf.h"
#include "font_mono5x8.h"
#include "font_comicsans10pt.h"
#include "font_consolas12pt.h"
#include "font_sansserif11pt.h"

TDisplayBuffer display_buffer;

uint8_t st7565_buffer[1024] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,
	0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,
	0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,
	0x01,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x0F,
	0x01,0x80,0x00,0x01,0x1F,0xFF,0xFF,0x1F,0x00,0x00,0xE0,0xFE,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0x7F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x0F,0x7F,0xFE,0xF0,
	0x80,0x00,0x80,0xF0,0xFE,0x7F,0x1F,0x01,0xE0,0xFC,0xFF,0x9F,0x83,0x83,0x9F,0xFF,
	0xFC,0xE0,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x61,0x61,0xE1,0xF3,0xFF,0xBF,0x1E,0x00,
	0x00,0xFF,0xFF,0xFF,0x00,0x00,0xFE,0xFF,0xFF,0x07,0x3F,0xFE,0xF8,0x80,0x80,0xF8,
	0xFE,0x3F,0x07,0xFF,0xFF,0xFE,0x00,0x00,0xFF,0xFF,0xFF,0x71,0x71,0x71,0x71,0x71,
	0x71,0x00,0x00,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01,0x03,0x07,0xFF,0xFF,0xFE,0xF8,
	0x00,0x00,0x00,0x00,0x38,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,
	0xFC,0xFF,0xFE,0xC0,0x00,0x03,0x01,0x00,0xC0,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0x7F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x0F,
	0x1F,0x1E,0x1F,0x0F,0x03,0x00,0x00,0x1E,0x1F,0x0F,0x03,0x01,0x01,0x01,0x01,0x03,
	0x0F,0x1F,0x1E,0x00,0x00,0x1F,0x1F,0x1F,0x00,0x00,0x00,0x03,0x1F,0x1F,0x1F,0x00,
	0x00,0x1F,0x1F,0x1F,0x00,0x00,0x1F,0x1F,0x07,0x00,0x00,0x00,0x07,0x1F,0x1F,0x07,
	0x00,0x00,0x00,0x03,0x1F,0x1F,0x00,0x00,0x1F,0x1F,0x1F,0x1C,0x1C,0x1C,0x1C,0x1C,
	0x1C,0x00,0x00,0x1F,0x1F,0x1F,0x18,0x18,0x18,0x1C,0x1C,0x0E,0x0F,0x07,0x03,0x00,
	0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
	0x07,0x07,0x07,0x07,0x00,0x00,0x00,0x00,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
	0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};
/*uint8_t st7565_buffer[1024] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
			0xc0, 0xc0, 0xe0, 0x70, 0x70, 0x38, 0x38, 0xfc, 0xfe, 0x1e, 0x06, 0x00, 0x00, 0x80, 0xc0, 0xc0,
			0xe0, 0x60, 0x60, 0xa0, 0x30, 0xf0, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x26, 0x6e,
			0x6e, 0x7e, 0xfe, 0xfe, 0xdc, 0xdc, 0x1c, 0x1c, 0x1c, 0x98, 0x98, 0x98, 0x30, 0x30, 0x60, 0xc0,
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x1e, 0x1b, 0x31,
			0x7c, 0xfe, 0xf2, 0xe0, 0x80, 0x80, 0x03, 0x0f, 0x0f, 0x0c, 0x1c, 0x1e, 0x9f, 0xff, 0xed, 0xf8,
			0x58, 0x0c, 0x0c, 0x18, 0x18, 0x19, 0x0e, 0x0e, 0x1c, 0x1c, 0x0c, 0x06, 0x0e, 0x1c, 0x6c, 0xf6,
			0xf3, 0xfe, 0xde, 0x0f, 0x07, 0x03, 0x40, 0x60, 0x30, 0x98, 0xcc, 0xe3, 0xf1, 0xf8, 0xbc, 0x1f,
			0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0xff, 0xff, 0xfc, 0xc4, 0x00, 0x00, 0x00, 0xfc,
			0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x07, 0x0f, 0x7e, 0xfe, 0xe7, 0xc3, 0xc3, 0xc1, 0xf0, 0xe0,
			0x60, 0x60, 0xe0, 0x00, 0x00, 0x80, 0xc0, 0xc0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe4,
			0xe0, 0xc1, 0xc1, 0x83, 0x83, 0x06, 0x04, 0x46, 0xe7, 0x67, 0x03, 0x03, 0x3f, 0xff, 0xff, 0x03,
			0x06, 0x04, 0x0c, 0x18, 0x18, 0x30, 0x60, 0x6c, 0xfe, 0x7f, 0x1f, 0x1f, 0x4e, 0x80, 0xc0, 0xff,
			0x7f, 0xf0, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xef, 0xc7, 0xff, 0xff, 0xe3, 0xf3, 0x3b,
			0x1f, 0x07, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x05, 0x0c, 0x1e, 0x3f, 0x3f, 0x67, 0x67,
			0x67, 0x7f, 0xff, 0xc0, 0xc1, 0xc3, 0xff, 0xfe, 0xf9, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x7f, 0x03,
			0x02, 0x06, 0x04, 0x0c, 0x18, 0x18, 0x30, 0xf8, 0x10, 0x00, 0x80, 0xc0, 0x60, 0x30, 0x3c, 0x1c,
			0x0f, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f,
			0x3e, 0x78, 0xe0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x08, 0x1c, 0x0f, 0x03, 0x01, 0x00, 0x00, 0x10, 0x18, 0x0b, 0x07, 0x07, 0x1f, 0xf8,
			0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x01, 0x03, 0x0f, 0x1e, 0x3c, 0x70, 0xf0, 0xd0, 0xc0, 0xc0, 0x20, 0x80, 0x10, 0x0c,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20,
			0x9f, 0xff, 0xfe, 0xf8, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x07, 0x04, 0x0c,
			0x1c, 0x18, 0x38, 0x30, 0x70, 0xe0, 0xe0, 0xc0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0x03,
			0x01, 0x04, 0x03, 0x03, 0x0f, 0x3f, 0xf8, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x07, 0x07, 0x0e, 0x0e, 0x0e, 0x1c,
			0x1e, 0x1e, 0x1e, 0x3c, 0x3c, 0x3c, 0x38, 0x7f, 0x7f, 0x7c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};*/

void bmp_put_mono_pixel(bmp_img *img, int x, int y, uint8_t color);
char str[1000];

int
main (int argc, char *argv[])
{
    bmp_img img;
	

    display_buffer.width = 128;
    display_buffer.height = 64;
    display_buffer.buffer = st7565_buffer;
    dbufCalcBufSize(&display_buffer);
    dbufInvert(&display_buffer);
    //dbufFill(&display_buffer, 0);
  
    /*
    sprintf(str, ":D \210\216\214\212 11pt");
  dbufPutString(&display_buffer, 2, 2, str, &sansserif_11pt, 1);
   
    
    dbufDrawLine(&display_buffer, 0, 0, 0, 63, 1);
  dbufDrawLine(&display_buffer, 0, 0, 127, 0, 1);
  dbufDrawLine(&display_buffer, 0, 63, 127, 63, 1);
  dbufDrawLine(&display_buffer, 127, 0, 127, 63, 1);
  dbufDrawCirc(&display_buffer, 109, 19, 15, 1);
  dbufDrawRect(&display_buffer, 93, 3, 32, 32, 1);
  dbufDrawLine(&display_buffer, 93, 3, 93+32, 3+32, 1);
  dbufDrawLine(&display_buffer, 93, 3+32, 93+32, 3, 1);
  
  //dbufPutString(&display_buffer, 2, 2, str, &sansserif_11pt, 1);
  //sprintf(str, "Monospace 5x8");
  //dbufPutString(&display_buffer, 2, 19, str, &mono5x8, 1);
  sprintf(str, "Comic 10pt");
  dbufPutString(&display_buffer, 2, 28, str, &comicsans_10pt, 1);
  sprintf(str, "Consolas 12pt");
  dbufPutString(&display_buffer, 2, 45, str, &consolas_12pt, 1);
    */
    
	bmp_img_init_df (&img, display_buffer.width * 4, display_buffer.height*4);
    
    int _x,_y;
	for (size_t y = 0; y < display_buffer.height*4; y++)
	{
		for (size_t x = 0; x < display_buffer.width*4; x++)
		{
            _x = x /4;
            _y = y /4;
            if(y%4==3 || x%4==3){
                bmp_put_mono_pixel(&img, x, y, 0);
                continue;
            }
            if((display_buffer.buffer[_x + (_y / 8) * display_buffer.width] & (1 << (_y % 8))) == 0){
                bmp_put_mono_pixel(&img, x, y, 0);
            }else{
                bmp_put_mono_pixel(&img, x, y, 1);
            }
		}
	}
	
	bmp_img_write (&img, "test.bmp");
	bmp_img_free (&img);
	return 0;
}
 
void bmp_put_mono_pixel(bmp_img *img, int x, int y, uint8_t color){
    if(color == 1){
        bmp_pixel_init (&(img->img_pixels[y][x]), 250, 250, 250);
    }else{
        bmp_pixel_init (&(img->img_pixels[y][x]), 0, 0, 0);
    }
}

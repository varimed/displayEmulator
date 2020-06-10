/*
 * dispbuf.h
 *
 *  Created on: Sep 25, 2019
 *      Author: grzegorz
 */
#include <stdint.h>
#include "fonttypes.h"

#ifndef DISPBUF_H_
#define DISPBUF_H_

typedef struct TDisplayBuffer_s{
	uint16_t width;
	uint16_t height;
	uint16_t size;
	uint8_t *buffer;
} TDisplayBuffer;


void dbufCalcBufSize(TDisplayBuffer *dbuf);
void dbufFill(TDisplayBuffer *dbuf, uint8_t mono_color);
void dbufInvert(TDisplayBuffer *dbuf);
void dbufDrawPixel(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t mono_color);
void dbufDrawLine(TDisplayBuffer *dbuf, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t mono_color);
void dbufDrawRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color);
void dbufDrawFillRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color);
void dbufDrawCirc(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t r, uint8_t mono_color);
uint8_t dbufPutChar(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t c, const TFont *font, uint8_t mono_color);
uint16_t dbufPutString(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t *str, const TFont *font, uint8_t mono_color);
uint16_t dbufGetStringWidth(TDisplayBuffer *dbuf, char *str, const TFont *font);
uint8_t dbufGetCharWidth(TDisplayBuffer *dbuf, uint8_t c, const TFont *font);

#endif /* DISPBUF_H_ */

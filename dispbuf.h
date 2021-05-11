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


void dbuf_Init(TDisplayBuffer *dbuf);
void dbuf_Fill(TDisplayBuffer *dbuf, uint8_t mono_color);
void dbuf_Invert(TDisplayBuffer *dbuf);
void dbuf_DrawPixel(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t mono_color);
void dbuf_DrawLine(TDisplayBuffer *dbuf, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t mono_color);
void dbuf_DrawRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color);
void dbuf_DrawFillRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color);
void dbuf_DrawCirc(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t r, uint8_t mono_color);
uint8_t dbuf_PutChar(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t c, const TFont *font, uint8_t mono_color);
uint16_t dbuf_PutString(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t *str, const TFont *font, uint8_t mono_color);
uint16_t dbuf_GetStringWidth(TDisplayBuffer *dbuf, char *str, const TFont *font);
uint8_t dbuf_GetCharWidth(TDisplayBuffer *dbuf, uint8_t c, const TFont *font);

#endif /* DISPBUF_H_ */

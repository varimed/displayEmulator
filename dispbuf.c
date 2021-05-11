/*
 * dispbuf.c
 *
 *  Created on: Sep 25, 2019
 *      Author: grzegorz
 */

#include <string.h>
#include "dispbuf.h"
#include "fonttypes.h"
#include <stdlib.h>

void dbuf_Init(TDisplayBuffer *dbuf){
	if(dbuf->width == 0 || dbuf->height == 0){
		dbuf->size = 0;
	}else{
		dbuf->size = dbuf->width * ((dbuf->height-1)/8+1);
	}
}

void dbuf_Fill(TDisplayBuffer *dbuf, uint8_t mono_color){
	memset(dbuf->buffer, (mono_color == 0) ? 0x00 : 0xFF, dbuf->size);
}

void dbuf_Invert(TDisplayBuffer *dbuf){
	uint16_t i;

	for (i = 0; i < dbuf->size; i++) {
		dbuf->buffer[i] = ~dbuf->buffer[i];
	}
}

void dbuf_DrawPixel(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t mono_color){
	if(x < dbuf->width && y < dbuf->height){
		/* Set color */
		if (mono_color == 1) {
			dbuf->buffer[x + (y / 8) * dbuf->width] |= 1 << (y % 8);
		} else {
			dbuf->buffer[x + (y / 8) * dbuf->width] &= ~(1 << (y % 8));
		}
	}
}

void dbuf_DrawLine(TDisplayBuffer *dbuf, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t mono_color){
	  int16_t dx = abs(x2-x1), sx = x1<x2 ? 1 : -1;
	  int16_t dy = abs(y2-y1), sy = y1<y2 ? 1 : -1;
	  int16_t err = (dx>dy ? dx : -dy)/2, e2;

	  for(;;){
	    dbuf_DrawPixel(dbuf, x1, y1, mono_color);
	    if (x1==x2 && y1==y2) break;
	    e2 = err;
	    if (e2 >-dx) { err -= dy; x1 += sx; }
	    if (e2 < dy) { err += dx; y1 += sy; }
	  }
}

void dbuf_DrawRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color){
	dbuf_DrawLine(dbuf, x, y, x+w-1, y, mono_color);
	dbuf_DrawLine(dbuf, x+w, y, x+w, y+w-1, mono_color);
	dbuf_DrawLine(dbuf, x+w, y+w, x+1, y+w, mono_color);
	dbuf_DrawLine(dbuf, x, y+w, x, y+1, mono_color);
}

void dbuf_DrawFillRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color){
	for (uint16_t i = 0; i<h; i++){
		dbuf_DrawLine(dbuf, x, y+i, x+w-1, y+i, mono_color);
	}
}

void dbuf_DrawCirc(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t r, uint8_t mono_color){
    int16_t X = r;
    int16_t Y = 0;
    int16_t err = 0;

    while (X >= Y)
    {
    	dbuf_DrawPixel(dbuf, x + X, y + Y, mono_color);
    	dbuf_DrawPixel(dbuf, x + Y, y + X, mono_color);
    	dbuf_DrawPixel(dbuf, x - Y, y + X, mono_color);
    	dbuf_DrawPixel(dbuf, x - X, y + Y, mono_color);
    	dbuf_DrawPixel(dbuf, x - X, y - Y, mono_color);
    	dbuf_DrawPixel(dbuf, x - Y, y - X, mono_color);
    	dbuf_DrawPixel(dbuf, x + Y, y - X, mono_color);
    	dbuf_DrawPixel(dbuf, x + X, y - Y, mono_color);

	if (err <= 0)
	{
	    Y += 1;
	    err += 2*Y + 1;
	}

	if (err > 0)
	{
	    X -= 1;
	    err -= 2*X + 1;
	}
    }
}

uint8_t dbuf_PutChar(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t c, const TFont *font, uint8_t mono_color){
	uint8_t row, page, col, mask;

	uint8_t width;
	uint16_t offset;
	const TFontCharInfo *descriptor;
	const uint8_t *bitmap;

	if(c < font->start_char) return 0;
	if(c > font->end_char) return 0;

	c -= font->start_char;
	descriptor = font->descriptor;
	bitmap = font->data;
	width = descriptor[c].width;
	offset = descriptor[c].offset;

	for(row = 0; row < font->height; row++){
		page = row/8;
		mask = 1 << (row%8);
		for(col = 0; col < width; col++){
			dbuf_DrawPixel(dbuf, x+col, y+row, (bitmap[offset+page*width+col] & mask) ? mono_color : !mono_color);
		}
	}

	return width;
}

uint16_t dbuf_PutString(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t *str, const TFont *font, uint8_t mono_color){
    uint16_t width;

    width = 0;
    while(*str){
        if(width != 0) width += font->spacing;
        width += dbuf_PutChar(dbuf, x+width, y, *str++, font, mono_color);
    }
    return width;
}

uint16_t dbuf_GetStringWidth(TDisplayBuffer *dbuf, char *str, const TFont *font){
    uint16_t width;

    width = 0;
    while(*str){
        width += dbuf_GetCharWidth(dbuf, *str++, font) + font->spacing;
    }
    width -= font->spacing;
    return width;
}

uint8_t dbuf_GetCharWidth(TDisplayBuffer *dbuf, uint8_t c, const TFont *font){
    uint8_t width;
    const TFontCharInfo *descriptor;

    if(c < font->start_char) return 0;
    if(c > font->end_char) return 0;

    c -= font->start_char;
    descriptor = font->descriptor;
    width = descriptor[c].width;
    return width;
}

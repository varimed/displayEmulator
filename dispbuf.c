/*
 * dispbuf.c
 *
 *  Created on: Sep 25, 2019
 *      Author: grzegorz
 */

#include <string.h>
#include "dispbuf.h"
#include "fonttypes.h"

void dbufCalcBufSize(TDisplayBuffer *dbuf){
	if(dbuf->width == 0 || dbuf->height == 0){
		dbuf->size = 0;
	}else{
		dbuf->size = dbuf->width * ((dbuf->height-1)/8+1);
	}
}

void dbufFill(TDisplayBuffer *dbuf, uint8_t mono_color){
	memset(dbuf->buffer, (mono_color == 0) ? 0x00 : 0xFF, dbuf->size);
}

void dbufInvert(TDisplayBuffer *dbuf){
	uint16_t i;

	for (i = 0; i < dbuf->size; i++) {
		dbuf->buffer[i] = ~dbuf->buffer[i];
	}
}

void dbufDrawPixel(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t mono_color){
	if(x < dbuf->width && y < dbuf->height){
		/* Set color */
		if (mono_color == 1) {
			dbuf->buffer[x + (y / 8) * dbuf->width] |= 1 << (y % 8);
		} else {
			dbuf->buffer[x + (y / 8) * dbuf->width] &= ~(1 << (y % 8));
		}
	}
}

void dbufDrawLine(TDisplayBuffer *dbuf, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t mono_color){
	  int16_t dx = abs(x2-x1), sx = x1<x2 ? 1 : -1;
	  int16_t dy = abs(y2-y1), sy = y1<y2 ? 1 : -1;
	  int16_t err = (dx>dy ? dx : -dy)/2, e2;

	  for(;;){
	    dbufDrawPixel(dbuf, x1, y1, mono_color);
	    if (x1==x2 && y1==y2) break;
	    e2 = err;
	    if (e2 >-dx) { err -= dy; x1 += sx; }
	    if (e2 < dy) { err += dx; y1 += sy; }
	  }
}

void dbufDrawRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color){
	dbufDrawLine(dbuf, x, y, x+w-1, y, mono_color);
	dbufDrawLine(dbuf, x+w, y, x+w, y+w-1, mono_color);
	dbufDrawLine(dbuf, x+w, y+w, x+1, y+w, mono_color);
	dbufDrawLine(dbuf, x, y+w, x, y+1, mono_color);
}

void dbufDrawFillRect(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mono_color){

}

void dbufDrawCirc(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint16_t r, uint8_t mono_color){
    int16_t X = r;
    int16_t Y = 0;
    int16_t err = 0;

    while (X >= Y)
    {
    	dbufDrawPixel(dbuf, x + X, y + Y, mono_color);
    	dbufDrawPixel(dbuf, x + Y, y + X, mono_color);
    	dbufDrawPixel(dbuf, x - Y, y + X, mono_color);
    	dbufDrawPixel(dbuf, x - X, y + Y, mono_color);
    	dbufDrawPixel(dbuf, x - X, y - Y, mono_color);
    	dbufDrawPixel(dbuf, x - Y, y - X, mono_color);
    	dbufDrawPixel(dbuf, x + Y, y - X, mono_color);
    	dbufDrawPixel(dbuf, x + X, y - Y, mono_color);

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

uint8_t dbufPutChar(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t c, const TFont *font, uint8_t mono_color){
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
			dbufDrawPixel(dbuf, x+col, y+row, (bitmap[offset+page*width+col] & mask) ? 1 : 0);
		}
	}

	return width;
}

uint16_t dbufPutString(TDisplayBuffer *dbuf, uint16_t x, uint16_t y, uint8_t *str, const TFont *font, uint8_t mono_color){
    uint16_t width;

    width = 0;
    while(*str){
        if(width != 0) width += font->spacing;
        width += dbufPutChar(dbuf, x+width, y, *str++, font, mono_color);
    }
    return width;
}

uint16_t dbufGetStringWidth(TDisplayBuffer *dbuf, char *str, const TFont *font){
    uint16_t width;

    width = 0;
    while(*str){
        width += dbufGetCharWidth(dbuf, *str++, font) + font->spacing;
    }
    width -= font->spacing;
    return width;
}

uint8_t dbufGetCharWidth(TDisplayBuffer *dbuf, uint8_t c, const TFont *font){
    uint8_t width;
    const TFontCharInfo *descriptor;

    if(c < font->start_char) return 0;
    if(c > font->end_char) return 0;

    c -= font->start_char;
    descriptor = font->descriptor;
    width = descriptor[c].width;
    return width;
}

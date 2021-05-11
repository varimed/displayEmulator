/*
 * fonttype.h
 *
 *  Created on: Sep 29, 2019
 *      Author: grzegorz
 */

#include "inttypes.h"

#ifndef FONTTYPE_H_
#define FONTTYPE_H_

typedef struct TFontCharInfo_s{
    uint8_t width;
    uint16_t offset;
} TFontCharInfo;

typedef struct TFont_s{
    uint8_t height;
    uint8_t spacing;
    uint8_t start_char;
    uint8_t end_char;
    const TFontCharInfo *descriptor;
    const uint8_t *data;
} TFont;

#endif /* FONTTYPE_H_ */

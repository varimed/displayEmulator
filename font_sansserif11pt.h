#ifndef FONT_SANSSERIF11PT_H_INCLUDED
#define FONT_SANSSERIF11PT_H_INCLUDED

#include "fonttypes.h"
#include "inttypes.h"

/*
**  Font data for Sans Serif 11pt
*/

/* Character bitmaps for Sans Serif 11pt */
const uint8_t sansserif_11ptBitmaps[] =
{
    // @0 ' ' (6 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @12 '!' (2 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	// ##
	//
	//
	//
	0xFE, 0xFE,
	0x0B, 0x0B,

	// @16 '"' (5 pixels wide)
	//
	// ## ##
	// ## ##
	// ## ##
	// ## ##
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x1E, 0x1E, 0x00, 0x1E, 0x1E,
	0x00, 0x00, 0x00, 0x00, 0x00,

	// @26 '#' (9 pixels wide)
	//
	//    ## ##
	//    ## ##
	//    ## ##
	//  ########
	//   ## ##
	//   ## ##
	//   ## ##
	// ########
	//  ## ##
	//  ## ##
	//  ## ##
	//
	//
	//
	0x00, 0x10, 0xF0, 0xFE, 0x1E, 0xF0, 0xFE, 0x1E, 0x10,
	0x01, 0x0F, 0x0F, 0x01, 0x0F, 0x0F, 0x01, 0x01, 0x00,

	// @44 '$' (7 pixels wide)
	//    ##
	//  #####
	// ## ####
	// ## ####
	// ## ##
	// ## ##
	//  #####
	//   ## ##
	//   ## ##
	// #### ##
	// #### ##
	//  #####
	//   ##
	//
	//
	0x3C, 0x7E, 0xC2, 0xFF, 0x7F, 0xCE, 0x8C,
	0x06, 0x0E, 0x1F, 0x1F, 0x08, 0x0F, 0x07,

	// @58 '%' (12 pixels wide)
	//
	//  ####   ##
	// ##  ## ##
	// ##  ## ##
	// ##  ####
	// ##  ####
	//  ##########
	//     ####  ##
	//     ####  ##
	//    ## ##  ##
	//    ## ##  ##
	//   ##   ####
	//
	//
	//
	0x3C, 0x7E, 0x42, 0x42, 0xFE, 0xFC, 0xF0, 0xFC, 0x4E, 0x42, 0xC0, 0x80,
	0x00, 0x00, 0x08, 0x0E, 0x07, 0x01, 0x07, 0x0F, 0x08, 0x08, 0x0F, 0x07,

	// @82 '&' (9 pixels wide)
	//
	//   ####
	//  ##  ##
	//  ##  ##
	//  ##  ##
	//   ####
	//  ####
	// ##  ## ##
	// ##   ####
	// ##    ##
	// ##   ###
	//  ##### ##
	//
	//
	//
	0x80, 0xDC, 0x7E, 0x62, 0xE2, 0xBE, 0x1C, 0x80, 0x80,
	0x07, 0x0F, 0x08, 0x08, 0x08, 0x0D, 0x07, 0x0F, 0x09,

	// @100 ''' (2 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x1E, 0x1E,
	0x00, 0x00,

	// @104 '(' (5 pixels wide)
	//
	//    ##
	//   ##
	//  ##
	//  ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//  ##
	//  ##
	//   ##
	//    ##
	0xE0, 0xF8, 0x1C, 0x06, 0x02,
	0x07, 0x1F, 0x38, 0x60, 0x40,

	// @114 ')' (5 pixels wide)
	//
	// ##
	//  ##
	//   ##
	//   ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//   ##
	//   ##
	//  ##
	// ##
	0x02, 0x06, 0x1C, 0xF8, 0xE0,
	0x40, 0x60, 0x38, 0x1F, 0x07,

	// @124 '*' (6 pixels wide)
	//
	//   ##
	// ######
	//  ####
	//  ####
	//  ####
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x04, 0x3C, 0x3E, 0x3E, 0x3C, 0x04,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @136 '+' (8 pixels wide)
	//
	//
	//
	//    ##
	//    ##
	//    ##
	// ########
	//    ##
	//    ##
	//    ##
	//
	//
	//
	//
	//
	0x40, 0x40, 0x40, 0xF8, 0xF8, 0x40, 0x40, 0x40,
	0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00,

	// @152 ',' (3 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//  ##
	//  ##
	//  ##
	// ##
	0x00, 0x00, 0x00,
	0x40, 0x78, 0x38,

	// @158 '-' (5 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	// #####
	//
	//
	//
	//
	//
	//
	//
	0x80, 0x80, 0x80, 0x80, 0x80,
	0x00, 0x00, 0x00, 0x00, 0x00,

	// @168 '.' (2 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	// ##
	//
	//
	//
	0x00, 0x00,
	0x08, 0x08,

	// @172 '/' (5 pixels wide)
	//
	//    ##
	//    ##
	//   ##
	//   ##
	//   ##
	//  ##
	//  ##
	//  ##
	// ##
	// ##
	// ##
	//
	//
	//
	0x00, 0xC0, 0xF8, 0x3E, 0x06,
	0x0E, 0x0F, 0x01, 0x00, 0x00,

	// @182 '0' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0xFC, 0xFE, 0x02, 0x02, 0x02, 0xFE, 0xFC,
	0x07, 0x0F, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @196 '1' (4 pixels wide)
	//
	//   ##
	// ####
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//
	//
	//
	0x04, 0x04, 0xFE, 0xFE,
	0x00, 0x00, 0x0F, 0x0F,

	// @204 '2' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	//      ##
	//      ##
	//     ##
	//    ##
	//   ##
	//  ##
	// ##
	// #######
	//
	//
	//
	0x0C, 0x0E, 0x02, 0x82, 0xC2, 0x7E, 0x3C,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08,

	// @218 '3' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	//      ##
	//      ##
	//   ####
	//      ##
	//      ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0x0C, 0x0E, 0x42, 0x42, 0x42, 0xFE, 0xBC,
	0x06, 0x0E, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @232 '4' (7 pixels wide)
	//
	//     ##
	//    ###
	//    ###
	//   ####
	//  ## ##
	//  ## ##
	// ##  ##
	// #######
	//     ##
	//     ##
	//     ##
	//
	//
	//
	0x80, 0xE0, 0x70, 0x1C, 0xFE, 0xFE, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x0F, 0x0F, 0x01,

	// @246 '5' (7 pixels wide)
	//
	// #######
	// ##
	// ##
	// ##
	// ######
	// ##   ##
	//      ##
	//      ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0x7E, 0x7E, 0x22, 0x22, 0x22, 0xE2, 0xC2,
	0x06, 0x0E, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @260 '6' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	// ##
	// ######
	// ###  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0xFC, 0xFE, 0x62, 0x22, 0x22, 0xEE, 0xCC,
	0x07, 0x0F, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @274 '7' (7 pixels wide)
	//
	// #######
	//      ##
	//      ##
	//     ##
	//     ##
	//    ##
	//    ##
	//   ##
	//   ##
	//   ##
	//  ##
	//
	//
	//
	0x02, 0x02, 0x02, 0xC2, 0xF2, 0x3E, 0x0E,
	0x00, 0x08, 0x0F, 0x07, 0x00, 0x00, 0x00,

	// @288 '8' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//  #####
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0xBC, 0xFE, 0x42, 0x42, 0x42, 0xFE, 0xBC,
	0x07, 0x0F, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @302 '9' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##  ###
	//  ######
	//      ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0x7C, 0xFE, 0x82, 0x82, 0xC2, 0xFE, 0xFC,
	0x06, 0x0E, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @316 ':' (2 pixels wide)
	//
	//
	//
	//
	// ##
	//
	//
	//
	//
	//
	//
	// ##
	//
	//
	//
	0x10, 0x10,
	0x08, 0x08,

	// @320 ';' (3 pixels wide)
	//
	//
	//
	//
	//  ##
	//
	//
	//
	//
	//
	//
	//  ##
	//  ##
	//  ##
	// ##
	0x00, 0x10, 0x10,
	0x40, 0x78, 0x38,

	// @326 '<' (8 pixels wide)
	//
	//
	//
	//      ###
	//    ###
	//  ###
	// ##
	//  ###
	//    ###
	//      ###
	//
	//
	//
	//
	//
	0x40, 0xE0, 0xA0, 0xB0, 0x10, 0x18, 0x08, 0x08,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x02, 0x02,

	// @342 '=' (8 pixels wide)
	//
	//
	//
	//
	// ########
	//
	//
	// ########
	//
	//
	//
	//
	//
	//
	//
	0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @358 '>' (8 pixels wide)
	//
	//
	//
	// ###
	//   ###
	//     ###
	//       ##
	//     ###
	//   ###
	// ###
	//
	//
	//
	//
	//
	0x08, 0x08, 0x18, 0x10, 0xB0, 0xA0, 0xE0, 0x40,
	0x02, 0x02, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00,

	// @374 '?' (7 pixels wide)
	//
	//  #####
	// ##   ##
	// ##   ##
	//      ##
	//     ##
	//    ##
	//    ##
	//    ##
	//
	//
	//    ##
	//
	//
	//
	0x0C, 0x0E, 0x02, 0xC2, 0xE2, 0x3E, 0x1C,
	0x00, 0x00, 0x00, 0x09, 0x09, 0x00, 0x00,

	// @388 '@' (14 pixels wide)
	//
	//      #####
	//    ###   ###
	//   ##       ##
	//  ##  ########
	//  ## ##   ## ##
	// ##  ##   ## ##
	// ## ##    ## ##
	// ## ##   ##  ##
	// ## ##   ##  ##
	// ## ##  ### ##
	//  ## ########
	//  ##
	//   ###     ##
	//     #######
	0xC0, 0xF0, 0x38, 0x8C, 0xE4, 0x76, 0x12, 0x12, 0x12, 0xF6, 0xF4, 0x1C, 0xF8, 0xE0,
	0x07, 0x1F, 0x38, 0x27, 0x6F, 0x48, 0x48, 0x4C, 0x4F, 0x4F, 0x68, 0x2C, 0x07, 0x03,

	// @416 'A' (10 pixels wide)
	//
	//     ##
	//     ##
	//    ####
	//    ####
	//   ##  ##
	//   ##  ##
	//   ######
	//  ##    ##
	//  ##    ##
	// ##      ##
	// ##      ##
	//
	//
	//
	0x00, 0x00, 0xE0, 0xF8, 0x9E, 0x9E, 0xF8, 0xE0, 0x00, 0x00,
	0x0C, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x0C,

	// @436 'B' (9 pixels wide)
	//
	// #######
	// ##    ##
	// ##     ##
	// ##     ##
	// ##    ##
	// #######
	// ##    ##
	// ##     ##
	// ##     ##
	// ##    ##
	// #######
	//
	//
	//
	0xFE, 0xFE, 0x42, 0x42, 0x42, 0x42, 0xE6, 0xBC, 0x18,
	0x0F, 0x0F, 0x08, 0x08, 0x08, 0x08, 0x0C, 0x07, 0x03,

	// @454 'C' (11 pixels wide)
	//
	//    #####
	//  ###   ###
	//  ##     ##
	// ##       ##
	// ##
	// ##
	// ##
	// ##       ##
	//  ##     ##
	//  ###   ###
	//    #####
	//
	//
	//
	0xF0, 0xFC, 0x0C, 0x06, 0x02, 0x02, 0x02, 0x06, 0x0C, 0x1C, 0x10,
	0x01, 0x07, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x0C, 0x06, 0x07, 0x01,

	// @476 'D' (10 pixels wide)
	//
	// #######
	// ##    ###
	// ##     ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##     ##
	// ##    ###
	// #######
	//
	//
	//
	0xFE, 0xFE, 0x02, 0x02, 0x02, 0x02, 0x06, 0x0C, 0xFC, 0xF0,
	0x0F, 0x0F, 0x08, 0x08, 0x08, 0x08, 0x0C, 0x06, 0x07, 0x01,

	// @496 'E' (9 pixels wide)
	//
	// #########
	// ##
	// ##
	// ##
	// ##
	// ########
	// ##
	// ##
	// ##
	// ##
	// #########
	//
	//
	//
	0xFE, 0xFE, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x02,
	0x0F, 0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

	// @514 'F' (9 pixels wide)
	//
	// #########
	// ##
	// ##
	// ##
	// ##
	// ########
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	0xFE, 0xFE, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x02,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @532 'G' (11 pixels wide)
	//
	//    ######
	//  ###    ##
	//  ##      ##
	// ##
	// ##
	// ##    #####
	// ##       ##
	// ##       ##
	//  ##      ##
	//  ###    ###
	//    ########
	//
	//
	//
	0xF0, 0xFC, 0x0C, 0x06, 0x02, 0x02, 0x42, 0x42, 0x46, 0xCC, 0xC8,
	0x01, 0x07, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x08, 0x0C, 0x0F, 0x0F,

	// @554 'H' (10 pixels wide)
	//
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##########
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	//
	//
	//
	0xFE, 0xFE, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0xFE, 0xFE,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F,

	// @574 'I' (2 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	0xFE, 0xFE,
	0x0F, 0x0F,

	// @578 'J' (7 pixels wide)
	//
	//      ##
	//      ##
	//      ##
	//      ##
	//      ##
	//      ##
	//      ##
	//      ##
	// ##   ##
	// ##   ##
	//  #####
	//
	//
	//
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE,
	0x06, 0x0E, 0x08, 0x08, 0x08, 0x0F, 0x07,

	// @592 'K' (10 pixels wide)
	//
	// ##     ##
	// ##    ##
	// ##   ##
	// ##  ##
	// ## ##
	// #####
	// ### ##
	// ##   ##
	// ##    ##
	// ##     ##
	// ##      ##
	//
	//
	//
	0xFE, 0xFE, 0xC0, 0x60, 0xF0, 0x98, 0x0C, 0x06, 0x02, 0x00,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06, 0x0C, 0x08,

	// @612 'L' (8 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ########
	//
	//
	//
	0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x0F, 0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

	// @628 'M' (12 pixels wide)
	//
	// ##        ##
	// ##        ##
	// ###      ###
	// ####    ####
	// ####    ####
	// ## ##  ## ##
	// ## ##  ## ##
	// ##  ####  ##
	// ##  ####  ##
	// ##   ##   ##
	// ##   ##   ##
	//
	//
	//
	0xFE, 0xFE, 0x38, 0xF0, 0xC0, 0x00, 0x00, 0xC0, 0xF0, 0x38, 0xFE, 0xFE,
	0x0F, 0x0F, 0x00, 0x00, 0x03, 0x0F, 0x0F, 0x03, 0x00, 0x00, 0x0F, 0x0F,

	// @652 'N' (10 pixels wide)
	//
	// ##      ##
	// ###     ##
	// ####    ##
	// ## ##   ##
	// ## ##   ##
	// ##  ##  ##
	// ##   ## ##
	// ##   ## ##
	// ##    ####
	// ##     ###
	// ##      ##
	//
	//
	//
	0xFE, 0xFE, 0x0C, 0x38, 0x70, 0xC0, 0x80, 0x00, 0xFE, 0xFE,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06, 0x0F, 0x0F,

	// @672 'O' (11 pixels wide)
	//
	//    #####
	//  ###   ###
	//  ##     ##
	// ##       ##
	// ##       ##
	// ##       ##
	// ##       ##
	// ##       ##
	//  ##     ##
	//  ###   ###
	//    #####
	//
	//
	//
	0xF0, 0xFC, 0x0C, 0x06, 0x02, 0x02, 0x02, 0x06, 0x0C, 0xFC, 0xF0,
	0x01, 0x07, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x0C, 0x06, 0x07, 0x01,

	// @694 'P' (10 pixels wide)
	//
	// ########
	// ##     ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##     ##
	// ########
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	0xFE, 0xFE, 0x82, 0x82, 0x82, 0x82, 0x82, 0xC6, 0x7C, 0x38,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @714 'Q' (11 pixels wide)
	//
	//    #####
	//  ###   ###
	//  ##     ##
	// ##       ##
	// ##       ##
	// ##       ##
	// ##       ##
	// ##   ##  ##
	//  ##   ####
	//  ###   ###
	//    #######
	//          ##
	//
	//
	0xF0, 0xFC, 0x0C, 0x06, 0x02, 0x02, 0x02, 0x06, 0x0C, 0xFC, 0xF0,
	0x01, 0x07, 0x06, 0x0C, 0x08, 0x09, 0x0B, 0x0E, 0x0E, 0x1F, 0x11,

	// @736 'R' (10 pixels wide)
	//
	// #######
	// ##    ##
	// ##     ##
	// ##     ##
	// ##    ##
	// ########
	// ##    ##
	// ##     ##
	// ##     ##
	// ##     ##
	// ##      ##
	//
	//
	//
	0xFE, 0xFE, 0x42, 0x42, 0x42, 0x42, 0xE6, 0xFC, 0x18, 0x00,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x08,

	// @756 'S' (9 pixels wide)
	//
	//   #####
	//  ##   ##
	// ##     ##
	// ##
	//  ##
	//   #####
	//       ##
	//        ##
	// ##     ##
	//  ##   ##
	//   #####
	//
	//
	//
	0x18, 0x3C, 0x66, 0x42, 0x42, 0x42, 0xC6, 0x8C, 0x08,
	0x02, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x0C, 0x07, 0x03,

	// @774 'T' (10 pixels wide)
	//
	// ##########
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//
	//
	//
	0x02, 0x02, 0x02, 0x02, 0xFE, 0xFE, 0x02, 0x02, 0x02, 0x02,
	0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,

	// @794 'U' (10 pixels wide)
	//
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	// ##      ##
	//  ##    ##
	//   ######
	//
	//
	//
	0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE,
	0x03, 0x07, 0x0C, 0x08, 0x08, 0x08, 0x08, 0x0C, 0x07, 0x03,

	// @814 'V' (10 pixels wide)
	//
	// ##      ##
	// ##      ##
	//  ##    ##
	//  ##    ##
	//   ##  ##
	//   ##  ##
	//   ##  ##
	//    ####
	//    ####
	//     ##
	//     ##
	//
	//
	//
	0x06, 0x1E, 0xF8, 0xE0, 0x00, 0x00, 0xE0, 0xF8, 0x1E, 0x06,
	0x00, 0x00, 0x00, 0x03, 0x0F, 0x0F, 0x03, 0x00, 0x00, 0x00,

	// @834 'W' (16 pixels wide)
	//
	// ##     ##     ##
	// ##     ##     ##
	// ##    ####    ##
	//  ##   ####   ##
	//  ##  ##  ##  ##
	//  ##  ##  ##  ##
	//   ####    ####
	//   ####    ####
	//    ##      ##
	//    ##      ##
	//    ##      ##
	//
	//
	//
	0x0E, 0x7E, 0xF0, 0x80, 0x80, 0xE0, 0x78, 0x1E, 0x1E, 0x78, 0xE0, 0x80, 0x80, 0xF0, 0x7E, 0x0E,
	0x00, 0x00, 0x01, 0x0F, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0F, 0x01, 0x00, 0x00,

	// @866 'X' (11 pixels wide)
	//
	// ##       ##
	//  ##     ##
	//   ##   ##
	//    ## ##
	//     ###
	//     ###
	//     ###
	//    ## ##
	//   ##   ##
	//  ##     ##
	// ##       ##
	//
	//
	//
	0x02, 0x06, 0x0C, 0x18, 0xF0, 0xE0, 0xF0, 0x18, 0x0C, 0x06, 0x02,
	0x08, 0x0C, 0x06, 0x03, 0x01, 0x00, 0x01, 0x03, 0x06, 0x0C, 0x08,

	// @888 'Y' (10 pixels wide)
	//
	// ##      ##
	// ##      ##
	//  ##    ##
	//   ##  ##
	//    ####
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	//
	//
	//
	0x06, 0x0E, 0x18, 0x30, 0xE0, 0xE0, 0x30, 0x18, 0x0E, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,

	// @908 'Z' (8 pixels wide)
	//
	// ########
	//       ##
	//      ##
	//     ##
	//     ##
	//    ##
	//   ##
	//   ##
	//  ##
	// ##
	// ########
	//
	//
	//
	0x02, 0x02, 0x82, 0xC2, 0x72, 0x3A, 0x0E, 0x06,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08, 0x08,

	// @924 '[' (4 pixels wide)
	//
	// ####
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ####
	0xFE, 0xFE, 0x02, 0x02,
	0x7F, 0x7F, 0x40, 0x40,

	// @932 '\' (5 pixels wide)
	//
	// ##
	// ##
	//  ##
	//  ##
	//  ##
	//   ##
	//   ##
	//   ##
	//    ##
	//    ##
	//    ##
	//
	//
	//
	0x06, 0x3E, 0xF8, 0xC0, 0x00,
	0x00, 0x00, 0x01, 0x0F, 0x0E,

	// @942 ']' (4 pixels wide)
	//
	// ####
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	// ####
	0x02, 0x02, 0xFE, 0xFE,
	0x40, 0x40, 0x7F, 0x7F,

	// @950 '^' (6 pixels wide)
	//
	//   ##
	//  ####
	// ##  ##
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x08, 0x0C, 0x06, 0x06, 0x0C, 0x08,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @962 '_' (9 pixels wide)
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	// #########
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,

	// @980 '`' (3 pixels wide)
	//
	// ##
	//  ##
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	0x02, 0x06, 0x04,
	0x00, 0x00, 0x00,

	// @986 'a' (8 pixels wide)
	//
	//
	//
	//
	//  #####
	// ##   ##
	//      ##
	//  ######
	// ##   ##
	// ##   ##
	// ##   ##
	//  #######
	//
	//
	//
	0x20, 0xB0, 0x90, 0x90, 0x90, 0xF0, 0xE0, 0x00,
	0x07, 0x0F, 0x08, 0x08, 0x08, 0x0F, 0x0F, 0x08,

	// @1002 'b' (7 pixels wide)
	//
	// ##
	// ##
	// ##
	// ######
	// ###  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ###  ##
	// ######
	//
	//
	//
	0xFE, 0xFE, 0x30, 0x10, 0x10, 0xF0, 0xE0,
	0x0F, 0x0F, 0x0C, 0x08, 0x08, 0x0F, 0x07,

	// @1016 'c' (8 pixels wide)
	//
	//
	//
	//
	//   ####
	//  ##  ##
	// ##    ##
	// ##
	// ##
	// ##    ##
	//  ##  ##
	//   ####
	//
	//
	//
	0xC0, 0xE0, 0x30, 0x10, 0x10, 0x30, 0x60, 0x40,
	0x03, 0x07, 0x0C, 0x08, 0x08, 0x0C, 0x06, 0x02,

	// @1032 'd' (7 pixels wide)
	//
	//      ##
	//      ##
	//      ##
	//  ######
	// ##  ###
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##  ###
	//  ######
	//
	//
	//
	0xE0, 0xF0, 0x10, 0x10, 0x30, 0xFE, 0xFE,
	0x07, 0x0F, 0x08, 0x08, 0x0C, 0x0F, 0x0F,

	// @1046 'e' (7 pixels wide)
	//
	//
	//
	//
	//   ###
	//  ## ##
	// ##   ##
	// #######
	// ##
	// ##   ##
	//  ## ##
	//   ###
	//
	//
	//
	0xC0, 0xE0, 0xB0, 0x90, 0xB0, 0xE0, 0xC0,
	0x03, 0x07, 0x0C, 0x08, 0x0C, 0x06, 0x02,

	// @1060 'f' (5 pixels wide)
	//
	//   ###
	//  ##
	//  ##
	// ####
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//
	//
	//
	0x10, 0xFC, 0xFE, 0x12, 0x02,
	0x00, 0x0F, 0x0F, 0x00, 0x00,

	// @1070 'g' (7 pixels wide)
	//
	//
	//
	//
	//  ######
	// ##  ###
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##  ###
	//  ######
	//      ##
	//      ##
	//  #####
	0xE0, 0xF0, 0x10, 0x10, 0x30, 0xF0, 0xF0,
	0x07, 0x4F, 0x48, 0x48, 0x4C, 0x7F, 0x3F,

	// @1084 'h' (7 pixels wide)
	//
	// ##
	// ##
	// ##
	// ######
	// ###  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//
	//
	//
	0xFE, 0xFE, 0x30, 0x10, 0x10, 0xF0, 0xE0,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x0F,

	// @1098 'i' (2 pixels wide)
	//
	// ##
	//
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	0xF2, 0xF2,
	0x0F, 0x0F,

	// @1102 'j' (3 pixels wide)
	//
	//  ##
	//
	//
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	// ##
	0x00, 0xF2, 0xF2,
	0x40, 0x7F, 0x3F,

	// @1108 'k' (8 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##   ##
	// ##  ##
	// ## ##
	// ####
	// #####
	// ##  ##
	// ##   ##
	// ##    ##
	//
	//
	//
	0xFE, 0xFE, 0x80, 0xC0, 0x60, 0x30, 0x10, 0x00,
	0x0F, 0x0F, 0x01, 0x01, 0x03, 0x06, 0x0C, 0x08,

	// @1124 'l' (2 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	0xFE, 0xFE,
	0x0F, 0x0F,

	// @1128 'm' (12 pixels wide)
	//
	//
	//
	//
	// ###### ####
	// ###  ###  ##
	// ##   ##   ##
	// ##   ##   ##
	// ##   ##   ##
	// ##   ##   ##
	// ##   ##   ##
	// ##   ##   ##
	//
	//
	//
	0xF0, 0xF0, 0x30, 0x10, 0x10, 0xF0, 0xE0, 0x30, 0x10, 0x10, 0xF0, 0xE0,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x0F,

	// @1152 'n' (7 pixels wide)
	//
	//
	//
	//
	// ######
	// ###  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//
	//
	//
	0xF0, 0xF0, 0x30, 0x10, 0x10, 0xF0, 0xE0,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x0F,

	// @1166 'o' (8 pixels wide)
	//
	//
	//
	//
	//   ####
	//  ##  ##
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	//  ##  ##
	//   ####
	//
	//
	//
	0xC0, 0xE0, 0x30, 0x10, 0x10, 0x30, 0xE0, 0xC0,
	0x03, 0x07, 0x0C, 0x08, 0x08, 0x0C, 0x07, 0x03,

	// @1182 'p' (7 pixels wide)
	//
	//
	//
	//
	// ######
	// ###  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ###  ##
	// ######
	// ##
	// ##
	// ##
	0xF0, 0xF0, 0x30, 0x10, 0x10, 0xF0, 0xE0,
	0x7F, 0x7F, 0x0C, 0x08, 0x08, 0x0F, 0x07,

	// @1196 'q' (7 pixels wide)
	//
	//
	//
	//
	//  ######
	// ##  ###
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##  ###
	//  ######
	//      ##
	//      ##
	//      ##
	0xE0, 0xF0, 0x10, 0x10, 0x30, 0xF0, 0xF0,
	0x07, 0x0F, 0x08, 0x08, 0x0C, 0x7F, 0x7F,

	// @1210 'r' (5 pixels wide)
	//
	//
	//
	//
	// #####
	// ###
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	0xF0, 0xF0, 0x30, 0x10, 0x10,
	0x0F, 0x0F, 0x00, 0x00, 0x00,

	// @1220 's' (7 pixels wide)
	//
	//
	//
	//
	//  #####
	// ##   ##
	// ##
	//  ###
	//    ###
	//      ##
	// ##   ##
	//  #####
	//
	//
	//
	0x60, 0xF0, 0x90, 0x90, 0x10, 0x30, 0x20,
	0x04, 0x0C, 0x08, 0x09, 0x09, 0x0F, 0x06,

	// @1234 't' (4 pixels wide)
	//
	//
	//  ##
	//  ##
	// ####
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//  ##
	//   ##
	//
	//
	//
	0x10, 0xFC, 0xFC, 0x10,
	0x00, 0x07, 0x0F, 0x08,

	// @1242 'u' (7 pixels wide)
	//
	//
	//
	//
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##  ###
	//  ######
	//
	//
	//
	0xF0, 0xF0, 0x00, 0x00, 0x00, 0xF0, 0xF0,
	0x07, 0x0F, 0x08, 0x08, 0x0C, 0x0F, 0x0F,

	// @1256 'v' (8 pixels wide)
	//
	//
	//
	//
	// ##    ##
	// ##    ##
	//  ##  ##
	//  ##  ##
	//   ####
	//   ####
	//    ##
	//    ##
	//
	//
	//
	0x30, 0xF0, 0xC0, 0x00, 0x00, 0xC0, 0xF0, 0x30,
	0x00, 0x00, 0x03, 0x0F, 0x0F, 0x03, 0x00, 0x00,

	// @1272 'w' (12 pixels wide)
	//
	//
	//
	//
	// ##   ##   ##
	// ##   ##   ##
	//  ## #### ##
	//  ## #### ##
	//  ## #### ##
	//   ###  ###
	//   ##    ##
	//   ##    ##
	//
	//
	//
	0x30, 0xF0, 0xC0, 0x00, 0xC0, 0xF0, 0xF0, 0xC0, 0x00, 0xC0, 0xF0, 0x30,
	0x00, 0x01, 0x0F, 0x0E, 0x03, 0x01, 0x01, 0x03, 0x0E, 0x0F, 0x01, 0x00,

	// @1296 'x' (8 pixels wide)
	//
	//
	//
	//
	// ##    ##
	//  ##  ##
	//   ####
	//    ##
	//    ##
	//   ####
	//  ##  ##
	// ##    ##
	//
	//
	//
	0x10, 0x30, 0x60, 0xC0, 0xC0, 0x60, 0x30, 0x10,
	0x08, 0x0C, 0x06, 0x03, 0x03, 0x06, 0x0C, 0x08,

	// @1312 'y' (8 pixels wide)
	//
	//
	//
	//
	// ##    ##
	// ##    ##
	//  ##  ##
	//  ##  ##
	//   ####
	//   ####
	//    ##
	//    ##
	//    ##
	//   ##
	// ###
	0x30, 0xF0, 0xC0, 0x00, 0x00, 0xC0, 0xF0, 0x30,
	0x40, 0x40, 0x63, 0x3F, 0x1F, 0x03, 0x00, 0x00,

	// @1328 'z' (7 pixels wide)
	//
	//
	//
	//
	// #######
	//      ##
	//     ##
	//    ##
	//   ##
	//  ##
	// ##
	// #######
	//
	//
	//
	0x10, 0x10, 0x10, 0x90, 0xD0, 0x70, 0x30,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08,

	// @1342 '{' (6 pixels wide)
	//
	//    ###
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	// ###
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//    ###
	0x80, 0x80, 0xFC, 0x7E, 0x02, 0x02,
	0x00, 0x00, 0x3F, 0x7F, 0x40, 0x40,

	// @1354 '|' (2 pixels wide)
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	0xFE, 0xFE,
	0x7F, 0x7F,

	// @1358 '}' (6 pixels wide)
	//
	// ###
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//    ###
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	// ###
	0x02, 0x02, 0x7E, 0xFC, 0x80, 0x80,
	0x40, 0x40, 0x7F, 0x3F, 0x00, 0x00,

	// @1370 'Ą' (10 pixels wide)
	//
	//     ##
	//     ##
	//    ####
	//    ####
	//   ##  ##
	//   ##  ##
	//   ######
	//  ##    ##
	//  ##    ##
	// ##      ##
	// ##      ##
	//       ##
	//        ##
	//
	0x00, 0x00, 0xE0, 0xF8, 0x9E, 0x9E, 0xF8, 0xE0, 0x00, 0x00,
	0x0C, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x10, 0x33, 0x2F, 0x0C,

	// @1390 'Ć' (11 pixels wide)
	//        ##
	//    #####
	//  ###   ###
	//  ##     ##
	// ##       ##
	// ##
	// ##
	// ##
	// ##       ##
	//  ##     ##
	//  ###   ###
	//    #####
	//
	//
	//
	0xF0, 0xFC, 0x0C, 0x06, 0x02, 0x02, 0x02, 0x07, 0x0D, 0x1C, 0x10,
	0x01, 0x07, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x0C, 0x06, 0x07, 0x01,

	// @1412 'Ę' (9 pixels wide)
	//
	// #########
	// ##
	// ##
	// ##
	// ##
	// ########
	// ##
	// ##
	// ##
	// ##
	// #########
	//     ##
	//      ##
	//
	0xFE, 0xFE, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x02,
	0x0F, 0x0F, 0x08, 0x08, 0x18, 0x38, 0x28, 0x08, 0x08,

	// @1430 'Ł' (9 pixels wide)
	//
	//  ##
	//  ##
	//  ##  ##
	//  ## ##
	//  ####
	//  ###
	//  ##
	// ###
	//  ##
	//  ##
	//  ########
	//
	//
	//
	0x00, 0xFE, 0xFE, 0x60, 0x30, 0x18, 0x08, 0x00, 0x00,
	0x01, 0x0F, 0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

	// @1448 'Ń' (10 pixels wide)
	//      ##
	// ##  ##  ##
	// ###     ##
	// ####    ##
	// ## ##   ##
	// ## ##   ##
	// ##  ##  ##
	// ##   ## ##
	// ##   ## ##
	// ##    ####
	// ##     ###
	// ##      ##
	//
	//
	//
	0xFE, 0xFE, 0x0C, 0x3A, 0x73, 0xC1, 0x80, 0x00, 0xFE, 0xFE,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06, 0x0F, 0x0F,

	// @1468 'Ó' (11 pixels wide)
	//	    ##
	//    #####
	//  ###   ###
	//  ##     ##
	// ##       ##
	// ##       ##
	// ##       ##
	// ##       ##
	// ##       ##
	//  ##     ##
	//  ###   ###
	//    #####
	//
	//
	//
	0xF0, 0xFC, 0x0C, 0x06, 0x02, 0x02, 0x02, 0x07, 0x0D, 0xFC, 0xF0,
	0x01, 0x07, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x0C, 0x06, 0x07, 0x01,

	// @1490 'Ś' (9 pixels wide)
	//       ##
	//   #####
	//  ##   ##
	// ##     ##
	// ##
	//  ##
	//   #####
	//       ##
	//        ##
	// ##     ##
	//  ##   ##
	//   #####
	//
	//
	//
	0x18, 0x3C, 0x66, 0x42, 0x42, 0x42, 0xC7, 0x8D, 0x08,
	0x02, 0x06, 0x0C, 0x08, 0x08, 0x08, 0x0C, 0x07, 0x03,

	// @1508 'Ź' (8 pixels wide)
	//     ##
	// ########
	//   ##  ##
	//      ##
	//     ##
	//     ##
	//    ##
	//   ##
	//   ##
	//  ##
	// ##
	// ########
	//
	//
	//
	0x02, 0x02, 0x86, 0xC6, 0x73, 0x3B, 0x0E, 0x06,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08, 0x08,

	// @1524 'Ż' (8 pixels wide)
	//     ##
	// ########
	//       ##
	//      ##
	//     ##
	//     ##
	//    ##
	//   ##
	//   ##
	//  ##
	// ##
	// ########
	//
	//
	//
	0x02, 0x02, 0x82, 0xC2, 0x73, 0x3B, 0x0E, 0x06,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08, 0x08,

	// @1540 'ą' (8 pixels wide)
	//
	//
	//
	//
	//  #####
	// ##   ##
	//      ##
	//  ######
	// ##   ##
	// ##   ##
	// ##   ##
	//  #######
	//     ##
	//      ##
	//
	0x20, 0xB0, 0x90, 0x90, 0x90, 0xF0, 0xE0, 0x00,
	0x07, 0x0F, 0x08, 0x08, 0x18, 0x3F, 0x2F, 0x08,

	// @1556 'ć' (8 pixels wide)
	//
	//
	//     ##
	//    ##
	//   ####
	//  ##  ##
	// ##    ##
	// ##
	// ##
	// ##    ##
	//  ##  ##
	//   ####
	//
	//
	//
	0xC0, 0xE0, 0x30, 0x18, 0x1C, 0x34, 0x60, 0x40,
	0x03, 0x07, 0x0C, 0x08, 0x08, 0x0C, 0x06, 0x02,

	// @1572 'ę' (7 pixels wide)
	//
	//
	//
	//
	//   ###
	//  ## ##
	// ##   ##
	// #######
	// ##
	// ##   ##
	//  ## ##
	//   ###
	//    ##
	//     ##
	//
	0xC0, 0xE0, 0xB0, 0x90, 0xB0, 0xE0, 0xC0,
	0x03, 0x07, 0x0C, 0x18, 0x3C, 0x26, 0x02,

	// @1586 'ł' (5 pixels wide)
	//
	//  ##
	//  ##
	//  ##
	//  ## #
	//  ###
	//  ##
	// ###
	//  ##
	//  ##
	//  ##
	//  ##
	//
	//
	//
	0x80, 0xFE, 0xFE, 0x40, 0x20,
	0x00, 0x0F, 0x0F, 0x00, 0x00,

	// @1596 'ń' (7 pixels wide)
	//
	//
	//     ##
	//    ##
	// ######
	// ###  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##   ##
	//
	//
	//
	0xF0, 0xF0, 0x30, 0x18, 0x1C, 0xF4, 0xE0,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x0F,

	// @1610 'ó' (8 pixels wide)
	//
	//
	//      ##
	//     ##
	//   ####
	//  ##  ##
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	//  ##  ##
	//   ####
	//
	//
	//
	0xC0, 0xE0, 0x30, 0x10, 0x18, 0x3C, 0xE4, 0xC0,
	0x03, 0x07, 0x0C, 0x08, 0x08, 0x0C, 0x07, 0x03,

	// @1626 'ś' (7 pixels wide)
	//
	//
	//     ##
	//    ##
	//  #####
	// ##   ##
	// ##
	//  ###
	//    ###
	//      ##
	// ##   ##
	//  #####
	//
	//
	//
	0x60, 0xF0, 0x90, 0x98, 0x1C, 0x34, 0x20,
	0x04, 0x0C, 0x08, 0x09, 0x09, 0x0F, 0x06,

	// @1640 'ź' (7 pixels wide)
	//
	//
	//     ##
	//    ##
	// #######
	//  #   ##
	//     ##
	//    ##
	//   ##
	//  ##
	// ##
	// #######
	//
	//
	//
	0x10, 0x30, 0x10, 0x98, 0xDC, 0x74, 0x30,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08,

	// @1654 'ż' (7 pixels wide)
	//
	//
	//    ##
	//
	// #######
	//      ##
	//     ##
	//    ##
	//   ##
	//  ##
	// ##
	// #######
	//
	//
	//
	0x10, 0x10, 0x10, 0x94, 0xD4, 0x70, 0x30,
	0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x08, 0x08,
};

/* Character descriptors for Consolas 12pt */
/* { [Char width in bits], [Offset into consolas_12ptCharBitmaps in bytes] } */
const TFontCharInfo sansserif_11ptDescriptors[] =
{
	{6, 0}, 		/*   */
	{2, 12}, 		/* ! */
	{5, 16}, 		/* " */
	{9, 26}, 		/* # */
	{7, 44}, 		/* $ */
	{12, 58}, 		/* % */
	{9, 82}, 		/* & */
	{2, 100}, 		/* ' */
	{5, 104}, 		/* ( */
	{5, 114}, 		/* ) */
	{6, 124}, 		/* * */
	{8, 136}, 		/* + */
	{3, 152}, 		/* , */
	{5, 158}, 		/* - */
	{2, 168}, 		/* . */
	{5, 172}, 		/* / */
	{7, 182}, 		/* 0 */
	{4, 196}, 		/* 1 */
	{7, 204}, 		/* 2 */
	{7, 218}, 		/* 3 */
	{7, 232}, 		/* 4 */
	{7, 246}, 		/* 5 */
	{7, 260}, 		/* 6 */
	{7, 274}, 		/* 7 */
	{7, 288}, 		/* 8 */
	{7, 302}, 		/* 9 */
	{2, 316}, 		/* : */
	{3, 320}, 		/* ; */
	{8, 326}, 		/* < */
	{8, 342}, 		/* = */
	{8, 358}, 		/* > */
	{7, 374}, 		/* ? */
	{14, 388}, 		/* @ */
	{10, 416}, 		/* A */
	{9, 436}, 		/* B */
	{11, 454}, 		/* C */
	{10, 476}, 		/* D */
	{9, 496}, 		/* E */
	{9, 514}, 		/* F */
	{11, 532}, 		/* G */
	{10, 554}, 		/* H */
	{2, 574}, 		/* I */
	{7, 578}, 		/* J */
	{10, 592}, 		/* K */
	{8, 612}, 		/* L */
	{12, 628}, 		/* M */
	{10, 652}, 		/* N */
	{11, 672}, 		/* O */
	{10, 694}, 		/* P */
	{11, 714}, 		/* Q */
	{10, 736}, 		/* R */
	{9, 756}, 		/* S */
	{10, 774}, 		/* T */
	{10, 794}, 		/* U */
	{10, 814}, 		/* V */
	{16, 834}, 		/* W */
	{11, 866}, 		/* X */
	{10, 888}, 		/* Y */
	{8, 908}, 		/* Z */
	{4, 924}, 		/* [ */
	{5, 932}, 		/* \ */
	{4, 942}, 		/* ] */
	{6, 950}, 		/* ^ */
	{9, 962}, 		/* _ */
	{3, 980}, 		/* ` */
	{8, 986}, 		/* a */
	{7, 1002}, 		/* b */
	{8, 1016}, 		/* c */
	{7, 1032}, 		/* d */
	{7, 1046}, 		/* e */
	{5, 1060}, 		/* f */
	{7, 1070}, 		/* g */
	{7, 1084}, 		/* h */
	{2, 1098}, 		/* i */
	{3, 1102}, 		/* j */
	{8, 1108}, 		/* k */
	{2, 1124}, 		/* l */
	{12, 1128}, 		/* m */
	{7, 1152}, 		/* n */
	{8, 1166}, 		/* o */
	{7, 1182}, 		/* p */
	{7, 1196}, 		/* q */
	{5, 1210}, 		/* r */
	{7, 1220}, 		/* s */
	{4, 1234}, 		/* t */
	{7, 1242}, 		/* u */
	{8, 1256}, 		/* v */
	{12, 1272}, 		/* w */
	{8, 1296}, 		/* x */
	{8, 1312}, 		/* y */
	{7, 1328}, 		/* z */
	{6, 1342}, 		/* { */
	{2, 1354}, 		/* | */
	{6, 1358}, 		/* } */
	{0, 0}, 		/* ~   brak */
	{0, 0}, 		/* DEL brak */
    {10, 1370}, 	/* Ą \200*/
    {11, 1390}, 	/* Ć \201*/
    {9, 1412}, 		/* Ę \202*/
    {9, 1430}, 		/* Ł \203*/
    {10, 1448}, 	/* Ń \204*/
    {11, 1468}, 	/* Ó \205*/
    {9, 1490}, 		/* Ś \206*/
    {8, 1508}, 		/* Ź \207*/
    {8, 1524}, 		/* Ż \210*/
    {8, 1540}, 		/* ą \211*/
    {8, 1556}, 		/* ć \212*/
    {7, 1572}, 		/* ę \213*/
    {5, 1586}, 		/* ł \214*/
    {7, 1596}, 		/* ń \215*/
    {8, 1610}, 		/* ó \216*/
    {7, 1626}, 		/* ś \217*/
    {7, 1640}, 		/* ź \220*/
    {7, 1654}, 		/* ż \221*/
};


/* Font information for Sans Serif 11pt */
const TFont sansserif_11pt =
{
	15, /*  Character height */
	1, /* space */
	' ', /*  Start character */
	'}'+20, /*  End character + "~" + DEL i 18 polskich znakow*/
    //'}'+20, /*  End character + "~" + DEL i 18 polskich znakow*/
	sansserif_11ptDescriptors, /*  Character descriptor array */
	sansserif_11ptBitmaps, /*  Character bitmap array */
};


#endif // FONT_SANSSERIF11PT_H_INCLUDED


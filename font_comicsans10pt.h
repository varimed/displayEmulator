#ifndef FONT_COMICSANS10PT_H_INCLUDED
#define FONT_COMICSANS10PT_H_INCLUDED


#include "fonttypes.h"
#include "inttypes.h"

/*
**  Font data for Comic Sans MS 10pt
*/

/* Character bitmaps for Comic Sans MS 10pt */
const uint8_t comicSansMS_10ptBitmaps[] =
{
	/* @0 ' ' (8 pixels wide) */
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
	//
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* @16 '!' (2 pixels wide) */
	//
	//
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//
	// ##
	// ##
	//
	//
	//
	//
	0xFC, 0xFC,
	0x0D, 0x0D,

	/* @20 '"' (5 pixels wide) */
	//
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
	0x3C, 0x3C, 0x00, 0x3C, 0x3C,
	0x00, 0x00, 0x00, 0x00, 0x00,

	/* @30 '#' (9 pixels wide) */
	//
	//
	//    ## ##
	//    ## ##
	//  ########
	//  ########
	//   ## ##
	//   ## ##
	// #########
	// #########
	//  ## ##
	//  ## ##
	//
	//
	//
	//
	0x00, 0x30, 0xF0, 0xFC, 0x3C, 0xF0, 0xFC, 0x3C, 0x30,
	0x03, 0x0F, 0x0F, 0x03, 0x0F, 0x0F, 0x03, 0x03, 0x03,

	/* @48 '$' (8 pixels wide) */
	//    ##
	//    ##
	//   #####
	//  ######
	// ## ##
	// ## ##
	// #######
	//  #######
	//    ## ##
	//    ## ##
	// ########
	// ######
	//    ##
	//    ##
	//
	//
	0x70, 0xF8, 0xCC, 0xFF, 0xFF, 0xCC, 0xCC, 0x80,
	0x0C, 0x0C, 0x0C, 0x3F, 0x3F, 0x0C, 0x07, 0x07,

	/* @64 '%' (11 pixels wide) */
	//
	//
	//  ###  ##
	// ##### ##
	// ## ####
	// #######
	//  #####
	//     ## ###
	//    ## #####
	//    ## ## ##
	//   ##  #####
	//   ##   ###
	//
	//
	//
	//
	0x38, 0x7C, 0x6C, 0x7C, 0xF8, 0xF0, 0x3C, 0x8C, 0x80, 0x80, 0x00,
	0x00, 0x00, 0x0C, 0x0F, 0x03, 0x00, 0x07, 0x0F, 0x0D, 0x0F, 0x07,

	/* @86 '&' (9 pixels wide) */
	//
	//
	//    ###
	//   #####
	//   ## ##
	//   ####
	//  ####
	// ### ## ##
	// ##  #####
	// ##   ###
	// ########
	//  ##### ##
	//
	//
	//
	//
	0x80, 0xC0, 0xF8, 0x7C, 0xEC, 0xBC, 0x18, 0x80, 0x80,
	0x07, 0x0F, 0x0C, 0x0C, 0x0D, 0x0F, 0x07, 0x0F, 0x09,

	/* @104 ''' (2 pixels wide) */
	//
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
	0x3C, 0x3C,
	0x00, 0x00,

	/* @108 '(' (4 pixels wide) */
	//
	//
	//   ##
	//   ##
	//  ##
	//  ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	//  ##
	//  ###
	//   ##
	0xC0, 0xF0, 0x3C, 0x0C,
	0x1F, 0x7F, 0xE0, 0xC0,

	/* @116 ')' (4 pixels wide) */
	//
	//
	// ##
	// ##
	//  ##
	//  ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//  ##
	// ###
	// ##
	0x0C, 0x3C, 0xF0, 0xC0,
	0xC0, 0xE0, 0x7F, 0x1F,

	/* @124 '*' (6 pixels wide) */
	//
	//
	//   ##
	// ######
	//  ####
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
	0x48, 0x78, 0x3C, 0x3C, 0x78, 0x48,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* @136 '+' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ##
	//   ##
	// ######
	// ######
	//   ##
	//   ##
	//
	//
	//
	//
	//
	0x80, 0x80, 0xE0, 0xE0, 0x80, 0x80,
	0x01, 0x01, 0x07, 0x07, 0x01, 0x01,

	/* @148 ',' (2 pixels wide) */
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
	//  #
	// ##
	// #
	//
	//
	0x00, 0x00,
	0x30, 0x18,

	/* @152 '-' (5 pixels wide) */
	//
	//
	//
	//
	//
	//
	//
	// #####
	// #####
	//
	//
	//
	//
	//
	//
	//
	0x80, 0x80, 0x80, 0x80, 0x80,
	0x01, 0x01, 0x01, 0x01, 0x01,

	/* @162 '.' (2 pixels wide) */
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
	// ##
	//
	//
	//
	//
	0x00, 0x00,
	0x0C, 0x0C,

	/* @166 '/' (6 pixels wide) */
	//
	//
	//      #
	//     ##
	//     #
	//    ##
	//    ##
	//   ##
	//   #
	//  ##
	//  #
	// ##
	// ##
	//
	//
	//
	0x00, 0x00, 0x80, 0xE0, 0x78, 0x0C,
	0x18, 0x1E, 0x03, 0x00, 0x00, 0x00,

	/* @178 '0' (6 pixels wide) */
	//
	//
	//   ###
	//  ####
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	//  ####
	//  ####
	//
	//
	//
	//
	0xF0, 0xF8, 0x0C, 0x0C, 0xFC, 0xF0,
	0x03, 0x0F, 0x0C, 0x0C, 0x0F, 0x03,

	/* @190 '1' (6 pixels wide) */
	//
	//
	//   ##
	// ####
	// ####
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	// ######
	// ######
	//
	//
	//
	//
	0x18, 0x18, 0xFC, 0xFC, 0x00, 0x00,
	0x0C, 0x0C, 0x0F, 0x0F, 0x0C, 0x0C,

	/* @202 '2' (7 pixels wide) */
	//
	//
	//   ####
	// #######
	// ##   ##
	//      ##
	//    ###
	//   ###
	//  ##
	// ##
	// #######
	// #######
	//
	//
	//
	//
	0x18, 0x18, 0x8C, 0xCC, 0xCC, 0x7C, 0x38,
	0x0E, 0x0F, 0x0D, 0x0C, 0x0C, 0x0C, 0x0C,

	/* @216 '3' (6 pixels wide) */
	//
	//
	//  ####
	// ######
	// ##  ##
	//     ##
	//  ####
	//  #####
	//     ##
	// ##  ##
	// ######
	//  ####
	//
	//
	//
	//
	0x18, 0xDC, 0xCC, 0xCC, 0xFC, 0xB8,
	0x06, 0x0E, 0x0C, 0x0C, 0x0F, 0x07,

	/* @228 '4' (7 pixels wide) */
	//
	//
	//     ##
	//    ###
	//    ###
	//   # ##
	//  #  ##
	// #######
	// #######
	//     ##
	//     ##
	//     ##
	//
	//
	//
	//
	0x80, 0xC0, 0xA0, 0x98, 0xFC, 0xFC, 0x80,
	0x01, 0x01, 0x01, 0x01, 0x0F, 0x0F, 0x01,

	/* @242 '5' (6 pixels wide) */
	//
	//
	// ######
	// ######
	// ##
	// #####
	// ######
	//     ##
	//     ##
	// #   ##
	// #####
	//  ###
	//
	//
	//
	//
	0x7C, 0x7C, 0x6C, 0x6C, 0xEC, 0xCC,
	0x06, 0x0C, 0x0C, 0x0C, 0x07, 0x03,

	/* @254 '6' (6 pixels wide) */
	//
	//
	//    ##
	//   ###
	//  ###
	// ###
	// #####
	// ######
	// ##  ##
	// ##  ##
	// ######
	//  ####
	//
	//
	//
	//
	0xE0, 0xF0, 0xF8, 0xDC, 0xCC, 0x80,
	0x07, 0x0F, 0x0C, 0x0C, 0x0F, 0x07,

	/* @266 '7' (7 pixels wide) */
	//
	//
	// #######
	// #######
	//      #
	//     #
	//    ##
	//    #
	//   ##
	//   ##
	//  ##
	//  ##
	//
	//
	//
	//
	0x0C, 0x0C, 0x0C, 0xCC, 0x6C, 0x1C, 0x0C,
	0x00, 0x0C, 0x0F, 0x03, 0x00, 0x00, 0x00,

	/* @280 '8' (6 pixels wide) */
	//
	//
	//  ####
	// ######
	// ##  ##
	// ##  ##
	//  ####
	//  ####
	// ##  ##
	// ##  ##
	// ######
	//  ####
	//
	//
	//
	//
	0x38, 0xFC, 0xCC, 0xCC, 0xFC, 0x38,
	0x07, 0x0F, 0x0C, 0x0C, 0x0F, 0x07,

	/* @292 '9' (6 pixels wide) */
	//
	//
	//  ####
	// ######
	// ##  ##
	// ##  ##
	// ##  ##
	// ######
	//  ####
	//    ##
	//  ###
	//  ##
	//
	//
	//
	//
	0xF8, 0xFC, 0x8C, 0x8C, 0xFC, 0xF8,
	0x00, 0x0D, 0x0D, 0x07, 0x03, 0x00,

	/* @304 ':' (2 pixels wide) */
	//
	//
	//
	//
	//
	// ##
	// ##
	//
	//
	// ##
	// ##
	//
	//
	//
	//
	//
	0x60, 0x60,
	0x06, 0x06,

	/* @308 ';' (3 pixels wide) */
	//
	//
	//
	//
	//
	//  ##
	//  ##
	//
	//
	//
	//  ##
	// ###
	// ##
	//
	//
	//
	0x00, 0x60, 0x60,
	0x18, 0x1C, 0x0C,

	/* @314 '<' (4 pixels wide) */
	//
	//
	//
	//
	//
	//    #
	//   ##
	//  ##
	// ##
	//  ##
	//   ##
	//
	//
	//
	//
	//
	0x00, 0x80, 0xC0, 0x60,
	0x01, 0x03, 0x06, 0x04,

	/* @322 '=' (5 pixels wide) */
	//
	//
	//
	//
	//
	// #####
	// #####
	//
	//
	// #####
	// #####
	//
	//
	//
	//
	//
	0x60, 0x60, 0x60, 0x60, 0x60,
	0x06, 0x06, 0x06, 0x06, 0x06,

	/* @332 '>' (4 pixels wide) */
	//
	//
	//
	//
	//
	// #
	// ##
	//  ###
	//   ##
	//  ##
	// ##
	//
	//
	//
	//
	//
	0x60, 0xC0, 0x80, 0x80,
	0x04, 0x06, 0x03, 0x01,

	/* @340 '?' (7 pixels wide) */
	//
	//
	//  ####
	// ######
	// ##  ###
	//      ##
	//     ##
	//   ###
	//   ##
	//
	//   ##
	//   ##
	//
	//
	//
	//
	0x18, 0x1C, 0x8C, 0x8C, 0xDC, 0x78, 0x30,
	0x00, 0x00, 0x0D, 0x0D, 0x00, 0x00, 0x00,

	/* @354 '@' (12 pixels wide) */
	//
	//
	//    ######
	//   #########
	//  ###     ###
	// ###  ##   ##
	// ## ## ##  ##
	// ## #########
	// ##  #######
	// ###
	//  ###   ##
	//   #######
	//    #####
	//
	//
	//
	0xE0, 0xF0, 0x38, 0xDC, 0xCC, 0xAC, 0xEC, 0xCC, 0x8C, 0x98, 0xF8, 0xF0,
	0x03, 0x07, 0x0E, 0x1C, 0x19, 0x19, 0x19, 0x1D, 0x0D, 0x01, 0x01, 0x00,

	/* @378 'A' (8 pixels wide) */
	//
	//
	//      #
	//     ##
	//    ###
	//    ####
	//   ## ##
	//   #####
	//  ######
	//  ##   #
	// ##    ##
	// ##    ##
	//
	//
	//
	//
	0x00, 0x00, 0xC0, 0xF0, 0xB8, 0xFC, 0xE0, 0x00,
	0x0C, 0x0F, 0x03, 0x01, 0x01, 0x01, 0x0F, 0x0C,

	/* @394 'B' (7 pixels wide) */
	//
	//
	// ####
	// ######
	// ##  ##
	// ##  ##
	// #####
	// ######
	// ##   ##
	// ##  ###
	// ######
	// #####
	//
	//
	//
	//
	0xFC, 0xFC, 0xCC, 0xCC, 0xF8, 0xB8, 0x00,
	0x0F, 0x0F, 0x0C, 0x0C, 0x0E, 0x07, 0x03,

	/* @408 'C' (8 pixels wide) */
	//
	//
	//     ####
	//   ######
	//   ##  ##
	//  ##
	// ###
	// ##
	// ##
	// ##    ##
	//  #######
	//   ####
	//
	//
	//
	//
	0xC0, 0xE0, 0x78, 0x18, 0x0C, 0x0C, 0x1C, 0x1C,
	0x03, 0x07, 0x0C, 0x0C, 0x0C, 0x0C, 0x06, 0x06,

	/* @424 'D' (7 pixels wide) */
	//
	//
	// ##
	// ####
	// ## ##
	// ##  ##
	// ##   ##
	// ##   ##
	// ##   ##
	// ##  ###
	// ######
	//  ####
	//
	//
	//
	//
	0xFC, 0xFC, 0x08, 0x18, 0x30, 0xE0, 0xC0,
	0x07, 0x0F, 0x0C, 0x0C, 0x0E, 0x07, 0x03,

	/* @438 'E' (7 pixels wide) */
	//
	//
	// #######
	// #######
	// ##
	// ##
	// #######
	// #######
	// ##
	// ##
	// #######
	//  ######
	//
	//
	//
	//
	0xFC, 0xFC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
	0x07, 0x0F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,

	/* @452 'F' (6 pixels wide) */
	//
	//
	// ######
	// ######
	// ##
	// ##
	// ######
	// ######
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	//
	0xFC, 0xFC, 0xCC, 0xCC, 0xCC, 0xCC,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,

	/* @464 'G' (9 pixels wide) */
	//
	//
	//    ####
	//   ######
	//  ###  ##
	//  ##
	// ##
	// ##  #####
	// ## ######
	// ##    ###
	// ########
	//  #####
	//
	//
	//
	//
	0xC0, 0xF0, 0x38, 0x1C, 0x8C, 0x8C, 0x9C, 0x98, 0x80,
	0x07, 0x0F, 0x0C, 0x0D, 0x0D, 0x0D, 0x07, 0x07, 0x03,

	/* @482 'H' (8 pixels wide) */
	//
	//
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	// ########
	// ########
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	//
	//
	//
	//
	0xFC, 0xFC, 0xC0, 0xC0, 0xC0, 0xC0, 0xFC, 0xFC,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F,

	/* @498 'I' (6 pixels wide) */
	//
	//
	// ######
	// ######
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	// ######
	// ######
	//
	//
	//
	//
	0x0C, 0x0C, 0xFC, 0xFC, 0x0C, 0x0C,
	0x0C, 0x0C, 0x0F, 0x0F, 0x0C, 0x0C,

	/* @510 'J' (8 pixels wide) */
	//
	//
	//  #######
	//  #######
	//     ##
	//     ##
	//     ##
	//     ##
	//     ##
	// ##  ##
	// ##  ##
	//  #####
	//   ###
	//
	//
	//
	0x00, 0x0C, 0x0C, 0x0C, 0xFC, 0xFC, 0x0C, 0x0C,
	0x06, 0x0E, 0x18, 0x18, 0x1F, 0x0F, 0x00, 0x00,

	/* @526 'K' (7 pixels wide) */
	//
	//
	// ##   ##
	// ##  ###
	// ## ###
	// #####
	// ####
	// ###
	// ####
	// ## ###
	// ##  ###
	// ##   ##
	//
	//
	//
	//
	0xFC, 0xFC, 0xE0, 0x70, 0x38, 0x1C, 0x0C,
	0x0F, 0x0F, 0x01, 0x03, 0x06, 0x0E, 0x0C,

	/* @540 'L' (6 pixels wide) */
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
	// ######
	// ######
	//
	//
	//
	//
	0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00,
	0x0F, 0x0F, 0x0C, 0x0C, 0x0C, 0x0C,

	/* @552 'M' (10 pixels wide) */
	//
	//
	//   ##   ##
	//   ##   ##
	//   ##   ##
	//  #### ###
	//  #### ###
	//  #### # ##
	//  ###### ##
	// ##  ### ##
	// ##  ##  ##
	// ##  ##  ##
	//
	//
	//
	//
	0x00, 0xE0, 0xFC, 0xFC, 0xE0, 0x00, 0xE0, 0x7C, 0xFC, 0x80,
	0x0E, 0x0F, 0x01, 0x01, 0x0F, 0x0F, 0x03, 0x00, 0x0F, 0x0F,

	/* @572 'N' (9 pixels wide) */
	//
	//
	// ##     ##
	// ##     ##
	// ###    ##
	// ####   ##
	// ## #   ##
	// ##  #  ##
	// ##  ## ##
	// ##   ####
	// ##    ###
	// ##     ##
	//
	//
	//
	//
	0xFC, 0xFC, 0x30, 0x60, 0x80, 0x00, 0x00, 0xFC, 0xFC,
	0x0F, 0x0F, 0x00, 0x00, 0x01, 0x03, 0x06, 0x0F, 0x0F,

	/* @590 'O' (10 pixels wide) */
	//
	//
	//    #####
	//   #######
	//  ###   ###
	// ###     ##
	// ##      ##
	// ##      ##
	// ##     ###
	// ###   ###
	//  #######
	//   #####
	//
	//
	//
	//
	0xE0, 0xF0, 0x38, 0x1C, 0x0C, 0x0C, 0x0C, 0x1C, 0xF8, 0xF0,
	0x03, 0x07, 0x0E, 0x0C, 0x0C, 0x0C, 0x0E, 0x07, 0x03, 0x01,

	/* @610 'P' (5 pixels wide) */
	//
	//
	// ####
	// #####
	// ## ##
	// ## ##
	// #####
	// ####
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	//
	0xFC, 0xFC, 0xCC, 0xFC, 0x78,
	0x0F, 0x0F, 0x00, 0x00, 0x00,

	/* @620 'Q' (11 pixels wide) */
	//
	//
	//     ####
	//   ########
	//  ###    ##
	// ###     ###
	// ##       ##
	// ##       ##
	// ##       ##
	// ###  ##  ##
	//  ### ## ##
	//   #######
	//    #######
	//         ###
	//          ##
	//
	0xE0, 0xF0, 0x38, 0x18, 0x0C, 0x0C, 0x0C, 0x0C, 0x38, 0xF8, 0xE0,
	0x03, 0x07, 0x0E, 0x1C, 0x18, 0x1E, 0x1E, 0x18, 0x3C, 0x77, 0x63,

	/* @642 'R' (6 pixels wide) */
	//
	//
	// ####
	// #####
	// ## ###
	// ##  ##
	// ##  ##
	// #####
	// ####
	// ## ##
	// ##  ##
	// ##   #
	//
	//
	//
	//
	0xFC, 0xFC, 0x8C, 0x9C, 0xF8, 0x70,
	0x0F, 0x0F, 0x01, 0x03, 0x06, 0x0C,

	/* @654 'S' (8 pixels wide) */
	//
	//
	//    ###
	//   #####
	//  ##
	//  ##
	//  ######
	//   ######
	//       ##
	// ##   ###
	// #######
	//  #####
	//
	//
	//
	//
	0x00, 0x70, 0xF8, 0xCC, 0xCC, 0xCC, 0xC8, 0x80,
	0x06, 0x0E, 0x0C, 0x0C, 0x0C, 0x0E, 0x07, 0x03,

	/* @670 'T' (8 pixels wide) */
	//
	//
	// ########
	// ########
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//
	//
	//
	//
	0x0C, 0x0C, 0x0C, 0xFC, 0xFC, 0x0C, 0x0C, 0x0C,
	0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00,

	/* @686 'U' (8 pixels wide) */
	//
	//
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	// ##    ##
	// ##   ##
	// ###  ##
	//  #####
	//   ###
	//
	//
	//
	//
	0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC,
	0x03, 0x07, 0x0E, 0x0C, 0x0C, 0x07, 0x03, 0x00,

	/* @702 'V' (9 pixels wide) */
	//
	//
	// ##     ##
	// ##    ###
	// ###   ##
	//  ##  ###
	//  ##  ##
	//  ##  ##
	//   ####
	//   ####
	//    ###
	//    ##
	//
	//
	//
	//
	0x1C, 0xFC, 0xF0, 0x00, 0x00, 0xE0, 0xF8, 0x3C, 0x0C,
	0x00, 0x00, 0x03, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00,

	/* @720 'W' (13 pixels wide) */
	//
	//
	// ##   ##    ##
	// ##   ##    ##
	// ##   ##   ##
	//  ## ###   ##
	//  ## #### ##
	//  #### ## ##
	//  #### ## #
	//   ###  ###
	//   ##   ###
	//   ##   ##
	//
	//
	//
	//
	0x1C, 0xFC, 0xE0, 0x80, 0xE0, 0x7C, 0xFC, 0xC0, 0x00, 0xC0, 0xF0, 0x3C, 0x0C,
	0x00, 0x01, 0x0F, 0x0F, 0x03, 0x00, 0x01, 0x0F, 0x0E, 0x07, 0x00, 0x00, 0x00,

	/* @746 'X' (9 pixels wide) */
	//
	//
	// ##     ##
	// ###   ###
	//  ### ###
	//   #####
	//    ###
	//    ###
	//   #####
	//  ###  ##
	// ###   ###
	// ##     ##
	//
	//
	//
	//
	0x0C, 0x1C, 0x38, 0xF0, 0xE0, 0xF0, 0x38, 0x1C, 0x0C,
	0x0C, 0x0E, 0x07, 0x03, 0x01, 0x01, 0x07, 0x0E, 0x0C,

	/* @764 'Y' (8 pixels wide) */
	//
	//
	// ##    ##
	// ###   ##
	//  ##  ##
	//  ##  ##
	//   ####
	//    ###
	//    ###
	//    ##
	//   ###
	//   ##
	//
	//
	//
	//
	0x0C, 0x3C, 0x78, 0xC0, 0xC0, 0xF0, 0x3C, 0x0C,
	0x00, 0x00, 0x0C, 0x0F, 0x07, 0x01, 0x00, 0x00,

	/* @780 'Z' (8 pixels wide) */
	//
	//
	// ########
	// ########
	//      #
	//     ##
	//    ##
	//   ##
	//   #
	//  ##
	// ########
	// ########
	//
	//
	//
	//
	0x0C, 0x0C, 0x8C, 0xCC, 0x6C, 0x3C, 0x0C, 0x0C,
	0x0C, 0x0E, 0x0F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,

	/* @796 '[' (4 pixels wide) */
	//
	//
	// ####
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
	// ####
	// ####
	0xFC, 0xFC, 0x0C, 0x0C,
	0xFF, 0xFF, 0xC0, 0xC0,

	/* @804 '\' (6 pixels wide) */
	//
	//
	// ##
	// ##
	//  ##
	//  ##
	//   ##
	//   ##
	//    #
	//    ##
	//    ##
	//     ##
	//     ##
	//
	//
	//
	0x0C, 0x3C, 0xF0, 0xC0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x07, 0x1E, 0x18,

	/* @816 ']' (4 pixels wide) */
	//
	//
	// ####
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
	// ####
	// ####
	0x0C, 0x0C, 0xFC, 0xFC,
	0xC0, 0xC0, 0xFF, 0xFF,

	/* @824 '^' (6 pixels wide) */
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
	//
	0x08, 0x0C, 0x06, 0x06, 0x0C, 0x08,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* @836 '_' (9 pixels wide) */
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
	//
	//
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,

	/* @854 '`' (3 pixels wide) */
	//
	// ##
	//  ##
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
	0x02, 0x0E, 0x0C,
	0x00, 0x00, 0x00,

	/* @860 'a' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ###
	//  #####
	// ### ##
	// ##  ##
	// ##  ##
	// ######
	//  #####
	//
	//
	//
	//
	0x80, 0xC0, 0xE0, 0x60, 0xE0, 0xC0,
	0x07, 0x0F, 0x0C, 0x0C, 0x0F, 0x0F,

	/* @872 'b' (6 pixels wide) */
	//
	//
	// ##
	// ##
	// ##
	// #####
	// ######
	// ##  ##
	// ##  ##
	// ##  ##
	// #####
	// ####
	//
	//
	//
	//
	0xFC, 0xFC, 0x60, 0x60, 0xE0, 0xC0,
	0x0F, 0x0F, 0x0C, 0x0C, 0x07, 0x03,

	/* @884 'c' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ###
	//  #####
	// ###  #
	// ##
	// ##   #
	// ######
	//  ####
	//
	//
	//
	//
	0x80, 0xC0, 0xE0, 0x60, 0x60, 0xC0,
	0x07, 0x0F, 0x0C, 0x0C, 0x0C, 0x06,

	/* @896 'd' (7 pixels wide) */
	//
	//
	//      ##
	//      ##
	//      ##
	//   #####
	//  ######
	// ##   ##
	// ##   ##
	// ##   ##
	// #######
	//   #####
	//
	//
	//
	//
	0x80, 0xC0, 0x60, 0x60, 0x60, 0xFC, 0xFC,
	0x07, 0x07, 0x0C, 0x0C, 0x0C, 0x0F, 0x0F,

	/* @910 'e' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ###
	//  #####
	// ##  ##
	// ## ###
	// ####
	// ######
	//  ####
	//
	//
	//
	//
	0x80, 0xC0, 0x60, 0x60, 0xE0, 0xC0,
	0x07, 0x0F, 0x0E, 0x0F, 0x0D, 0x05,

	/* @922 'f' (6 pixels wide) */
	//
	//
	//    ###
	//   ####
	//   ##
	// ######
	// ######
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//
	//
	//
	0x60, 0x60, 0xF8, 0xFC, 0x6C, 0x6C,
	0x00, 0x00, 0x1F, 0x1F, 0x00, 0x00,

	/* @934 'g' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ###
	//  #####
	// ##  ##
	// ##  ##
	// ## ###
	// ######
	//  #####
	//     ##
	//    ###
	// #####
	// ####
	0x80, 0xC0, 0x60, 0x60, 0xE0, 0xC0,
	0xC7, 0xCF, 0xCC, 0xEE, 0x7F, 0x3F,

	/* @946 'h' (6 pixels wide) */
	//
	//
	// ##
	// ##
	// ##
	// #####
	// ######
	// ### ##
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	//
	//
	//
	//
	0xFC, 0xFC, 0xE0, 0x60, 0xE0, 0xC0,
	0x0F, 0x0F, 0x00, 0x00, 0x0F, 0x0F,

	/* @958 'i' (2 pixels wide) */
	//
	//
	// ##
	// ##
	//
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
	//
	0xEC, 0xEC,
	0x0F, 0x0F,

	/* @962 'j' (5 pixels wide) */
	//
	//
	//    ##
	//    ##
	//
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	//    ##
	// ## ##
	// ## ##
	//  ####
	//  ###
	0x00, 0x00, 0x00, 0xEC, 0xEC,
	0x30, 0xF0, 0xC0, 0xFF, 0x7F,

	/* @972 'k' (6 pixels wide) */
	//
	//
	// ##
	// ##
	// ##
	// ##  ##
	// ## ###
	// #####
	// ####
	// ## ##
	// ##  ##
	// ##  ##
	//
	//
	//
	//
	0xFC, 0xFC, 0x80, 0xC0, 0xE0, 0x60,
	0x0F, 0x0F, 0x01, 0x03, 0x0E, 0x0C,

	/* @984 'l' (2 pixels wide) */
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
	// ##
	// ##
	//
	//
	//
	//
	0xFC, 0xFC,
	0x0F, 0x0F,

	/* @988 'm' (8 pixels wide) */
	//
	//
	//
	//
	//
	// ## #####
	// ########
	// ##### ##
	// ## ## ##
	// ## ## ##
	// ## ## ##
	// ## ## ##
	//
	//
	//
	//
	0xE0, 0xE0, 0xC0, 0xE0, 0xE0, 0x60, 0xE0, 0xE0,
	0x0F, 0x0F, 0x00, 0x0F, 0x0F, 0x00, 0x0F, 0x0F,

	/* @1004 'n' (6 pixels wide) */
	//
	//
	//
	//
	//
	// ## ##
	// ######
	// ### ##
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	//
	//
	//
	//
	0xE0, 0xE0, 0xC0, 0x60, 0xE0, 0xC0,
	0x0F, 0x0F, 0x00, 0x00, 0x0F, 0x0F,

	/* @1016 'o' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ###
	//  #####
	// ##  ##
	// ##  ##
	// ##  ##
	// #####
	//  ###
	//
	//
	//
	//
	0x80, 0xC0, 0x60, 0x60, 0xE0, 0xC0,
	0x07, 0x0F, 0x0C, 0x0C, 0x07, 0x03,

	/* @1028 'p' (6 pixels wide) */
	//
	//
	//
	//
	//
	// #####
	// ######
	// ##  ##
	// ##  ##
	// ##  ##
	// ######
	// #####
	// ##
	// ##
	// ##
	// ##
	0xE0, 0xE0, 0x60, 0x60, 0xE0, 0xC0,
	0xFF, 0xFF, 0x0C, 0x0C, 0x0F, 0x07,

	/* @1040 'q' (6 pixels wide) */
	//
	//
	//
	//
	//
	//   ####
	//  #####
	// ### ##
	// ##  ##
	// ##  ##
	// ######
	//  #####
	//     ##
	//     ##
	//     ##
	//     ##
	0x80, 0xC0, 0xE0, 0x60, 0xE0, 0xE0,
	0x07, 0x0F, 0x0C, 0x0C, 0xFF, 0xFF,

	/* @1052 'r' (6 pixels wide) */
	//
	//
	//
	//
	//
	// ######
	// ######
	// ### ##
	// ##
	// ##
	// ##
	// ##
	//
	//
	//
	//
	0xE0, 0xE0, 0xE0, 0x60, 0xE0, 0xE0,
	0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,

	/* @1064 's' (5 pixels wide) */
	//
	//
	//
	//
	//
	//  ####
	// #####
	// ##
	//  ###
	//    ##
	// #####
	// ####
	//
	//
	//
	//
	0xC0, 0xE0, 0x60, 0x60, 0x60,
	0x0C, 0x0D, 0x0D, 0x0F, 0x06,

	/* @1074 't' (6 pixels wide) */
	//
	//
	//
	//   ##
	//   ##
	// ######
	// ######
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	//
	//
	//
	//
	0x60, 0x60, 0xF8, 0xF8, 0x60, 0x60,
	0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,

	/* @1086 'u' (6 pixels wide) */
	//
	//
	//
	//
	//
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	// ##  ##
	// ######
	//  #####
	//
	//
	//
	//
	0xE0, 0xE0, 0x00, 0x00, 0xE0, 0xE0,
	0x07, 0x0F, 0x0C, 0x0C, 0x0F, 0x0F,

	/* @1098 'v' (6 pixels wide) */
	//
	//
	//
	//
	//
	// ##  ##
	// ##  ##
	//  ####
	//  ####
	//  ####
	//   ##
	//   ##
	//
	//
	//
	//
	0x60, 0xE0, 0x80, 0x80, 0xE0, 0x60,
	0x00, 0x03, 0x0F, 0x0F, 0x03, 0x00,

	/* @1110 'w' (8 pixels wide) */
	//
	//
	//
	//
	//
	// ## ## ##
	// ## ## ##
	// ## ## ##
	// ##### #
	// ### ###
	//  ## ###
	//  #   #
	//
	//
	//
	//
	0xE0, 0xE0, 0x00, 0xE0, 0xE0, 0x00, 0xE0, 0xE0,
	0x03, 0x0F, 0x07, 0x01, 0x07, 0x0E, 0x07, 0x00,

	/* @1126 'x' (7 pixels wide) */
	//
	//
	//
	//
	//
	// ##   ##
	// ### ###
	//  #####
	//   ###
	//  #####
	// ### ###
	// ##   ##
	//
	//
	//
	//
	0x60, 0xE0, 0xC0, 0x80, 0xC0, 0xE0, 0x60,
	0x0C, 0x0E, 0x07, 0x03, 0x07, 0x0E, 0x0C,

	/* @1140 'y' (7 pixels wide) */
	//
	//
	//
	//
	//
	// ##   ##
	// ##  ###
	//  ## ##
	//  ## ##
	//   ###
	//   ###
	//   ###
	//   ##
	//  ###
	//  ##
	//  ##
	0x60, 0xE0, 0x80, 0x00, 0xC0, 0xE0, 0x60,
	0x00, 0xE1, 0xFF, 0x3E, 0x0F, 0x01, 0x00,

	/* @1154 'z' (6 pixels wide) */
	//
	//
	//
	//
	//
	// ######
	// ######
	//    ##
	//   ##
	//  ##
	// ######
	// ######
	//
	//
	//
	//
	0x60, 0x60, 0x60, 0xE0, 0xE0, 0x60,
	0x0C, 0x0E, 0x0F, 0x0D, 0x0C, 0x0C,

	/* @1166 '{' (5 pixels wide) */
	//
	//
	//   ###
	//  ####
	//  ##
	//  ##
	//  ##
	//  ##
	// ##
	// ###
	//  ##
	//  ##
	//  ##
	//  ##
	//   ###
	//   ###
	0x00, 0xF8, 0xFC, 0x0C, 0x0C,
	0x03, 0x3F, 0xFE, 0xC0, 0xC0,

	/* @1176 '|' (2 pixels wide) */
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
	// ##
	// ##
	// ##
	// ##
	// ##
	// ##
	0xFC, 0xFC,
	0xFF, 0xFF,

	/* @1180 '}' (5 pixels wide) */
	//
	//
	// ###
	// ####
	//   ##
	//   ##
	//   ##
	//   ##
	//    ##
	//   ##
	//   ##
	//   ##
	//   ##
	//   ##
	// ###
	// ###
	0x0C, 0x0C, 0xFC, 0xF8, 0x00,
	0xC0, 0xC0, 0xFE, 0x3F, 0x01,
};

/* Character descriptors for Comic Sans MS 10pt */
/* { [Char width in bits], [Offset into comicSansMS_10ptCharBitmaps in bytes] } */
const TFontCharInfo comicSansMS_10ptDescriptors[] =
{
	{8, 0}, 		/*   */
	{2, 16}, 		/* ! */
	{5, 20}, 		/* " */
	{9, 30}, 		/* # */
	{8, 48}, 		/* $ */
	{11, 64}, 		/* % */
	{9, 86}, 		/* & */
	{2, 104}, 		/* ' */
	{4, 108}, 		/* ( */
	{4, 116}, 		/* ) */
	{6, 124}, 		/* * */
	{6, 136}, 		/* + */
	{2, 148}, 		/* , */
	{5, 152}, 		/* - */
	{2, 162}, 		/* . */
	{6, 166}, 		/* / */
	{6, 178}, 		/* 0 */
	{6, 190}, 		/* 1 */
	{7, 202}, 		/* 2 */
	{6, 216}, 		/* 3 */
	{7, 228}, 		/* 4 */
	{6, 242}, 		/* 5 */
	{6, 254}, 		/* 6 */
	{7, 266}, 		/* 7 */
	{6, 280}, 		/* 8 */
	{6, 292}, 		/* 9 */
	{2, 304}, 		/* : */
	{3, 308}, 		/* ; */
	{4, 314}, 		/* < */
	{5, 322}, 		/* = */
	{4, 332}, 		/* > */
	{7, 340}, 		/* ? */
	{12, 354}, 		/* @ */
	{8, 378}, 		/* A */
	{7, 394}, 		/* B */
	{8, 408}, 		/* C */
	{7, 424}, 		/* D */
	{7, 438}, 		/* E */
	{6, 452}, 		/* F */
	{9, 464}, 		/* G */
	{8, 482}, 		/* H */
	{6, 498}, 		/* I */
	{8, 510}, 		/* J */
	{7, 526}, 		/* K */
	{6, 540}, 		/* L */
	{10, 552}, 		/* M */
	{9, 572}, 		/* N */
	{10, 590}, 		/* O */
	{5, 610}, 		/* P */
	{11, 620}, 		/* Q */
	{6, 642}, 		/* R */
	{8, 654}, 		/* S */
	{8, 670}, 		/* T */
	{8, 686}, 		/* U */
	{9, 702}, 		/* V */
	{13, 720}, 		/* W */
	{9, 746}, 		/* X */
	{8, 764}, 		/* Y */
	{8, 780}, 		/* Z */
	{4, 796}, 		/* [ */
	{6, 804}, 		/* \ */
	{4, 816}, 		/* ] */
	{6, 824}, 		/* ^ */
	{9, 836}, 		/* _ */
	{3, 854}, 		/* ` */
	{6, 860}, 		/* a */
	{6, 872}, 		/* b */
	{6, 884}, 		/* c */
	{7, 896}, 		/* d */
	{6, 910}, 		/* e */
	{6, 922}, 		/* f */
	{6, 934}, 		/* g */
	{6, 946}, 		/* h */
	{2, 958}, 		/* i */
	{5, 962}, 		/* j */
	{6, 972}, 		/* k */
	{2, 984}, 		/* l */
	{8, 988}, 		/* m */
	{6, 1004}, 		/* n */
	{6, 1016}, 		/* o */
	{6, 1028}, 		/* p */
	{6, 1040}, 		/* q */
	{6, 1052}, 		/* r */
	{5, 1064}, 		/* s */
	{6, 1074}, 		/* t */
	{6, 1086}, 		/* u */
	{6, 1098}, 		/* v */
	{8, 1110}, 		/* w */
	{7, 1126}, 		/* x */
	{7, 1140}, 		/* y */
	{6, 1154}, 		/* z */
	{5, 1166}, 		/* { */
	{2, 1176}, 		/* | */
	{5, 1180}, 		/* } */
};

/* Font information for Comic Sans MS 10pt */
const TFont comicsans_10pt =
{
	16, /*  Character height */
	2, /* Space between characters */
	' ', /*  Start character */
	'}', /*  End character */
	comicSansMS_10ptDescriptors, /*  Character descriptor array */
	comicSansMS_10ptBitmaps, /*  Character bitmap array */
};


#endif // FONT_COMICSANS10PT_H_INCLUDED
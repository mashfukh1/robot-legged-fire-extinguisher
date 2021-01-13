/*
LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
LLL  LLLLL   LLL   LL      LLLL      LLLL      LLL  LLL      LLLL
LLL  LLLLLL   L   LL   LL   LL   LL   LL   LL  LLL  LLL   LL   LL
LLL  LLLLLLL     LL   LLLLLLL   LLLL   LL     LLLL  LLL  LLL   LL
LLL  LLLLLLLL   LLL   LLLLLLL   LLLL   LLLLL    LL  LLL  LLL   LL
LLL   LLLLLLL   LLLL   LL   LL   LL   LL   LL   LL  LLL   LL   LL
LLL       LLL   LLLLL      LLLL      LLLL      LLL  LLL       LLL
LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
   -----------------------------------------------------------
LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
LLL  LLL        LL       LL         LL    LLLL    LLL  LL   LLLLL
LLL  LLL  LLL   L   LLL   LLLL   LLLLL     LLL     LL  LL   LLLLL
LLL  LLL       LL   LLLL  LLLL   LLLL   L  LLL         LL   LLLLL
LLL  LLL   L   LL   LLLL  LLLL   LLLL   L   LL   L     LL   LLLLL
LLL  LLL  LL    LL   L    LLLL   LLL         L   LL    LL   LLLLL
LLL  LLL  LLL   LLL     LLLLLL   LLL  LLLL   L   LLL   LL   LLLLL
LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
*/

/* COPYRIGHT
* Copyright (C) 2016  IR-64 POLITEKNIK NEGERI JEMBER
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* AUTHOR
* Amad Waris Al H (warisafidz@gmail.com)
* Nashrul Bahri ()
* Muhammad Masfukh ()
*/

/* PIN Analog ----> */
// Software Serial

/* Protokol Komunikas ----> */
#define ACK       '0'
#define GET       'X'
#define CLR       0
#define SYNC      '3'

// Navigasi

/* NOTE
 *  TURN 90* use 14n
 *  Standby to SCAN use 8n
 *  TURN BACK use 32n
 */
#define TURNRSLOW 'Y'
#define TURNLSLOW 'Z'
#define TURNR     'B'
#define TURNL     'C'
#define BACK      'T'
#define SLIDR	  'U'
#define SLIDL	  'S'
#define SCAN      'E'
#define SSCAN     'F'
#define WALK      'G'
#define WALK2 	  'V'
#define WALK3 	  'A'
#define CLIMB	  'W'
#define GETUP     'H'

#define SSPEED    'I'
#define S1        1
#define S2        2
#define S3        3
#define S4        4

#define TURNRA    'J'
#define TURNLA    'K'
#define STAND     'L'
#define FIREON    'M'
#define FIREOFF   'N'
#define EXMODE    'O'
#define TUNE      'P'
#define AUTO_PILOT   'Q'
#define MANUAL  'R'
#define LOOK_AT   '+'

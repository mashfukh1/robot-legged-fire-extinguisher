
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
#include <Arduino.h>

// STANDARD PROTOCOL
#define CLR 0
#define GET 'X'

// TPA81 Asset
#define TPA_ADDRESS 0x68
#define TPA_SOFTREG 0x00
#define TPA_AMBIANT 0x01

class LycosidComputeClass{
public:
  void begin(long baud);

  bool tryPost(const char cmd);
  float readPing(char pingPIN);
  void initCMPS(void);
  float readCMPS(void);
  int tpaAmbi(void);
  int tpaPixel(uint8_t pixel);
  bool tryPost(const char cmd, const int param);
  bool tryPost(const char cmd, const float param);
  void keyPost(const char cmd);
  void keyPost(const char cmd, int param);
  void keyPost(const char cmd, float param);
};
extern LycosidComputeClass Compute;


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
#include <Arduino.h>

//STANDARD PROTOCOL
#define GET 'X'

#define SRX   A0  // SoftSerial RX
#define STX   A1  // SoftSerial TX

/* -----| Protokol Komunikasi */

/* Konstanta Nilai --- > */
// ID LEG pada Servo
#define ILEG1_A  1
#define ILEG1_B  2
#define ILEG1_C  3
#define ILEG2_A  7
#define ILEG2_B  8
#define ILEG2_C  9
#define ILEG3_A  13
#define ILEG3_B  14
#define ILEG3_C  15
#define ILEG4_A  16
#define ILEG4_B  17
#define ILEG4_C  18
#define ILEG5_A  10
#define ILEG5_B  11
#define ILEG5_C  12
#define ILEG6_A  4
#define ILEG6_B  5
#define ILEG6_C  6

#define setPoint  15    // Jarak robot dengan dinding
#define C_TIMER0    15    // TImer delay sesaat
#define C_TIMER1   300     //85    // Timer delay penuh
#define C_SPEED0   100   // Speed sesaat
#define C_SPEED1    100   // Speed penuh

#define SHIGH     145 // Define robot high
#define RHIGH     20  // Define Raise Scale
/* -----| Konstanta Nilai */

/* Macro Ekspansi ---> */
#define MAP(x)   (int)map(x, 0, 300, 0, 1023)
/* -----| Macro Ekspansi */

/* Treshold ----> */
// Batas sudut kaki
#define MAXRange  MAP(20) // Limit rentang maksimal sebesar 20'
#define SPRange   MAP(20) // Setpoint pergerakan
/* ----| Treshold*/

/* VARIABEL GLOBAL ----> */

// langkah kaki tiap sendi step 1 -> 2 -> 3 -> 4 -> 1

/* ----| VARIABEL GLOBAL */

class LycosidControlClass {
private:
  int
    speed0 = C_SPEED0,
    speed1 = C_SPEED1,
    timer0 = C_TIMER0,
    timer1 = C_TIMER1,
    timeout = 0;
  bool dataIn = false;

      int
      // 0: Depan
      SLEG1_A [4] = {MAP(115), MAP(135), MAP(135), MAP(115)},
      SLEG1_B [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},
      SLEG1_C [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},

      SLEG2_A [4] = {MAP(160), MAP(140), MAP(140), MAP(160)},
      SLEG2_B [4] = {MAP(SHIGH-2), MAP(SHIGH-2), MAP(SHIGH-2), MAP((SHIGH-RHIGH)-2)},
      SLEG2_C [4] = {MAP(SHIGH-2), MAP(SHIGH-2), MAP(SHIGH-2), MAP((SHIGH-RHIGH)-2)},

      // 1: Belakang
      SLEG3_A [4] = {MAP(165), MAP(185), MAP(185), MAP(165)},
      SLEG3_B [4] = {MAP(SHIGH-4), MAP((SHIGH-RHIGH)-4), MAP(SHIGH-4), MAP(SHIGH-4)},
      SLEG3_C [4] = {MAP(SHIGH-4), MAP((SHIGH-RHIGH)-4), MAP(SHIGH-4), MAP(SHIGH-4)},

      // 0: Depan
      SLEG4_A [4] = {MAP(115), MAP(135), MAP(135), MAP(115)},
      SLEG4_B [4] = {MAP(SHIGH-4), MAP(SHIGH-4), MAP(SHIGH-4), MAP((SHIGH-RHIGH)-4)},
      SLEG4_C [4] = {MAP(SHIGH-4), MAP(SHIGH-4), MAP(SHIGH-4), MAP((SHIGH-RHIGH)-4)},

      SLEG5_A [4] = {MAP(160), MAP(140), MAP(140), MAP(160)},
      SLEG5_B [4] = {MAP(SHIGH-2), MAP((SHIGH-RHIGH)-2), MAP(SHIGH-2), MAP(SHIGH-2)},
      SLEG5_C [4] = {MAP(SHIGH-2), MAP((SHIGH-RHIGH)-2), MAP(SHIGH-2), MAP(SHIGH-2)},

      // 1: Belakang
      SLEG6_A [4] = {MAP(165), MAP(185), MAP(185), MAP(165)},
      SLEG6_B [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},
      SLEG6_C [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)};

/*int
SLEG1_A [4] = {MAP(115), MAP(135), MAP(135), MAP(115)},
SLEG1_B [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},
SLEG1_C [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},

SLEG2_A [4] = {MAP(160), MAP(140), MAP(140), MAP(160)},
SLEG2_B [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},
SLEG2_C [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},

SLEG3_A [4] = {MAP(165), MAP(185), MAP(185), MAP(165)},
SLEG3_B [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},
SLEG3_C [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},

SLEG4_A [4] = {MAP(115), MAP(135), MAP(135), MAP(115)},
SLEG4_B [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},
SLEG4_C [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},

SLEG5_A [4] = {MAP(160), MAP(140), MAP(140), MAP(160)},
SLEG5_B [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},
SLEG5_C [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},

SLEG6_A [4] = {MAP(165), MAP(185), MAP(185), MAP(165)},
SLEG6_B [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},
SLEG6_C [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)};
/*
  int
  SLEG1_A [4] = {MAP(115), MAP(135), MAP(135), MAP(115)},
  SLEG1_B [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},
  SLEG1_C [4] = {MAP(SHIGH), MAP(SHIGH-RHIGH), MAP(SHIGH), MAP(SHIGH)},

  SLEG2_A [4] = {MAP(160), MAP(140), MAP(140), MAP(160)},
  SLEG2_B [4] = {MAP(SHIGH - 10), MAP(SHIGH - 10), MAP(SHIGH - 10), MAP((SHIGH-RHIGH)) - 10},
  SLEG2_C [4] = {MAP(SHIGH - 10), MAP(SHIGH - 10), MAP(SHIGH - 10), MAP((SHIGH-RHIGH)) - 10},

  SLEG3_A [4] = {MAP(165), MAP(185), MAP(185), MAP(165)},
  SLEG3_B [4] = {MAP(SHIGH - 20), MAP((SHIGH-RHIGH) - 20), MAP(SHIGH - 20), MAP(SHIGH - 20)},
  SLEG3_C [4] = {MAP(SHIGH - 20), MAP((SHIGH-RHIGH) - 20), MAP(SHIGH - 20), MAP(SHIGH - 20)},

  SLEG4_A [4] = {MAP(115), MAP(135), MAP(135), MAP(115)},
  SLEG4_B [4] = {MAP(SHIGH - 20), MAP(SHIGH - 20), MAP(SHIGH - 20), MAP((SHIGH-RHIGH) - 20)},
  SLEG4_C [4] = {MAP(SHIGH - 20), MAP(SHIGH - 20), MAP(SHIGH - 20), MAP((SHIGH-RHIGH) - 20)},

  SLEG5_A [4] = {MAP(160), MAP(140), MAP(140), MAP(160)},
  SLEG5_B [4] = {MAP(SHIGH - 10), MAP((SHIGH-RHIGH) - 10), MAP(SHIGH - 10), MAP(SHIGH - 10)},
  SLEG5_C [4] = {MAP(SHIGH - 10), MAP((SHIGH-RHIGH) - 10), MAP(SHIGH - 10), MAP(SHIGH - 10)},

  SLEG6_A [4] = {MAP(165), MAP(185), MAP(185), MAP(165)},
  SLEG6_B [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)},
  SLEG6_C [4] = {MAP(SHIGH), MAP(SHIGH), MAP(SHIGH), MAP(SHIGH-RHIGH)};
*/
  unsigned char steps;  // penyimpanan step
public:
  void begin(long baud);
	void begin(long sbaud, long baud, unsigned char dirPin);
  void tune(int tuner);
	void walk(float lRange, float rRange);
  void walk2(float lRange, float rRange);
  void walk3(float lRange, float rRange);
  void turnLslow(unsigned char nturn);
  void turnRslow(unsigned char nturn);
  void back(float lRange, float rRange);
  void slidR(unsigned char nturn);
  void slidL(unsigned char nturn);
	void turnR(unsigned char nturn);
	void turnL(unsigned char nturn);
  void moveL(unsigned char nmove);
  void moveR(unsigned char nmove);
	void getUP(void);
	void climb(void);
	void getData(char * resCMD, float * resParam);
  void viewData(char * resCMD, float * resParam);
  void look();
  void setSpeed(unsigned int speed_0, unsigned int speed_1, unsigned int timer_0, unsigned int timer_1);
  void turnLa(unsigned char nturn);
  void turnRa(unsigned char nturn);
  void stand(void);
  void exting(int range);

};
extern LycosidControlClass Control;

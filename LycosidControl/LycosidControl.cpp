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

#include "LycosidControl.h"
#include <SoftwareSerial.h>
#include <DynamixelSerial.h>

SoftwareSerial MegaNo(SRX, STX);	// SoftSerial Mega -> Nano

void LycosidControlClass::begin(long sbaud){
  MegaNo.begin(sbaud);
  timeout = 0;
  dataIn = true;
}

void LycosidControlClass::begin(long sbaud, long baud, unsigned char dirPin){
	MegaNo.begin(sbaud);		// Mulai komunikasi Mega -> Nano
	Dynamixel.begin(baud, dirPin);	// Mulai komunikasi Serial dengan Dynamixel
  dataIn = true;
  timeout = 0;
	steps = 0;
}

void LycosidControlClass::tune(int tuner){
  for (int i = 0; i < 4; i++){
    SLEG1_B[i] = SLEG1_C [i] = SLEG1_B[i] - tuner;
    SLEG2_B[i] = SLEG2_C [i] = SLEG2_B[i] - tuner;
    SLEG3_B[i] = SLEG3_C [i] = SLEG3_B[i] - tuner;
    SLEG4_B[i] = SLEG4_C [i] = SLEG4_B[i] - tuner;
    SLEG5_B[i] = SLEG5_C [i] = SLEG5_B[i] - tuner;
    SLEG6_B[i] = SLEG6_C [i] = SLEG6_B[i] - tuner;
  }
  Dynamixel.move(ILEG1_B, SLEG1_B[0]);
  Dynamixel.move(ILEG1_C, SLEG1_C[0]);
  Dynamixel.move(ILEG2_B, SLEG2_B[0]);
  Dynamixel.move(ILEG2_C, SLEG2_C[0]);
  Dynamixel.move(ILEG3_B, SLEG3_B[0]);
  Dynamixel.move(ILEG3_C, SLEG3_C[0]);
  Dynamixel.move(ILEG4_B, SLEG4_B[0]);
  Dynamixel.move(ILEG4_C, SLEG4_C[0]);
  Dynamixel.move(ILEG5_B, SLEG5_B[0]);
  Dynamixel.move(ILEG5_C, SLEG5_C[0]);
  Dynamixel.move(ILEG6_B, SLEG6_B[0]);
  Dynamixel.move(ILEG6_C, SLEG6_C[0]);
}
void LycosidControlClass::setSpeed(unsigned int speed_0, unsigned int speed_1, unsigned int timer_0, unsigned int timer_1){
  speed0 = speed_0;
  speed1 = speed_1;
  timer0 = timer_0;
  timer1 = timer_1;
}

void LycosidControlClass::getData(char * resCMD, float * resParam){
  char buff[20];
unsigned int timeoutX = 0;
MegaNo.print(GET);
  while(!(MegaNo.available())){
    delay(1);
    timeoutX++;
    if (timeoutX > 5000){
      Serial.println("Connection Timeout");
      *resCMD = 0;
      *resParam = 0;
      timeoutX = 0;
      return;
    }
  }

int i = 0;
  *resCMD = MegaNo.read();

  // FOR DEBUGIN ONLY
  //delay(100);
  while (MegaNo.available()){
    buff[i++] = MegaNo.read();
  }
  buff[i] = '\0';

  *resParam = atof(buff);
}

void LycosidControlClass::viewData(char * resCMD, float * resParam){
	char buff[20];

  if (timeout > 5000 || dataIn == true){
	   MegaNo.print(GET);
     timeout = 0;
     delay(5);
   }

  if(!(MegaNo.available())){
    *resCMD = 0;
    *resParam = 0;
    dataIn = false;
    delay(1);
    timeout++;
    return;
  }

  timeout = 0;
  dataIn = true;

	int i = 0;
    *resCMD = MegaNo.read();

    // FOR DEBUGIN ONLY
    //delay(100);
    while (MegaNo.available()){
		  buff[i++] = MegaNo.read();
    }
    buff[i] = '\0';

    *resParam = atof(buff);
}

void LycosidControlClass::look(){
  Dynamixel.moveSpeed(1, MAP(200),400); //depan
  Dynamixel.moveSpeed(7, MAP(170),400); //tengah
  Dynamixel.moveSpeed(13, MAP(150),400); //belakang
  
  Dynamixel.moveSpeed(2,MAP(170),400);
  Dynamixel.moveSpeed(3,MAP(170),400);
  Dynamixel.moveSpeed(5,MAP(135),400);
  Dynamixel.moveSpeed(6,MAP(135),400);
  Dynamixel.moveSpeed(8,MAP(165),400);
  Dynamixel.moveSpeed(9,MAP(165),400);
  Dynamixel.moveSpeed(11,MAP(135),400);
  Dynamixel.moveSpeed(12,MAP(135),400);
  Dynamixel.moveSpeed(14,MAP(140),400);
  Dynamixel.moveSpeed(15,MAP(140),400);
  Dynamixel.moveSpeed(17,MAP(160),400);
  Dynamixel.moveSpeed(18,MAP(160),400);
  
  Dynamixel.moveSpeed(4, MAP(140),400); //depan
  Dynamixel.moveSpeed(10, MAP(170),400); //tengah
  Dynamixel.moveSpeed(16, MAP(190),400); //belakang                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
  delay(200);
  
  Dynamixel.moveSpeed(1, MAP(160),400); //depan
  Dynamixel.moveSpeed(7, MAP(130),400); //tengah
  Dynamixel.moveSpeed(13, MAP(110),400); //belakang
  
  Dynamixel.moveSpeed(2,MAP(170),400);
  Dynamixel.moveSpeed(3,MAP(170),400);
  Dynamixel.moveSpeed(5,MAP(135),400);
  Dynamixel.moveSpeed(6,MAP(135),400);
  Dynamixel.moveSpeed(8,MAP(165),400);
  Dynamixel.moveSpeed(9,MAP(165),400);
  Dynamixel.moveSpeed(11,MAP(135),400);
  Dynamixel.moveSpeed(12,MAP(135),400);
  Dynamixel.moveSpeed(14,MAP(140),400);
  Dynamixel.moveSpeed(15,MAP(140),400);
  Dynamixel.moveSpeed(17,MAP(160),400);
  Dynamixel.moveSpeed(18,MAP(160),400);
  
  Dynamixel.moveSpeed(4, MAP(100),200); //depan
  Dynamixel.moveSpeed(10, MAP(130),200); //tengah
  Dynamixel.moveSpeed(16, MAP(150),200); //belakang                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
  delay(200);
}

void LycosidControlClass::moveR(unsigned char nmove){
  Dynamixel.move(ILEG1_B, SLEG1_B[1] - RHIGH);
  Dynamixel.move(ILEG1_C, SLEG1_C[1] - RHIGH);
  Dynamixel.move(ILEG3_B, SLEG3_B[1] - RHIGH);
  Dynamixel.move(ILEG3_B, SLEG3_B[1] - RHIGH);
  Dynamixel.move(ILEG5_B, SLEG5_B[1] - RHIGH);
  Dynamixel.move(ILEG5_B, SLEG5_B[1] - RHIGH);

  delay(timer0);

  Dynamixel.move(ILEG1_A, SLEG1_A[1] - MAP(35));
  Dynamixel.move(ILEG5_A, MAP(150));
  Dynamixel.move(ILEG3_A, SLEG3_A[0] + MAP(35));

  delay(timer1);

  Dynamixel.move(ILEG1_B, SLEG1_B[0]);
  Dynamixel.move(ILEG1_C, SLEG1_C[0]);
  Dynamixel.move(ILEG3_B, SLEG3_B[0]);
  Dynamixel.move(ILEG3_B, SLEG3_B[0]);
  Dynamixel.move(ILEG5_B, SLEG5_B[0]);
  Dynamixel.move(ILEG5_B, SLEG5_B[0]);

  delay(timer1);

  Dynamixel.move(ILEG1_A, SLEG1_A[0] - MAP(35));
  Dynamixel.move(ILEG5_A, MAP(150));
  Dynamixel.move(ILEG3_A, SLEG3_A[1] + MAP(35));
  delay(timer1);

}
void LycosidControlClass::stand(void){
    delay(1000);
    Dynamixel.moveSpeed(1,MAP(180),200);
    Dynamixel.moveSpeed(2,MAP(170),200);
    Dynamixel.moveSpeed(3,MAP(170),200);
    Dynamixel.moveSpeed(4,MAP(120),200);
    Dynamixel.moveSpeed(5,MAP(135),200);
    
    Dynamixel.moveSpeed(6,MAP(135),200);
    Dynamixel.moveSpeed(7,MAP(150),200);
    Dynamixel.moveSpeed(8,MAP(170),200);
    Dynamixel.moveSpeed(9,MAP(170),200);
    Dynamixel.moveSpeed(10,MAP(150),200);

    Dynamixel.moveSpeed(11,MAP(135),200);
    Dynamixel.moveSpeed(12,MAP(135),200);
    Dynamixel.moveSpeed(13,MAP(120),200);
    Dynamixel.moveSpeed(14,MAP(170),200);
    Dynamixel.moveSpeed(15,MAP(170),200);
    Dynamixel.moveSpeed(16,MAP(180),200);
    Dynamixel.moveSpeed(17,MAP(135),200);
    Dynamixel.moveSpeed(18,MAP(135),200);
}

void LycosidControlClass::getUP(void){
	Dynamixel.moveSpeed(16,MAP(180),200);
	 Dynamixel.moveSpeed(13,MAP(120),200);
	Dynamixel.moveSpeed(10,MAP(150),200);
	Dynamixel.moveSpeed(7,MAP(150),200);
	Dynamixel.moveSpeed(1,MAP(180),200);
	Dynamixel.moveSpeed(4,MAP(120),200);
    Dynamixel.moveSpeed(5,MAP(125),500);   //nai7
    Dynamixel.moveSpeed(6,MAP(125),500);

    Dynamixel.moveSpeed(8,MAP(175),500);  //naik
    Dynamixel.moveSpeed(9,MAP(175),500);

    Dynamixel.moveSpeed(17,MAP(125),500);  //naik
    Dynamixel.moveSpeed(18,MAP(125),500);
    delay(200);
    Dynamixel.moveSpeed(8,MAP(165),500);
    Dynamixel.moveSpeed(9,MAP(165),500);
    
    Dynamixel.moveSpeed(17,MAP(135),500);
    Dynamixel.moveSpeed(18,MAP(135),500);
    
    Dynamixel.moveSpeed(5,MAP(135),500); //turun
    Dynamixel.moveSpeed(6,MAP(135),500);
    delay(200);
    Dynamixel.moveSpeed(2,MAP(180),500);
    Dynamixel.moveSpeed(3,MAP(180),500); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),500); //angkat
    Dynamixel.moveSpeed(12,MAP(125),500);

    Dynamixel.moveSpeed(14,MAP(180),500);  //angkat
    Dynamixel.moveSpeed(15,MAP(180),500);
    delay(200);
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(135),500);
    Dynamixel.moveSpeed(12,MAP(135),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(200);	
}

void LycosidControlClass::turnR(unsigned char nturn){
       while(nturn-- > 0){
    switch (steps){
      case 0:
    Dynamixel.moveSpeed(5,MAP(125),500);   //nai7
    Dynamixel.moveSpeed(6,MAP(125),500);

    Dynamixel.moveSpeed(8,MAP(175),500);  //naik
    Dynamixel.moveSpeed(9,MAP(175),500);

    Dynamixel.moveSpeed(17,MAP(125),500);  //naik
    Dynamixel.moveSpeed(18,MAP(125),500);
    delay(20);
      break;
      case 1:
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(135),500);
    Dynamixel.moveSpeed(12,MAP(135),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(8);


    Dynamixel.moveSpeed(4,MAP(110),500); //geser 
    Dynamixel.moveSpeed(16,MAP(165),500);
    Dynamixel.moveSpeed(7,MAP(145),500);  

    Dynamixel.moveSpeed(1,MAP(195),500);
    Dynamixel.moveSpeed(13,MAP(140),500); //geser 
    Dynamixel.moveSpeed(10,MAP(160),500);//geser
    delay(100);
      break;

      case 2:
         /* 2 ---> 3 */
    Dynamixel.moveSpeed(2,MAP(180),500);
    Dynamixel.moveSpeed(3,MAP(180),500); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),500); //angkat
    Dynamixel.moveSpeed(12,MAP(125),500);

    Dynamixel.moveSpeed(14,MAP(180),500);  //angkat
    Dynamixel.moveSpeed(15,MAP(180),500);
    delay(20);
      break;
      case 3:
    Dynamixel.moveSpeed(8,MAP(165),500);
    Dynamixel.moveSpeed(9,MAP(165),500);
    
    Dynamixel.moveSpeed(17,MAP(135),500);
    Dynamixel.moveSpeed(18,MAP(135),500);

    Dynamixel.moveSpeed(5,MAP(135),500); //turun
    Dynamixel.moveSpeed(6,MAP(135),500);
    delay(9);

    
        Dynamixel.moveSpeed(4,MAP(130),500); //geser
    Dynamixel.moveSpeed(16,MAP(185),500);
    Dynamixel.moveSpeed(7,MAP(165),500); 
    
    Dynamixel.moveSpeed(1,MAP(175),500);
    Dynamixel.moveSpeed(10,MAP(140),500);
    Dynamixel.moveSpeed(13,MAP(125),500);
    delay(100);
      break;
    }
    steps++;
    if (steps > 3)
      steps = 0;
  }

}

void LycosidControlClass::turnLa(unsigned char nturn){
  while(nturn-- > 0){
   switch (steps){
    case 0:
      /* 4 ---> 1 */
    Dynamixel.moveSpeed(5,MAP(125),550);   //naik
    Dynamixel.moveSpeed(6,MAP(125),550);

    Dynamixel.moveSpeed(8,MAP(175),550);  //naik
    Dynamixel.moveSpeed(9,MAP(175),550);

    Dynamixel.moveSpeed(17,MAP(125),550);  //naik
    Dynamixel.moveSpeed(18,MAP(125),550);
    delay(20);
      break;

      case 1:
      /* 1 ---> 2 */
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(130),500);
    Dynamixel.moveSpeed(12,MAP(130),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(7);

    Dynamixel.moveSpeed(4,MAP(110),700); //geser
    Dynamixel.moveSpeed(7,MAP(155),700);  
    Dynamixel.moveSpeed(16,MAP(165),700);

    Dynamixel.moveSpeed(1,MAP(170),700);
    Dynamixel.moveSpeed(10,MAP(160),700);//geser
    Dynamixel.moveSpeed(13,MAP(135),700);
    delay(70);
    break;

    case 2:
    Dynamixel.moveSpeed(2,MAP(175),550);
    Dynamixel.moveSpeed(3,MAP(175),550); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),550); //angkat
    Dynamixel.moveSpeed(12,MAP(125),550);

    Dynamixel.moveSpeed(14,MAP(175),550);  //angkat
    Dynamixel.moveSpeed(15,MAP(175),550);
    delay(20);
    break;

    case 3:
    Dynamixel.moveSpeed(8,MAP(170),500);
    Dynamixel.moveSpeed(9,MAP(170),500);
    
    Dynamixel.moveSpeed(18,MAP(130),500);
    Dynamixel.moveSpeed(17,MAP(130),500);
    
    Dynamixel.moveSpeed(5,MAP(130),500); //turun
    Dynamixel.moveSpeed(6,MAP(130),500);
    delay(7);
    
    Dynamixel.moveSpeed(4,MAP(135),700); //geser
    Dynamixel.moveSpeed(7,MAP(145),700);  
    Dynamixel.moveSpeed(16,MAP(195),700);

    Dynamixel.moveSpeed(1,MAP(175),700);
    Dynamixel.moveSpeed(13,MAP(140),700); //geser 
    Dynamixel.moveSpeed(10,MAP(140),700);
    delay(70);
    break;
  }
  steps++;
  if (steps == 4)
    steps = 0;
  }
}

void LycosidControlClass::turnL(unsigned char nturn){
     while(nturn-- > 0){
    switch (steps){
      case 0:
    Dynamixel.moveSpeed(5,MAP(125),500);   //nai7
    Dynamixel.moveSpeed(6,MAP(125),500);

    Dynamixel.moveSpeed(8,MAP(175),500);  //naik
    Dynamixel.moveSpeed(9,MAP(175),500);

    Dynamixel.moveSpeed(17,MAP(125),500);  //naik
    Dynamixel.moveSpeed(18,MAP(125),500);
    delay(20);
      break;
      case 1:
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(135),500);
    Dynamixel.moveSpeed(12,MAP(135),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(8);


    Dynamixel.moveSpeed(4,MAP(130),500); //geser
    Dynamixel.moveSpeed(16,MAP(185),500);
    Dynamixel.moveSpeed(7,MAP(165),500); 
    
    Dynamixel.moveSpeed(1,MAP(175),500);
    Dynamixel.moveSpeed(10,MAP(140),500);
    Dynamixel.moveSpeed(13,MAP(125),500);
    delay(100);
      break;

      case 2:
         /* 2 ---> 3 */
    Dynamixel.moveSpeed(2,MAP(180),500);
    Dynamixel.moveSpeed(3,MAP(180),500); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),500); //angkat
    Dynamixel.moveSpeed(12,MAP(125),500);

    Dynamixel.moveSpeed(14,MAP(180),500);  //angkat
    Dynamixel.moveSpeed(15,MAP(180),500);
    delay(20);
      break;
      case 3:
    Dynamixel.moveSpeed(8,MAP(165),500);
    Dynamixel.moveSpeed(9,MAP(165),500);
    
    Dynamixel.moveSpeed(17,MAP(135),500);
    Dynamixel.moveSpeed(18,MAP(135),500);

    Dynamixel.moveSpeed(5,MAP(135),500); //turun
    Dynamixel.moveSpeed(6,MAP(135),500);
    delay(9);
    Dynamixel.moveSpeed(4,MAP(110),500); //geser 
    Dynamixel.moveSpeed(16,MAP(165),500);
    Dynamixel.moveSpeed(7,MAP(145),500);  

    Dynamixel.moveSpeed(1,MAP(195),500);
    Dynamixel.moveSpeed(13,MAP(140),500); //geser 
    Dynamixel.moveSpeed(10,MAP(160),500);//geser
    delay(100);
      break;
    }
    steps++;
    if (steps > 3)
      steps = 0;
  }

}
/*
void LycosidControlClass::turnR(unsigned char nturn){
	while(nturn-- > 0){
    switch (steps){
      case 0:

        Dynamixel.moveSpeed(ILEG1_B, SLEG1_B[0], speed1);
        Dynamixel.moveSpeed(ILEG1_C, SLEG1_C[0], speed1);
        Dynamixel.moveSpeed(ILEG2_B, SLEG2_B[0], speed1);
        Dynamixel.moveSpeed(ILEG2_C, SLEG2_C[0], speed1);
        Dynamixel.moveSpeed(ILEG3_B, SLEG3_B[0], speed1);
        Dynamixel.moveSpeed(ILEG3_C, SLEG3_C[0], speed1);
        Dynamixel.moveSpeed(ILEG4_B, SLEG4_B[0], speed1);
        Dynamixel.moveSpeed(ILEG4_C, SLEG4_C[0], speed1);
        Dynamixel.moveSpeed(ILEG5_B, SLEG5_B[0], speed1);
        Dynamixel.moveSpeed(ILEG5_C, SLEG5_C[0], speed0);
        Dynamixel.moveSpeed(ILEG6_B, SLEG6_B[0], speed1);
        Dynamixel.moveSpeed(ILEG6_C, SLEG6_C[0], speed1);

        delay(timer0);
        Dynamixel.moveSpeed(ILEG6_A, SLEG6_A[0], speed0);
        Dynamixel.moveSpeed(ILEG5_A, SLEG5_A[0], speed0);
        Dynamixel.moveSpeed(ILEG4_A, SLEG4_A[0], speed0);
        Dynamixel.moveSpeed(ILEG3_A, SLEG3_A[1], speed0);
        Dynamixel.moveSpeed(ILEG2_A, SLEG2_A[1], speed0);
        Dynamixel.moveSpeed(ILEG1_A, SLEG1_A[1], speed0);
        delay(timer1);
      break;

      case 1:

        Dynamixel.moveSpeed(ILEG1_C, SLEG1_C[1], speed1);
        Dynamixel.moveSpeed(ILEG1_B, SLEG1_B[1], speed1);

        Dynamixel.moveSpeed(ILEG2_C, SLEG2_C[1], speed1);
        Dynamixel.moveSpeed(ILEG2_B, SLEG2_B[1], speed1);

        Dynamixel.moveSpeed(ILEG3_C, SLEG3_C[1], speed1);
        Dynamixel.moveSpeed(ILEG3_B, SLEG3_B[1], speed1);

        Dynamixel.moveSpeed(ILEG4_C, SLEG4_C[1], speed1);
        Dynamixel.moveSpeed(ILEG4_B, SLEG4_B[1], speed1);

        Dynamixel.moveSpeed(ILEG5_C, SLEG5_C[1], speed1);
        Dynamixel.moveSpeed(ILEG5_B, SLEG5_B[1], speed1);

        Dynamixel.moveSpeed(ILEG6_C, SLEG6_C[1], speed1);
        Dynamixel.moveSpeed(ILEG6_B, SLEG6_B[1], speed1);

        delay(timer0);
        Dynamixel.moveSpeed(ILEG1_A, SLEG1_A[0], speed0);
        Dynamixel.moveSpeed(ILEG2_A, SLEG2_A[0], speed0);
        Dynamixel.moveSpeed(ILEG3_A, SLEG3_A[0], speed0);
        Dynamixel.moveSpeed(ILEG4_A, SLEG4_A[1], speed0);
        Dynamixel.moveSpeed(ILEG5_A, SLEG5_A[1], speed0);
        Dynamixel.moveSpeed(ILEG6_A, SLEG6_A[1], speed0);
        delay(timer1);
      break;

      case 2:
        Dynamixel.moveSpeed(ILEG1_A, SLEG1_A[3], speed0);
        Dynamixel.moveSpeed(ILEG2_A, SLEG2_A[3], speed0);
        Dynamixel.moveSpeed(ILEG3_A, SLEG3_A[3], speed0);
        Dynamixel.moveSpeed(ILEG4_A, SLEG4_A[2], speed0);
        Dynamixel.moveSpeed(ILEG5_A, SLEG5_A[2], speed0);
        Dynamixel.moveSpeed(ILEG6_A, SLEG6_A[2], speed0);

        delay(timer0);
        Dynamixel.moveSpeed(ILEG1_C, SLEG1_C[2], speed1);
        Dynamixel.moveSpeed(ILEG1_B, SLEG1_B[2], speed1);

        Dynamixel.moveSpeed(ILEG2_C, SLEG2_C[2], speed1);
        Dynamixel.moveSpeed(ILEG2_B, SLEG2_B[1], speed1);

        Dynamixel.moveSpeed(ILEG3_C, SLEG3_C[2], speed1);
        Dynamixel.moveSpeed(ILEG3_B, SLEG3_B[2], speed1);

        Dynamixel.moveSpeed(ILEG4_C, SLEG4_C[2], speed1);
        Dynamixel.moveSpeed(ILEG4_B, SLEG4_B[2], speed1);

        Dynamixel.moveSpeed(ILEG5_C, SLEG5_C[2], speed1);
        Dynamixel.moveSpeed(ILEG5_B, SLEG5_B[2], speed1);

        Dynamixel.moveSpeed(ILEG6_C, SLEG6_C[2], speed1);
        Dynamixel.moveSpeed(ILEG6_B, SLEG6_B[2], speed1);

        delay(timer1);
      break;

      case 3:

        Dynamixel.moveSpeed(ILEG1_A, SLEG1_A[2], speed0);
        Dynamixel.moveSpeed(ILEG1_B, SLEG1_B[3], speed1);
        Dynamixel.moveSpeed(ILEG1_C, SLEG1_C[3], speed1);

        Dynamixel.moveSpeed(ILEG2_A, SLEG2_A[2], speed0);
        Dynamixel.moveSpeed(ILEG2_B, SLEG2_B[3], speed1);
        Dynamixel.moveSpeed(ILEG2_C, SLEG2_C[3], speed1);

        Dynamixel.moveSpeed(ILEG3_A, SLEG3_A[2], speed0);
        Dynamixel.moveSpeed(ILEG3_B, SLEG3_B[3], speed1);
        Dynamixel.moveSpeed(ILEG3_C, SLEG3_C[3], speed1);

        Dynamixel.moveSpeed(ILEG4_A, SLEG4_A[3], speed0);
        Dynamixel.moveSpeed(ILEG4_B, SLEG4_B[3], speed1);
        Dynamixel.moveSpeed(ILEG4_C, SLEG4_C[3], speed1);

        Dynamixel.moveSpeed(ILEG5_A, SLEG5_A[3], speed0);
        Dynamixel.moveSpeed(ILEG5_B, SLEG5_B[3], speed1);
        Dynamixel.moveSpeed(ILEG5_C, SLEG5_C[3], speed1);

        Dynamixel.moveSpeed(ILEG6_A, SLEG6_A[3], speed0);
        Dynamixel.moveSpeed(ILEG6_B, SLEG6_B[3], speed1);
        Dynamixel.moveSpeed(ILEG6_C, SLEG6_C[3], speed1);
        delay(timer1);
      break;
    }
    steps++;
    if (steps > 3)
      steps = 0;
  }

}
*/

void LycosidControlClass::turnRa(unsigned char nturn){
  while(nturn-- > 0){
   switch (steps){
    case 0:
      /* 4 ---> 1 */
    Dynamixel.moveSpeed(5,MAP(125),550);   //naik
    Dynamixel.moveSpeed(6,MAP(125),550);

    Dynamixel.moveSpeed(8,MAP(175),550);  //naik
    Dynamixel.moveSpeed(9,MAP(175),550);

    Dynamixel.moveSpeed(17,MAP(125),550);  //naik
    Dynamixel.moveSpeed(18,MAP(125),550);
    delay(20);
      break;

      case 1:
      /* 1 ---> 2 */
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(130),500);
    Dynamixel.moveSpeed(12,MAP(130),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(7);

    Dynamixel.moveSpeed(4,MAP(130),700); //geser
    Dynamixel.moveSpeed(7,MAP(165),700);  //o
    Dynamixel.moveSpeed(16,MAP(165),700);

    Dynamixel.moveSpeed(1,MAP(170),700); //o
    Dynamixel.moveSpeed(10,MAP(140),700);//geser
    Dynamixel.moveSpeed(13,MAP(115),700); //o
    delay(70);
    break;

    case 2:
    Dynamixel.moveSpeed(2,MAP(175),550);
    Dynamixel.moveSpeed(3,MAP(175),550); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),550); //angkat
    Dynamixel.moveSpeed(12,MAP(125),550);

    Dynamixel.moveSpeed(14,MAP(175),550);  //angkat
    Dynamixel.moveSpeed(15,MAP(175),550);
    delay(20);
    break;

    case 3:
    Dynamixel.moveSpeed(8,MAP(170),500);
    Dynamixel.moveSpeed(9,MAP(170),500);
    
    Dynamixel.moveSpeed(18,MAP(130),500);
    Dynamixel.moveSpeed(17,MAP(130),500);
    
    Dynamixel.moveSpeed(5,MAP(130),500); //turun
    Dynamixel.moveSpeed(6,MAP(130),500);
    delay(7);
    
    Dynamixel.moveSpeed(4,MAP(135),700); //geser
    Dynamixel.moveSpeed(7,MAP(145),700);  
    Dynamixel.moveSpeed(16,MAP(175),700);

    Dynamixel.moveSpeed(1,MAP(195),700);
    Dynamixel.moveSpeed(13,MAP(140),700); //geser 
    Dynamixel.moveSpeed(10,MAP(140),700);
    delay(70);
    break;
  }
  steps++;
  if (steps == 4)
    steps = 0;
  }
}
void LycosidControlClass::turnRslow(unsigned char nturn){
       while(nturn-- > 0){
    switch (steps){
      case 0:
    Dynamixel.moveSpeed(5,MAP(125),100);   //nai7
    Dynamixel.moveSpeed(6,MAP(125),100);

    Dynamixel.moveSpeed(8,MAP(175),100);  //naik
    Dynamixel.moveSpeed(9,MAP(175),100);

    Dynamixel.moveSpeed(17,MAP(125),100);  //naik
    Dynamixel.moveSpeed(18,MAP(125),100);
    delay(90);
      break;
      case 1:
    Dynamixel.moveSpeed(2,MAP(170),100); 
    Dynamixel.moveSpeed(3,MAP(170),100);//turun

    Dynamixel.moveSpeed(11,MAP(135),100);
    Dynamixel.moveSpeed(12,MAP(135),100);//turun

    Dynamixel.moveSpeed(14,MAP(170),100);//turun
    Dynamixel.moveSpeed(15,MAP(170),100);
    delay(90);


    Dynamixel.moveSpeed(4,MAP(110),170); //geser 
    Dynamixel.moveSpeed(16,MAP(165),170);
    Dynamixel.moveSpeed(7,MAP(145),170);  

    Dynamixel.moveSpeed(1,MAP(195),170);
    Dynamixel.moveSpeed(13,MAP(140),170); //geser 
    Dynamixel.moveSpeed(10,MAP(160),170);//geser
    delay(200);
      break;

      case 2:
         /* 2 ---> 3 */
    Dynamixel.moveSpeed(2,MAP(180),100);
    Dynamixel.moveSpeed(3,MAP(180),100); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),100); //angkat
    Dynamixel.moveSpeed(12,MAP(125),100);

    Dynamixel.moveSpeed(14,MAP(180),100);  //angkat
    Dynamixel.moveSpeed(15,MAP(180),100);
    delay(90);
      break;
      case 3:
    Dynamixel.moveSpeed(8,MAP(165),100);
    Dynamixel.moveSpeed(9,MAP(165),100);
    
    Dynamixel.moveSpeed(17,MAP(135),100);
    Dynamixel.moveSpeed(18,MAP(135),100);

    Dynamixel.moveSpeed(5,MAP(135),100); //turun
    Dynamixel.moveSpeed(6,MAP(135),100);
    delay(90);

    
        Dynamixel.moveSpeed(4,MAP(130),170); //geser
    Dynamixel.moveSpeed(16,MAP(185),170);
    Dynamixel.moveSpeed(7,MAP(165),170); 
    
    Dynamixel.moveSpeed(1,MAP(175),170);
    Dynamixel.moveSpeed(10,MAP(140),170);
    Dynamixel.moveSpeed(13,MAP(125),170);
    delay(200);
      break;
    }
    steps++;
    if (steps > 3)
      steps = 0;
  }

}
void LycosidControlClass::turnLslow(unsigned char nturn){
     while(nturn-- > 0){
    switch (steps){
      case 0:
    Dynamixel.moveSpeed(5,MAP(125),100);   //nai7
    Dynamixel.moveSpeed(6,MAP(125),100);

    Dynamixel.moveSpeed(8,MAP(175),100);  //naik
    Dynamixel.moveSpeed(9,MAP(175),100);

    Dynamixel.moveSpeed(17,MAP(125),100);  //naik
    Dynamixel.moveSpeed(18,MAP(125),100);
    delay(90);
      break;
      case 1:
    Dynamixel.moveSpeed(2,MAP(170),100); 
    Dynamixel.moveSpeed(3,MAP(170),100);//turun

    Dynamixel.moveSpeed(11,MAP(135),100);
    Dynamixel.moveSpeed(12,MAP(135),100);//turun

    Dynamixel.moveSpeed(14,MAP(170),100);//turun
    Dynamixel.moveSpeed(15,MAP(170),100);
    delay(90);


    Dynamixel.moveSpeed(4,MAP(130),170); //geser
    Dynamixel.moveSpeed(16,MAP(185),170);
    Dynamixel.moveSpeed(7,MAP(165),170); 
    
    Dynamixel.moveSpeed(1,MAP(175),170);
    Dynamixel.moveSpeed(10,MAP(140),170);
    Dynamixel.moveSpeed(13,MAP(125),170);
    delay(200);
      break;

      case 2:
         /* 2 ---> 3 */
    Dynamixel.moveSpeed(2,MAP(180),100);
    Dynamixel.moveSpeed(3,MAP(180),100); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),100); //angkat
    Dynamixel.moveSpeed(12,MAP(125),100);

    Dynamixel.moveSpeed(14,MAP(180),100);  //angkat
    Dynamixel.moveSpeed(15,MAP(180),100);
    delay(90);
      break;
      case 3:
    Dynamixel.moveSpeed(8,MAP(165),100);
    Dynamixel.moveSpeed(9,MAP(165),100);
    
    Dynamixel.moveSpeed(17,MAP(135),100);
    Dynamixel.moveSpeed(18,MAP(135),100);

    Dynamixel.moveSpeed(5,MAP(135),100); //turun
    Dynamixel.moveSpeed(6,MAP(135),100);
    delay(90);
    Dynamixel.moveSpeed(4,MAP(110),170); //geser 
    Dynamixel.moveSpeed(16,MAP(165),170);
    Dynamixel.moveSpeed(7,MAP(145),170);  

    Dynamixel.moveSpeed(1,MAP(195),170);
    Dynamixel.moveSpeed(13,MAP(140),170); //geser 
    Dynamixel.moveSpeed(10,MAP(160),170);//geser
    delay(200);
      break;
    }
    steps++;
    if (steps > 3)
      steps = 0;
  }

}
void LycosidControlClass::walk(float lRange, float rRange){ // SPEED AWAL
	if (lRange > MAXRange)
    lRange = MAXRange;
  else if (lRange < MAP(5))
    lRange = MAP(5);

  if (rRange > MAXRange)
    rRange = MAXRange;
  else if(rRange < MAP(0))
    rRange = MAP(0);


  lRange = MAXRange - lRange;
  rRange = MAXRange - rRange;


  switch (steps){
    case 0:
      /* 4 ---> 1 */
    Dynamixel.moveSpeed(5,MAP(125),550);   //naik
    Dynamixel.moveSpeed(6,MAP(125),550);

    Dynamixel.moveSpeed(8,MAP(175),550);  //naik
    Dynamixel.moveSpeed(9,MAP(175),550);

    Dynamixel.moveSpeed(17,MAP(125),550);  //naik
    Dynamixel.moveSpeed(18,MAP(125),550);
    delay(20);
      break;

      case 1:
      /* 1 ---> 2 */
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(130),500);
    Dynamixel.moveSpeed(12,MAP(130),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(7);

    Dynamixel.moveSpeed(4,MAP(110)+lRange,700); //geser
    Dynamixel.moveSpeed(7,MAP(165)- rRange,700);  
    Dynamixel.moveSpeed(16,MAP(165)+(lRange/2),700);

    Dynamixel.moveSpeed(1,MAP(170)+ (rRange / 2),700);
    Dynamixel.moveSpeed(10,MAP(160) - lRange,700);//geser
    Dynamixel.moveSpeed(13,MAP(115) - lRange,700);
    delay(70);
    break;

    case 2:
    Dynamixel.moveSpeed(2,MAP(175),550);
    Dynamixel.moveSpeed(3,MAP(175),550); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),550); //angkat
    Dynamixel.moveSpeed(12,MAP(125),550);

    Dynamixel.moveSpeed(14,MAP(175),550);  //angkat
    Dynamixel.moveSpeed(15,MAP(175),550);
    delay(20);
    break;

    case 3:
    Dynamixel.moveSpeed(8,MAP(170),500);
    Dynamixel.moveSpeed(9,MAP(170),500);
    
    Dynamixel.moveSpeed(18,MAP(130),500);
    Dynamixel.moveSpeed(17,MAP(130),500);
    
    Dynamixel.moveSpeed(5,MAP(130),500); //turun
    Dynamixel.moveSpeed(6,MAP(130),500);
    delay(7);
    Dynamixel.moveSpeed(4,MAP(135)+lRange/2,700); //geser
    Dynamixel.moveSpeed(7,MAP(145),700);  
    Dynamixel.moveSpeed(16,MAP(195)-lRange/2,700);

    Dynamixel.moveSpeed(1,MAP(195),700);
    Dynamixel.moveSpeed(13,MAP(140),700); //geser 
    Dynamixel.moveSpeed(10,MAP(140),700);
    delay(70);
    break;
  }
  steps++;
  if (steps == 4)
    steps = 0;
}
void LycosidControlClass::walk3(float lRange, float rRange){
	if (lRange > MAXRange)
    lRange = MAXRange;
  else if (lRange < MAP(5))
    lRange = MAP(5);

  if (rRange > MAXRange)
    rRange = MAXRange;
  else if(rRange < MAP(0))
    rRange = MAP(0);


  lRange = MAXRange - lRange;
  rRange = MAXRange - rRange;


  switch (steps){
    case 0:
      /* 4 ---> 1 */
    Dynamixel.moveSpeed(5,MAP(125),800);   //naik
    Dynamixel.moveSpeed(6,MAP(125),800);

    Dynamixel.moveSpeed(8,MAP(175),800);  //naik
    Dynamixel.moveSpeed(9,MAP(175),800);

    Dynamixel.moveSpeed(17,MAP(125),800);  //naik
    Dynamixel.moveSpeed(18,MAP(125),800);
    delay(10);
      break;

      case 1:
      /* 1 ---> 2 */
    Dynamixel.moveSpeed(2,MAP(170),800); 
    Dynamixel.moveSpeed(3,MAP(170),800);//turun

    Dynamixel.moveSpeed(11,MAP(130),800);
    Dynamixel.moveSpeed(12,MAP(130),800);//turun

    Dynamixel.moveSpeed(14,MAP(170),800);//turun
    Dynamixel.moveSpeed(15,MAP(170),800);
    delay(10);

    Dynamixel.moveSpeed(4,MAP(110),800); //geser
    Dynamixel.moveSpeed(7,MAP(165),800);  
    Dynamixel.moveSpeed(16,MAP(165),800);

    Dynamixel.moveSpeed(1,MAP(170),800);
    Dynamixel.moveSpeed(10,MAP(160),800);//geser
    Dynamixel.moveSpeed(13,MAP(115),800);
    delay(60);
    break;

    case 2:
    Dynamixel.moveSpeed(2,MAP(175),800);
    Dynamixel.moveSpeed(3,MAP(175),800); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),800); //angkat
    Dynamixel.moveSpeed(12,MAP(125),800);

    Dynamixel.moveSpeed(14,MAP(175),800);  //angkat
    Dynamixel.moveSpeed(15,MAP(175),800);
    delay(10);
    break;

    case 3:
    Dynamixel.moveSpeed(8,MAP(170),800);
    Dynamixel.moveSpeed(9,MAP(170),800);
    
    Dynamixel.moveSpeed(18,MAP(130),800);
    Dynamixel.moveSpeed(17,MAP(130),800);
    
    Dynamixel.moveSpeed(5,MAP(130),800); //turun
    Dynamixel.moveSpeed(6,MAP(130),800);
    delay(10);
    
    Dynamixel.moveSpeed(4,MAP(135),800); //geser
    Dynamixel.moveSpeed(7,MAP(145),800);  
    Dynamixel.moveSpeed(16,MAP(195),800);

    Dynamixel.moveSpeed(1,MAP(195),800);
    Dynamixel.moveSpeed(13,MAP(140),800); //geser 
    Dynamixel.moveSpeed(10,MAP(140),800);
    delay(40);
    break;
  }
  steps++;
  if (steps == 4)
    steps = 0;
}

void LycosidControlClass::walk2(float lRange, float rRange){
	if (lRange > MAXRange)
    lRange = MAXRange;
  else if (lRange < MAP(5))
    lRange = MAP(5);

  if (rRange > MAXRange)
    rRange = MAXRange;
  else if(rRange < MAP(0))
    rRange = MAP(0);


  lRange = MAXRange - lRange;
  rRange = MAXRange - rRange;


  switch (steps){
    case 0:
      /* 4 ---> 1 */
    Dynamixel.moveSpeed(5,MAP(125),100);   //naik
    Dynamixel.moveSpeed(6,MAP(125),100);

    Dynamixel.moveSpeed(8,MAP(175),100);  //naik
    Dynamixel.moveSpeed(9,MAP(175),100);

    Dynamixel.moveSpeed(17,MAP(125),100);  //naik
    Dynamixel.moveSpeed(18,MAP(125),100);
    delay(90);
      break;

      case 1:
      /* 1 ---> 2 */
    Dynamixel.moveSpeed(2,MAP(170),100); 
    Dynamixel.moveSpeed(3,MAP(170),100);//turun

    Dynamixel.moveSpeed(11,MAP(130),100);
    Dynamixel.moveSpeed(12,MAP(130),100);//turun

    Dynamixel.moveSpeed(14,MAP(170),100);//turun
    Dynamixel.moveSpeed(15,MAP(170),100);
    delay(90);

    Dynamixel.moveSpeed(4,MAP(110)+lRange,170); //geser
    Dynamixel.moveSpeed(7,MAP(165)- rRange,170);  
    Dynamixel.moveSpeed(16,MAP(165)+(lRange/2),170);

    Dynamixel.moveSpeed(1,MAP(170)+ (rRange / 2),170);
    Dynamixel.moveSpeed(10,MAP(160) - lRange,170);//geser
    Dynamixel.moveSpeed(13,MAP(115) - lRange,170);
    delay(200);
    break;

    case 2:
    Dynamixel.moveSpeed(2,MAP(175),100);
    Dynamixel.moveSpeed(3,MAP(175),100); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),100); //angkat
    Dynamixel.moveSpeed(12,MAP(125),100);

    Dynamixel.moveSpeed(14,MAP(175),100);  //angkat
    Dynamixel.moveSpeed(15,MAP(175),100);
    delay(90);
    break;

    case 3:
    Dynamixel.moveSpeed(8,MAP(170),100);
    Dynamixel.moveSpeed(9,MAP(170),100);
    
    Dynamixel.moveSpeed(18,MAP(130),100);
    Dynamixel.moveSpeed(17,MAP(130),100);
    
    Dynamixel.moveSpeed(5,MAP(130),100); //turun
    Dynamixel.moveSpeed(6,MAP(130),100);
    delay(90);
    
    Dynamixel.moveSpeed(4,MAP(135)+lRange/2,170); //geser
    Dynamixel.moveSpeed(7,MAP(145),170);  
    Dynamixel.moveSpeed(16,MAP(195)-lRange/2,170);

    Dynamixel.moveSpeed(1,MAP(195),170);
    Dynamixel.moveSpeed(13,MAP(140),170); //geser 
    Dynamixel.moveSpeed(10,MAP(140),170);
    delay(200);
    break;
  }
  steps++;
  if (steps == 4)
    steps = 0;
}


void LycosidControlClass::back(float lRange, float rRange){
	if (lRange > MAXRange)
    lRange = MAXRange;
  else if (lRange < MAP(5))
    lRange = MAP(5);

  if (rRange > MAXRange)
    rRange = MAXRange;
  else if(rRange < MAP(0))
    rRange = MAP(0);


  lRange = MAXRange - lRange;
  rRange = MAXRange - rRange;


  switch (steps){
    case 0:
      /* 4 ---> 1 */
    Dynamixel.moveSpeed(5,MAP(125),550);   //naik
    Dynamixel.moveSpeed(6,MAP(125),550);

    Dynamixel.moveSpeed(8,MAP(175),550);  //naik
    Dynamixel.moveSpeed(9,MAP(175),550);

    Dynamixel.moveSpeed(17,MAP(125),550);  //naik
    Dynamixel.moveSpeed(18,MAP(125),550);
    delay(20);
      break;

      case 1:
      /* 1 ---> 2 */
    Dynamixel.moveSpeed(2,MAP(170),500); 
    Dynamixel.moveSpeed(3,MAP(170),500);//turun

    Dynamixel.moveSpeed(11,MAP(130),500);
    Dynamixel.moveSpeed(12,MAP(130),500);//turun

    Dynamixel.moveSpeed(14,MAP(170),500);//turun
    Dynamixel.moveSpeed(15,MAP(170),500);
    delay(7);
    Dynamixel.moveSpeed(4,MAP(135)+lRange/2,700); //geser
    Dynamixel.moveSpeed(7,MAP(145),700);  
    Dynamixel.moveSpeed(16,MAP(195)-lRange/2,700);

    Dynamixel.moveSpeed(1,MAP(195),700);
    Dynamixel.moveSpeed(13,MAP(140),700); //geser 
    Dynamixel.moveSpeed(10,MAP(140),700);
    delay(70);
    break;

    case 2:
    Dynamixel.moveSpeed(2,MAP(175),550);
    Dynamixel.moveSpeed(3,MAP(175),550); //angkat
 
    Dynamixel.moveSpeed(11,MAP(125),550); //angkat
    Dynamixel.moveSpeed(12,MAP(125),550);

    Dynamixel.moveSpeed(14,MAP(175),550);  //angkat
    Dynamixel.moveSpeed(15,MAP(175),550);
    delay(20);
    break;

    case 3:
    Dynamixel.moveSpeed(8,MAP(170),500);
    Dynamixel.moveSpeed(9,MAP(170),500);
    
    Dynamixel.moveSpeed(18,MAP(130),500);
    Dynamixel.moveSpeed(17,MAP(130),500);
    
    Dynamixel.moveSpeed(5,MAP(130),500); //turun
    Dynamixel.moveSpeed(6,MAP(130),500);
    delay(7);
      
    Dynamixel.moveSpeed(4,MAP(110)+lRange,700); //geser
    Dynamixel.moveSpeed(7,MAP(165)- rRange,700);  
    Dynamixel.moveSpeed(16,MAP(165)+(lRange/2),700);

    Dynamixel.moveSpeed(1,MAP(170)+ (rRange / 2),700);
    Dynamixel.moveSpeed(10,MAP(160) - lRange,700);//geser
    Dynamixel.moveSpeed(13,MAP(115) - lRange,700);
    delay(70);
    break;
  }
  steps++;
  if (steps == 4)
    steps = 0;
}
void LycosidControlClass::slidL(unsigned char nturn){
	 while(nturn-- > 0){
    switch (steps){
    case 0:
    Dynamixel.moveSpeed(1,MAP(180),200);
    Dynamixel.moveSpeed(7,MAP(150),200);
    Dynamixel.moveSpeed(13,MAP(120),200);
    Dynamixel.moveSpeed(10,MAP(150),200);
    Dynamixel.moveSpeed(16,MAP(180),200);
    Dynamixel.moveSpeed(4,MAP(120),200);
    
    Dynamixel.moveSpeed(5,MAP(120),550);   //naik
    Dynamixel.moveSpeed(6,MAP(110),550);

    Dynamixel.moveSpeed(8,MAP(180),550);  //naik
    Dynamixel.moveSpeed(9,MAP(160),550);
    
    Dynamixel.moveSpeed(17,MAP(120),550);  //naik
    Dynamixel.moveSpeed(18,MAP(110),550);
    delay(10);
    break;
    case 1:
    Dynamixel.moveSpeed(2,MAP(170),550);
    Dynamixel.moveSpeed(3,MAP(160),550);
 
    Dynamixel.moveSpeed(11,MAP(130),550); 
    Dynamixel.moveSpeed(12,MAP(110),550);

    Dynamixel.moveSpeed(14,MAP(170),550); 
    Dynamixel.moveSpeed(15,MAP(160),550);
    delay(10);
    Dynamixel.moveSpeed(3,MAP(180),550);
    Dynamixel.moveSpeed(12,MAP(130),550);
    Dynamixel.moveSpeed(15,MAP(180),550);
    delay(100);
    break;
    case 2:
    Dynamixel.moveSpeed(5,MAP(130),500); //turun
    Dynamixel.moveSpeed(6,MAP(110),500);
    
    Dynamixel.moveSpeed(8,MAP(170),500);
    Dynamixel.moveSpeed(9,MAP(160),500);
    
    Dynamixel.moveSpeed(17,MAP(130),500);
    Dynamixel.moveSpeed(18,MAP(110),500);
    delay(10);
    break;
    case 3:
    Dynamixel.moveSpeed(2,MAP(180),550);
    Dynamixel.moveSpeed(3,MAP(160),550); //angkat
 
    Dynamixel.moveSpeed(11,MAP(120),550); //angkat
    Dynamixel.moveSpeed(12,MAP(110),550);

    Dynamixel.moveSpeed(14,MAP(180),550);  //angkat
    Dynamixel.moveSpeed(15,MAP(160),550);
    delay(10);
    Dynamixel.moveSpeed(6,MAP(130),500);
    Dynamixel.moveSpeed(9,MAP(180),500);
    Dynamixel.moveSpeed(18,MAP(130),500);
    delay(100);
    break;
  }
   steps++;
     if (steps > 3)
    steps = 0;
    }
	}
void LycosidControlClass::slidR(unsigned char nturn){
  while(nturn-- > 0){
    switch (steps){
    case 0:
    Dynamixel.moveSpeed(1,MAP(180),200);
    Dynamixel.moveSpeed(7,MAP(150),200);
    Dynamixel.moveSpeed(13,MAP(120),200);
    Dynamixel.moveSpeed(10,MAP(150),200);
    Dynamixel.moveSpeed(16,MAP(180),200);
    Dynamixel.moveSpeed(4,MAP(120),200); 

    Dynamixel.moveSpeed(5,MAP(120),550);   //naik
    Dynamixel.moveSpeed(6,MAP(140),550);

    Dynamixel.moveSpeed(8,MAP(180),550);  //naik
    Dynamixel.moveSpeed(9,MAP(190),550);
    
    Dynamixel.moveSpeed(17,MAP(120),550);  //naik
    Dynamixel.moveSpeed(18,MAP(140),550);
    delay(10);
    break;
    case 1:
    Dynamixel.moveSpeed(2,MAP(170),550);   //turun
    Dynamixel.moveSpeed(3,MAP(190),550);

    Dynamixel.moveSpeed(11,MAP(130),550);  //turun
    Dynamixel.moveSpeed(12,MAP(140),550);
    
    Dynamixel.moveSpeed(14,MAP(170),550);  //turun
    Dynamixel.moveSpeed(15,MAP(190),550);
    delay(10);
    Dynamixel.moveSpeed(3,MAP(170),500);
    Dynamixel.moveSpeed(12,MAP(120),500);
    Dynamixel.moveSpeed(15,MAP(150),500);
    delay(100);
    break;
    case 2:
    Dynamixel.moveSpeed(5,MAP(130),550);   //turun
    Dynamixel.moveSpeed(6,MAP(140),550);

    Dynamixel.moveSpeed(8,MAP(170),550);  //turun
    Dynamixel.moveSpeed(9,MAP(190),550);
    
    Dynamixel.moveSpeed(17,MAP(130),550);  //turun
    Dynamixel.moveSpeed(18,MAP(140),550);
    delay(10);
    break;
    case 3:
    Dynamixel.moveSpeed(2,MAP(180),550);   //naik
    Dynamixel.moveSpeed(3,MAP(190),550);

    Dynamixel.moveSpeed(11,MAP(120),550);  //naik
    Dynamixel.moveSpeed(12,MAP(140),550);
    
    Dynamixel.moveSpeed(14,MAP(180),550);  //naik
    Dynamixel.moveSpeed(15,MAP(190),550);
    delay(10);
    Dynamixel.moveSpeed(6,MAP(120),500);
    Dynamixel.moveSpeed(9,MAP(170),500);
    Dynamixel.moveSpeed(18,MAP(120),500);
    delay(100);
    break;
  }
  steps++;
     if (steps > 3)
    steps = 0;
    }
}

void LycosidControlClass::exting(int range){
      Dynamixel.moveSpeed(ILEG3_B, SLEG3_B[0] - (range), speed1);
      Dynamixel.moveSpeed(ILEG3_C, SLEG3_C[0] - (range), speed1);

      Dynamixel.moveSpeed(ILEG4_B, SLEG4_B[0] - (range), speed1);
      Dynamixel.moveSpeed(ILEG4_C, SLEG4_C[0] - (range), speed1);

      Dynamixel.moveSpeed(ILEG1_B, SLEG1_B[0] + (range * 2), speed1);
      Dynamixel.moveSpeed(ILEG1_C, SLEG1_C[0] + (range * 2), speed1);

      Dynamixel.moveSpeed(ILEG6_B, SLEG6_B[0] + (range * 2), speed1);
      Dynamixel.moveSpeed(ILEG6_C, SLEG6_C[0] + (range * 2), speed1);

      Dynamixel.moveSpeed(ILEG2_B, SLEG2_B[0] + (range), speed1);
      Dynamixel.moveSpeed(ILEG2_C, SLEG2_C[0] + (range), speed1);

      Dynamixel.moveSpeed(ILEG5_B, SLEG5_B[0] + (range), speed1);
      Dynamixel.moveSpeed(ILEG5_C, SLEG5_C[0] + (range), speed1);


      //stand();
}
void climb1(){
    Dynamixel.moveSpeed(5,MAP(95),150);   //naik
    Dynamixel.moveSpeed(6,MAP(95),150);

    Dynamixel.moveSpeed(8,MAP(195),150);  //naik
    Dynamixel.moveSpeed(9,MAP(195),150);

    Dynamixel.moveSpeed(17,MAP(95),150);  //naik
    Dynamixel.moveSpeed(18,MAP(95),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(150),200); 
    Dynamixel.moveSpeed(3,MAP(150),200);//turun

    Dynamixel.moveSpeed(11,MAP(150),200);
    Dynamixel.moveSpeed(12,MAP(150),200);//turun

    Dynamixel.moveSpeed(14,MAP(150),200);//turun
    Dynamixel.moveSpeed(15,MAP(150),200);
    delay(300);

    Dynamixel.moveSpeed(4,MAP(120),150); //geser
    Dynamixel.moveSpeed(7,MAP(165),150);  
    Dynamixel.moveSpeed(16,MAP(165),150);

    Dynamixel.moveSpeed(1,MAP(165),150);
    Dynamixel.moveSpeed(10,MAP(160),150);//geser
    Dynamixel.moveSpeed(13,MAP(125),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(195),150);
    Dynamixel.moveSpeed(3,MAP(195),150); //angkat
 
    Dynamixel.moveSpeed(11,MAP(105),150); //angkat
    Dynamixel.moveSpeed(12,MAP(105),150);

    Dynamixel.moveSpeed(14,MAP(195),150);  //angkat
    Dynamixel.moveSpeed(15,MAP(195),150);
    delay(500);
    Dynamixel.moveSpeed(8,MAP(150),200);
    Dynamixel.moveSpeed(9,MAP(150),200);
    
    Dynamixel.moveSpeed(17,MAP(150),200);
    Dynamixel.moveSpeed(18,MAP(150),200);

    Dynamixel.moveSpeed(5,MAP(150),200); //turun
    Dynamixel.moveSpeed(6,MAP(150),200);
    delay(300);
    Dynamixel.moveSpeed(4,MAP(140),150); //geser
    Dynamixel.moveSpeed(7,MAP(145),150);  
    Dynamixel.moveSpeed(16,MAP(185),150);

    Dynamixel.moveSpeed(1,MAP(185),150);
    Dynamixel.moveSpeed(13,MAP(140),150); //geser 
    Dynamixel.moveSpeed(10,MAP(140),150);
    delay(500);
}
void climb2(){
    Dynamixel.moveSpeed(5,MAP(95),150);   //naik
    Dynamixel.moveSpeed(6,MAP(95),150);

    Dynamixel.moveSpeed(8,MAP(195),150);  //naik
    Dynamixel.moveSpeed(9,MAP(195),150);

    Dynamixel.moveSpeed(17,MAP(95),150);  //naik
    Dynamixel.moveSpeed(18,MAP(95),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(180),200); 
    Dynamixel.moveSpeed(3,MAP(180),200);//turun

    Dynamixel.moveSpeed(11,MAP(140),200);
    Dynamixel.moveSpeed(12,MAP(140),200);//turun

    Dynamixel.moveSpeed(14,MAP(150),200);//turun
    Dynamixel.moveSpeed(15,MAP(150),200);
    delay(300);

    Dynamixel.moveSpeed(4,MAP(120),150); //geser
    Dynamixel.moveSpeed(7,MAP(165),150);  
    Dynamixel.moveSpeed(16,MAP(165),150);

    Dynamixel.moveSpeed(1,MAP(165),150);
    Dynamixel.moveSpeed(10,MAP(160),150);//geser
    Dynamixel.moveSpeed(13,MAP(125),150);
    delay(500);
  Dynamixel.moveSpeed(2,MAP(205),100); 
  Dynamixel.moveSpeed(3,MAP(150),100); //angkat
 
    Dynamixel.moveSpeed(11,MAP(105),150); //angkat
    Dynamixel.moveSpeed(12,MAP(105),150);

    Dynamixel.moveSpeed(14,MAP(195),150);  //angkat
    Dynamixel.moveSpeed(15,MAP(195),150);
    delay(500);
    Dynamixel.moveSpeed(8,MAP(170),200);
    Dynamixel.moveSpeed(9,MAP(170),200);
    
    Dynamixel.moveSpeed(17,MAP(150),200);
    Dynamixel.moveSpeed(18,MAP(150),200);

    Dynamixel.moveSpeed(5,MAP(120),200); //turun
    Dynamixel.moveSpeed(6,MAP(120),200);
    delay(300);
    Dynamixel.moveSpeed(4,MAP(140),150); //geser
    Dynamixel.moveSpeed(7,MAP(145),150);  
    Dynamixel.moveSpeed(16,MAP(185),150);

    Dynamixel.moveSpeed(1,MAP(185),150);
    Dynamixel.moveSpeed(13,MAP(140),150); //geser 
    Dynamixel.moveSpeed(10,MAP(140),150);
    delay(500);
}
void climb3(){
    Dynamixel.moveSpeed(5,MAP(95),100); 
    Dynamixel.moveSpeed(6,MAP(150),100);

    Dynamixel.moveSpeed(8,MAP(195),150);  //naik
    Dynamixel.moveSpeed(9,MAP(195),150);

    Dynamixel.moveSpeed(17,MAP(120),150);  //naik
    Dynamixel.moveSpeed(18,MAP(120),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(200),200); 
    Dynamixel.moveSpeed(3,MAP(200),200);//turun

    Dynamixel.moveSpeed(11,MAP(110),200);
    Dynamixel.moveSpeed(12,MAP(110),200);//turun

    Dynamixel.moveSpeed(14,MAP(150),200);//turun
    Dynamixel.moveSpeed(15,MAP(150),200);
    delay(300);

    Dynamixel.moveSpeed(4,MAP(120),150); //geser
    Dynamixel.moveSpeed(7,MAP(165),150);  
    Dynamixel.moveSpeed(16,MAP(165),150);

    Dynamixel.moveSpeed(1,MAP(165),150);
    Dynamixel.moveSpeed(10,MAP(160),150);//geser
    Dynamixel.moveSpeed(13,MAP(125),150);
    delay(500);
   Dynamixel.moveSpeed(2,MAP(205),100); 
   Dynamixel.moveSpeed(3,MAP(150),100); //angkat
 
    Dynamixel.moveSpeed(11,MAP(105),150); //angkat
    Dynamixel.moveSpeed(12,MAP(105),150);

    Dynamixel.moveSpeed(14,MAP(170),150);  //angkat
    Dynamixel.moveSpeed(15,MAP(170),150);
    delay(500);
    Dynamixel.moveSpeed(8,MAP(180),200);
    Dynamixel.moveSpeed(9,MAP(180),200);
    
    Dynamixel.moveSpeed(17,MAP(150),200);
    Dynamixel.moveSpeed(18,MAP(150),200);

    Dynamixel.moveSpeed(5,MAP(95),200); //turun
    Dynamixel.moveSpeed(6,MAP(95),200);
    delay(300);
    Dynamixel.moveSpeed(4,MAP(140),150); //geser
    Dynamixel.moveSpeed(7,MAP(145),150);  
    Dynamixel.moveSpeed(16,MAP(185),150);

    Dynamixel.moveSpeed(1,MAP(185),150);
    Dynamixel.moveSpeed(13,MAP(140),150); //geser 
    Dynamixel.moveSpeed(10,MAP(140),150);
    delay(500);
}
void climb4(){
    Dynamixel.moveSpeed(5,MAP(95),100); 
    Dynamixel.moveSpeed(6,MAP(150),100);

    Dynamixel.moveSpeed(8,MAP(205),150);  //naik
    Dynamixel.moveSpeed(9,MAP(150),150);

    Dynamixel.moveSpeed(17,MAP(120),150);  //naik
    Dynamixel.moveSpeed(18,MAP(120),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(200),200); 
    Dynamixel.moveSpeed(3,MAP(200),200);//turun

    Dynamixel.moveSpeed(11,MAP(110),200);
    Dynamixel.moveSpeed(12,MAP(110),200);//turun

    Dynamixel.moveSpeed(14,MAP(150),200);//turun
    Dynamixel.moveSpeed(15,MAP(150),200);
    delay(300);

    Dynamixel.moveSpeed(4,MAP(120),150); //geser
    Dynamixel.moveSpeed(7,MAP(165),150);  
    Dynamixel.moveSpeed(16,MAP(165),150);

    Dynamixel.moveSpeed(1,MAP(165),150);
    Dynamixel.moveSpeed(10,MAP(160),150);//geser
    Dynamixel.moveSpeed(13,MAP(125),150);
    delay(500);
   Dynamixel.moveSpeed(2,MAP(205),100); 
   Dynamixel.moveSpeed(3,MAP(150),100); //angkat
 
    Dynamixel.moveSpeed(11,MAP(95),150); //angkat
    Dynamixel.moveSpeed(12,MAP(150),150);

    Dynamixel.moveSpeed(14,MAP(170),150);  //angkat
    Dynamixel.moveSpeed(15,MAP(170),150);
    delay(500);
    Dynamixel.moveSpeed(8,MAP(180),200);
    Dynamixel.moveSpeed(9,MAP(180),200);
    
    Dynamixel.moveSpeed(17,MAP(150),200);
    Dynamixel.moveSpeed(18,MAP(150),200);

    Dynamixel.moveSpeed(5,MAP(95),200); //turun
    Dynamixel.moveSpeed(6,MAP(95),200);
    delay(300);
    Dynamixel.moveSpeed(4,MAP(140),150); //geser
    Dynamixel.moveSpeed(7,MAP(145),150);  
    Dynamixel.moveSpeed(16,MAP(185),150);

    Dynamixel.moveSpeed(1,MAP(185),150);
    Dynamixel.moveSpeed(13,MAP(140),150); //geser 
    Dynamixel.moveSpeed(10,MAP(140),150);
    delay(500);
}
void climb5(){
    Dynamixel.moveSpeed(5,MAP(130),150);   //naik
    Dynamixel.moveSpeed(6,MAP(130),150);

    Dynamixel.moveSpeed(8,MAP(195),150);  //naik
    Dynamixel.moveSpeed(9,MAP(195),150);

    Dynamixel.moveSpeed(17,MAP(95),150);  //naik
    Dynamixel.moveSpeed(18,MAP(95),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(150),200); 
    Dynamixel.moveSpeed(3,MAP(150),200);//turun

    Dynamixel.moveSpeed(11,MAP(130),200);
    Dynamixel.moveSpeed(12,MAP(130),200);//turun

    Dynamixel.moveSpeed(14,MAP(180),200);//turun
    Dynamixel.moveSpeed(15,MAP(180),200);
    delay(300);

    Dynamixel.moveSpeed(4,MAP(120),150); //geser
    Dynamixel.moveSpeed(7,MAP(165),150);  
    Dynamixel.moveSpeed(16,MAP(165),150);

    Dynamixel.moveSpeed(1,MAP(165),150);
    Dynamixel.moveSpeed(10,MAP(160),150);//geser
    Dynamixel.moveSpeed(13,MAP(125),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(165),150);
    Dynamixel.moveSpeed(3,MAP(165),150); //angkat
 
    Dynamixel.moveSpeed(11,MAP(105),150); //angkat
    Dynamixel.moveSpeed(12,MAP(105),150);

    Dynamixel.moveSpeed(14,MAP(195),150);  //angkat
    Dynamixel.moveSpeed(15,MAP(195),150);
    delay(500);
    Dynamixel.moveSpeed(8,MAP(170),200);
    Dynamixel.moveSpeed(9,MAP(170),200);
    
    Dynamixel.moveSpeed(17,MAP(110),200);
    Dynamixel.moveSpeed(18,MAP(110),200);

    Dynamixel.moveSpeed(5,MAP(150),200); //turun
    Dynamixel.moveSpeed(6,MAP(150),200);
    delay(300);
    Dynamixel.moveSpeed(4,MAP(140),150); //geser
    Dynamixel.moveSpeed(7,MAP(145),150);  
    Dynamixel.moveSpeed(16,MAP(185),150);

    Dynamixel.moveSpeed(1,MAP(185),150);
    Dynamixel.moveSpeed(13,MAP(140),150); //geser 
    Dynamixel.moveSpeed(10,MAP(140),150);
    delay(500);
}
  void climb6(){
    Dynamixel.moveSpeed(5,MAP(95),150);   //naik
    Dynamixel.moveSpeed(6,MAP(95),150);

    Dynamixel.moveSpeed(8,MAP(195),150);  //naik
    Dynamixel.moveSpeed(9,MAP(195),150);

    Dynamixel.moveSpeed(17,MAP(95),150);  //naik
    Dynamixel.moveSpeed(18,MAP(95),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(150),200); 
    Dynamixel.moveSpeed(3,MAP(150),200);//turun

    Dynamixel.moveSpeed(11,MAP(150),200);
    Dynamixel.moveSpeed(12,MAP(150),200);//turun

    Dynamixel.moveSpeed(14,MAP(170),200);//turun
    Dynamixel.moveSpeed(15,MAP(170),200);
    delay(300);

    Dynamixel.moveSpeed(4,MAP(120),150); //geser
    Dynamixel.moveSpeed(7,MAP(165),150);  
    Dynamixel.moveSpeed(16,MAP(165),150);

    Dynamixel.moveSpeed(1,MAP(165),150);
    Dynamixel.moveSpeed(10,MAP(160),150);//geser
    Dynamixel.moveSpeed(13,MAP(125),150);
    delay(500);
    Dynamixel.moveSpeed(2,MAP(195),150);
    Dynamixel.moveSpeed(3,MAP(195),150); //angkat
 
    Dynamixel.moveSpeed(11,MAP(105),150); //angkat
    Dynamixel.moveSpeed(12,MAP(105),150);

    Dynamixel.moveSpeed(14,MAP(195),150);  //angkat
    Dynamixel.moveSpeed(15,MAP(195),150);
    delay(500);
    Dynamixel.moveSpeed(8,MAP(150),200);
    Dynamixel.moveSpeed(9,MAP(150),200);
    
    Dynamixel.moveSpeed(17,MAP(130),200);
    Dynamixel.moveSpeed(18,MAP(130),200);

    Dynamixel.moveSpeed(5,MAP(150),200); //turun
    Dynamixel.moveSpeed(6,MAP(150),200);
    delay(300);
    Dynamixel.moveSpeed(4,MAP(140),150); //geser
    Dynamixel.moveSpeed(7,MAP(145),150);  
    Dynamixel.moveSpeed(16,MAP(185),150);

    Dynamixel.moveSpeed(1,MAP(185),150);
    Dynamixel.moveSpeed(13,MAP(140),150); //geser 
    Dynamixel.moveSpeed(10,MAP(140),150);
    delay(500);
}
void LycosidControlClass::climb(void){
	climb1();
	delay(1000);
	climb2();
	delay(1000);
	climb3();
	delay(1000);
	climb3();
	delay(1000);
	climb4();
	delay(1000);
	climb4();
	delay(1000);
	climb5();
	delay(1000);
	climb5();
	delay(1000);
	climb5();
	delay(1000);
	climb5();
	delay(1000);
	climb6();
	delay(1000);
}
LycosidControlClass Control;

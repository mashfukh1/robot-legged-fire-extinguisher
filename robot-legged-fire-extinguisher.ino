

/*FABOLOUS lumayan
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
  Copyright (C) 2016  IR-64 POLITEKNIK NEGERI JEMBER

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


  AUTHOR
  Amad Waris Al H (warisafidz@gmail.com)
  Nashrul Bahri ()
  Muhammad Masfukh ()
*/

#include <LycosidComm.h>
#include <LycosidCompute.h>
#include <Servo.h>
#include <Wire.h>
#include <SharpIR.h>

//#include <HMC5883L.h>

/*  Macro ekspansi ----> */
#define MAP(x)   (int)map(x, 0, 300, 0, 1023) // Macro mapping untuk merubah skala sudut ke skala dynamixel
/*  ----| Macro ekspansi*/

/* Polymorfism */
Servo scanner;
//HMC5883L cmps;
/* Tata Letak PING
             FR
    .....##########.....
    ....############....
    LF ###........### RF
    ..###..........###..
    .###............###.
    ###..............###
    ##................##
    ##................##
  LE ##................## RI
    ##................##
    ##................##
    ##................##
    ###..............###
    .###............###.
    ..###..........###..
    ...###........###...
    .LB.############.RB.
    .....##########.....
             BA
*/
/* PIN Digital -----> */
#define STARTB    7   // emergency start button
#define SCANNER   42   // Servo Scanner
#define FLED      6   // FLAME LED
#define SLED      5   // SOUND LED
#define NALED     4   // N/A LED

#define LB_PING     22    // PING Kiri Belakang
#define LE_PING     24    // PING Kiri (LEft)
#define LF_PING     26    // PING Kiri Serong Depan
#define FRD_PING    28    // PING depan bawah 
#define FRMR_PING   30    // PING Depan tengah kanan
#define FRML_PING   32    // PING Depan tengah kiri
#define RB_PING     48    // PING Kanan belakang
#define RI_PING     50    // PING Kiri (RIght)
#define RF_PING     52    // PING Kanan serong
#define SHIELD_L    34    // Limit switch kiri
#define SHIELD_R    36    // Limit switch kanan
#define INFRA_A     38    // proximity atas kiri
#define INFRA_B     44    // proximity atas kanan
#define INFRA_C      2     // proximity boneka
#define parabola    46    // servo scan room
/* -----| PIN Digital */

/*
   Tata letak sensor proximity

  1 ###....### 2
  ###....###
  ..#....#..
  ...#..#...
  ....## 3
  ....##....
  ...#..#...
  ..#....#..
  ###....###
  4 ###....### 5

*/

/* PIN Analog ---> */
// PROXYMITI PIN
#define UVTRON    A8   // Sensor UVTRON C1
#define UVTRON1   A9   // Sensor UVTRON C2
#define PROX1     13   // Sensor Proximity1
#define PROX2     14   // Sensor Proximity2
#define FSENS     4    // Sensor Frekunsi Suara
#define SHARP     A10
#define SHARP_MODEL   430 // MODEL 0A41SK

/* ----| PIN Analog */

/* Treshold ------> */

// TRESHOLD SENSOR
#define PTRESH    400 // Treshold Proximity
#define FTRESH    500 // Treshold Sensor suara
#define UVTRESH   8  // Treshold UVTRON

// TRESHOLD NILAI
#define ILB_PING  0
#define ILE_PING  1
#define ILF_PING  2
#define IFR_PING  3
#define IRF_PING  4
#define IRI_PING  5
#define IRB_PING  6

#define RIGHT   true
#define LEFT    false
#define COS     95
#define LLSCAN  (COS - 60)
#define RLSCAN  (COS + 60)
#define LTLSCAN (COS - 60)  // Limit scan kiri saat travel
#define RTLSCAN (COS + 60)  // Limit scan kanan saat travel
/* -------| Treshold */

/* Konstanta Nilai ----> */
// PID
#define setPoint  15.0    // Jarak robot dengan dinding
#define KP      7.7   // Konstanta Proporsional PID
#define KD      0.77   // Konstanta Derivative / Turunan PID NOTE: 0.4
#define KI      0   // Konstanta Integral PID
#define TS      1   // Konstanta Time Sampling PID

/* ----| Konstanta Nilai */

/* Variabel global ---> */
float lastErr = 0;
unsigned char posTravel;
bool ctrlTravel;
int posMon = 0;
int tpa_val_max = 0;
char roomCounter = 0;
float fPos=0;   // flame position
/* ---| Variabel global */

/* Prototype Fungsi ----> */
void setup(void);
void loop(void);
//  Fungsi matematika untuk perhitungan
float PID(float pv, float sp);  // Perhitungan PID
// Fungsi navigasi dan control
void soundRec(void);
void oriCheck(void);
void naviOrigin(void);
void homeAv(void);
void findWall(void);
void findWall2nd(void);
void findWall3rd(void);
void rFollow(void);
void lFollow(void);
bool scanRoom(void);
bool scanRoom2(void);
void lineVoidL(void);
void lineVoidR(void);
void SerialClr(void);
bool fireEx(void);
bool fireEx2(void);
bool fireScan(void);
int rightScan(void);
int leftScan(void);
void treshSampling(void);
void fSampling(void);
bool TPA_scan();
bool TPA_scan2();
bool TPA_level(void);
bool TPA_level2(void);
bool TPA_mon(void);
bool firing(void);
void firePoint(void);
bool TPA_Travel(void);
bool gateLine(void);
int cmpsRDiff(int mark);
int cmpsLDiff(int mark);
int wallDetect(int *ptr);
bool degDev(int src, int dst);
bool readUV(void);
bool tpaFire(void);
void goHome(void);
void exitRoom(void);
void pingFire(void);
void standBy(void);
int state;
unsigned long previousMillis = 0; 
//float readCMPS(void);

/* -----| Prototype Fungsi */

/*
   Deklarasi untuk semua fungsi
*/


void setup() {
  delay(100);
  Serial.begin(9600); // Komunikasi Serial Arduino Mega -> PC
  Serial2.begin(9600);
  Serial3.begin(57600);  // Komunikasi Serial Arduino Mega -> Nano
  Wire.begin();
  //  Compute.initCMPS();
  scanner.attach(SCANNER);

  pinMode(FLED, OUTPUT);
  pinMode(SLED, OUTPUT);
  pinMode(NALED, OUTPUT);
  pinMode(STARTB, INPUT_PULLUP);
  pinMode(SHIELD_L, INPUT_PULLUP);
  pinMode(SHIELD_R, INPUT_PULLUP);
  pinMode(INFRA_A, INPUT);
  pinMode(INFRA_B, INPUT);
  pinMode(INFRA_C, INPUT);

  digitalWrite(FLED, LOW);
  digitalWrite(SLED, LOW);
  delay(200);

  scanner.write(COS);

  Compute.keyPost(SSPEED, S2);
  Compute.keyPost(SYNC);

  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);

  // delay(500);

  digitalWrite(NALED, HIGH);
  delay(80);
  digitalWrite(NALED, LOW);
  delay(80);
  digitalWrite(NALED, HIGH);
  delay(80);
  digitalWrite(NALED, LOW);
  Compute.keyPost(SSPEED,3);
  standBy();
  digitalWrite(SLED,HIGH);
  delay(700);
  digitalWrite(SLED,LOW);
  //BISMILLAH();
  homeAv();


}

void loop() {
  static int fireStat, i;
  static bool FIRE, tmp;
  if (Serial.available() > 0) {
    delay(10);
    state = Serial.read();
  }
  Compute.keyPost(SSPEED, S2);
  Compute.keyPost(SYNC);
  lFollow();
  roomCounter++;
  Compute.keyPost(SSPEED, S2);
  Compute.keyPost(SYNC);
  fireStat = scanRoom2();
  //  fireStat = tpaFire();

  /*
      Metode deteksi api:
        Jika UVTRON mendeteksi api namun TPA tidak
        maka pasti berada di pojok ruangan (baca
        FAQ untuk mengetahui kemungkinan posisi
        lilin)
  */

  // jika UVTRON mendeteksi api
  if (fireStat == true) {
    digitalWrite(FLED, HIGH);

    Compute.keyPost(SSPEED, S3);
    FIRE = tpaFire();
    //      FIRE = true;

    // jika TPA mendeteksi api
    if (FIRE == true) {
      if ( fireEx() == true) {
        digitalWrite(FLED, LOW);
        exitRoom();
        Compute.keyPost(SSPEED, S3);
        goHome();
      }
      else
        exitRoom();
    }
    else {
      // jika TPA tidak mendeteksi api, maka api
      // pasti berada dibelakang furniture dan
      // terdapat pada pojok ruangan
      Compute.keyPost(TURNR, 14);

      Compute.keyPost(GETUP);

      while (Compute.readPing(FRD_PING) > 10) {
        Compute.keyPost(WALK, 1);
        delay(5);
      }
      Compute.keyPost(SYNC);

      posTravel = LTLSCAN;
      ctrlTravel = RIGHT;
      scanner.write(posTravel);

      Compute.keyPost(AUTO_PILOT);
      Compute.keyPost(SYNC);


      FIRE = false;
      while (1) {
        static float pid, pfd, plf;

        plf = Compute.readPing(LF_PING);
        delay(5);
        pfd = digitalRead(INFRA_C);
        pid = PID(plf, 11);

        if (pfd==LOW) {
          Compute.keyPost(TURNR, 14);
          Compute.keyPost(SYNC);
        }
        else {
          while (Compute.tryPost(FORWARD,pid) == false) {
            FIRE = TPA_Travel();
            if (FIRE == true)
              break;
          }
          if (FIRE == true)
            break;
        }
        delay(2);
      }

      Compute.keyPost(MANUAL);
      Compute.keyPost(SYNC);
      Compute.keyPost(GETUP);
      if (fireEx2() == true) {
        digitalWrite(FLED, LOW);
        exitRoom();
        Compute.keyPost(SSPEED, S2);
        goHome();
      }
      else
        exitRoom();
    }
  }

}
/*
   Perhutungan PID yang akan dikirim ke CONTROL melalui serial
*/

float PID(float pv, float sp) {
  float err, P, D;

  err = sp - pv;
  P = KP * err;
  D = ((KD * 10.0) / TS) * (err - lastErr);
  lastErr = err;

  return (P + D);

}

/*
   Fungsi untuk menuggu aksi dari SOUND ACTIVATOR
   jika sound ACTIVATOR Gagal maka akan dibantu dengan start Button
*/
void soundRec(void) {
  while (analogRead(FSENS) > FTRESH || digitalRead(STARTB) == HIGH);
}

/*
   Check Orientasi Robot terhadap dinding, jika tidak
  sejajar dengan dinding (arah pergerakan) maka luruskan dengan kompas atau dengan
   navigasi PING)))
*/
void oriCheck(void) {
#ifdef CMPSENS
  while (cmpRead() != ORIGIN)
    turnR(1);
#else
  naviOrigin(); // not recomended
#endif
}

/*
   Fungsi untuk menghadapkan robot dengan pintu
   keluar pada arbitary start atau lorong pada non arbitary
*/
void naviOrigin(void) {
  return;
}

void homeAv(void) {
  lFollow();
  lineVoidL();
}

/*
  ALHAMDULILLAH, yang ini bisa
  BUG : tidak dapat memilih putaran terendah, inspect fungsi cmpsXDiff()

*/

/*
      .::BISMILLAH::.
*/
void BISMILLAH(void) {
  int LPING[3], RPING[3], mark, WALL_MIN, pointer, LWALL, RWALL, tmp;
  char nturn;

  nturn = 32;

  pointer = nturn;

  WALL_MIN = 0xff;
  Compute.keyPost(SSPEED, S2);
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  // possible
  while (nturn > 0) {
    /*    Serial.print(mark);
      Serial.print("\t");
      Serial.print(Compute.readCMPS());
      Serial.print("\t");
      Serial.println(tmp);
    */
    LPING[0] = Compute.readPing(LB_PING);
    delay(5);
    LPING[1] = Compute.readPing(LE_PING);
    delay(5);
    LPING[2] = Compute.readPing(LF_PING);
    delay(5);
    RPING[0] = Compute.readPing(RB_PING);
    delay(5);
    RPING[1] = Compute.readPing(RI_PING);
    delay(5);
    RPING[2] = Compute.readPing(RF_PING);
    delay(5);

    LWALL = wallDetect(LPING);
    RWALL = wallDetect(RPING);

    if (LWALL < WALL_MIN) {
      WALL_MIN = LWALL;
      pointer = nturn;
      //  delay(3000);
      /*      Serial.print("LWALL: ");
        Serial.print(WALL_MIN);
        Serial.print("\t");
        Serial.print(pointer);*/
      delay(5);
    }

    if (RWALL < WALL_MIN) {
      WALL_MIN = RWALL;
      pointer = nturn;
      //delay(3000);
      /*      Serial.print("RWALL: ");
        Serial.print(WALL_MIN);
        Serial.print("\t");
        Serial.print(pointer);*/
      delay(5);
    }


    Compute.keyPost(TURNR, 1);
    nturn--;
    delay(5);
  }

  if (pointer % 2 != 0)
    pointer += 1;
  //  pointer -= 16;

  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);

  Compute.keyPost(TURNL, pointer);
  Compute.keyPost(GETUP);
  //delay(5000);
  delay(10);
  LPING[0] = Compute.readPing(LB_PING);
  delay(5);
  LPING[1] = Compute.readPing(LE_PING);
  delay(5);
  LPING[2] = Compute.readPing(LF_PING);
  delay(5);
  RPING[0] = Compute.readPing(RB_PING);
  delay(5);
  RPING[1] = Compute.readPing(RI_PING);
  delay(5);
  RPING[2] = Compute.readPing(RF_PING);
  delay(5);

  LWALL = wallDetect2(LPING);
  RWALL = wallDetect2(RPING);

  /*  Serial.print("RESULR: ");
    Serial.print("LWALL :");
    Serial.print(LWALL);
    Serial.print("\tRWALL :");
    Serial.println(RWALL);*/
  if (LWALL < RWALL) {
    Compute.keyPost(TURNL, 16);
  }
  else {
    Compute.keyPost(TURNR, 16);
  }
  //Compute.keyPost(GETUP);
  //Compute.keyPost(SYNC);
  delay(5);

  //while(1);

  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  delay(5);
  while (1) {
    Compute.keyPost(WALK, 1);
    if (Compute.readPing(FRD_PING) < 12) {
      Compute.keyPost(TURNR, 16);
      break;
    }
    delay(5);
    if (Compute.readPing(RF_PING) < 12) {
      Compute.keyPost(TURNR, 26);
      break;
    }
    /*
      delay(1);
      if(Compute.readPing(RI_PING) < 12){
      Compute.keyPost(TURNR, 32);
      break;
      }
    */
    delay(5);
    if (Compute.readPing(LF_PING) < 12)
      break;
    /*
      delay(1);
      if(Compute.readPing(LE_PING) < 12)
      break;
    */
    delay(5);
  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
}

void findWall3rd(void) {
  int LPING[3], RPING[3], mark, WALL_MIN, pointer, LWALL, RWALL, tmp;
  bool turnTo;
  mark = Compute.readCMPS() + 5; // adjust to prevent error
  pointer = mark;

  WALL_MIN = 0xff;
  Compute.keyPost(SSPEED, S3);
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  // possible
  while ((tmp = cmpsRDiff(mark)) < 150) {
    /*    Serial.print(mark);
      Serial.print("\t");
      Serial.print(Compute.readCMPS());
      Serial.print("\t");
      Serial.println(tmp);
    */
    LPING[0] = Compute.readPing(LB_PING);
    delay(5);
    LPING[1] = Compute.readPing(LE_PING);
    delay(5);
    LPING[2] = Compute.readPing(LF_PING);
    delay(5);
    RPING[0] = Compute.readPing(RB_PING);
    delay(5);
    RPING[1] = Compute.readPing(RI_PING);
    delay(5);
    RPING[2] = Compute.readPing(RF_PING);
    delay(5);

    LWALL = wallDetect(LPING);
    RWALL = wallDetect(RPING);

    if (LWALL < WALL_MIN) {
      WALL_MIN = LWALL;
      pointer = Compute.readCMPS();
      /*      Serial.print("LWALL: ");
        Serial.print(WALL_MIN);
        Serial.print("\t");
        Serial.print(pointer);*/
      delay(5);
    }

    if (RWALL < WALL_MIN) {
      WALL_MIN = RWALL;
      pointer = Compute.readCMPS();
      /*      Serial.print("RWALL: ");
        Serial.print(WALL_MIN);
        Serial.print("\t");
        Serial.print(pointer);*/
      delay(5);
    }


    Compute.tryPost(TURNR, 1);
    delay(5);
  }
  turnTo = degDev(Compute.readCMPS(), pointer);

  if (turnTo == RIGHT) {
    while (cmpsRDiff(pointer) > 5) {
      Compute.tryPost(TURNR, 1);
      delay(5);
    }

  }
  else {
    while (cmpsLDiff(pointer) > 5) {
      Compute.tryPost(TURNL, 1);
      delay(1);
    }
    Compute.keyPost(GETUP);
  }
  Compute.keyPost(GETUP);
  delay(10);
  LPING[0] = Compute.readPing(LB_PING);
  delay(5);
  LPING[1] = Compute.readPing(LE_PING);
  delay(5);
  LPING[2] = Compute.readPing(LF_PING);
  delay(5);
  RPING[0] = Compute.readPing(RB_PING);
  delay(5);
  RPING[1] = Compute.readPing(RI_PING);
  delay(5);
  RPING[2] = Compute.readPing(RF_PING);
  delay(5);

  LWALL = wallDetect2(LPING);
  RWALL = wallDetect2(RPING);

  /*  Serial.print("RESULR: ");
    Serial.print("LWALL :");
    Serial.print(LWALL);
    Serial.print("\tRWALL :");
    Serial.println(RWALL);*/
  if (LWALL < RWALL) {
    Compute.keyPost(TURNL, 16);
  }
  else {
    Compute.keyPost(TURNR, 16);
  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  delay(5);
  while (1) {
    Compute.tryPost(WALK, 1);
    if (Compute.readPing(FRD_PING) < 12) {
      Compute.keyPost(TURNR, 16);
      break;
    }
    delay(5);
    if (Compute.readPing(RF_PING) < 12) {
      Compute.keyPost(TURNR, 26);
      break;
    }
    /*
      delay(1);
      if(Compute.readPing(RI_PING) < 12){
      Compute.keyPost(TURNR, 32);
      break;
      }
    */
    delay(5);
    if (Compute.readPing(LF_PING) < 12)
      break;
    /*
      delay(1);
      if(Compute.readPing(LE_PING) < 12)
      break;
    */
    delay(5);
  }
}

void findWall2nd(void) {
  char INDEX;
  int tmp;
  float PING_MAX, PINGS[7];
  Compute.keyPost(SSPEED, S3);
  // ONE/SOME OF PING CLOSE TO WALL
  Compute.keyPost(SYNC);
  PING_MAX = 0;
  INDEX = 0;

  PINGS[0] = Compute.readPing(LB_PING);
  delay(1);
  PINGS[1] = Compute.readPing(LE_PING);
  delay(1);
  PINGS[2] = Compute.readPing(LF_PING);
  delay(1);
  PINGS[3] = Compute.readPing(FRD_PING);
  delay(1);
  PINGS[4] = Compute.readPing(RF_PING);
  delay(1);
  PINGS[5] = Compute.readPing(RI_PING);
  delay(1);
  PINGS[6] = Compute.readPing(RB_PING);

  for (int i = 0; i < 7; i++) {
    if (PINGS[i] > PING_MAX && PINGS[i] != 0xffff) {
      PING_MAX = PINGS[i];
      INDEX = i;
    }
  }
  switch (INDEX) {
    case 0:
      Compute.keyPost(TURNL, 26);
      Compute.keyPost(SYNC);
      break;

    case 1:
      Compute.keyPost(TURNL, 16);
      Compute.keyPost(SYNC);
      break;

    case 2:
      Compute.keyPost(TURNL, 10);
      Compute.keyPost(SYNC);
      break;

    case 4:
      Compute.keyPost(TURNR, 10);
      Compute.keyPost(SYNC);
      break;

    case 5:
      Compute.keyPost(TURNR, 16);
      Compute.keyPost(SYNC);
      break;

    case 6:
      Compute.keyPost(TURNR, 26);
      Compute.keyPost(SYNC);
      break;
  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);

  while (1) {
    Compute.tryPost(WALK, 1);
    //    PINGS[0] = Compute.readPing(LB_PING);
    //    delay(1);
    PINGS[1] = Compute.readPing(LE_PING);
    delay(5);
    PINGS[2] = Compute.readPing(LF_PING);
    delay(5);
    PINGS[3] = Compute.readPing(FRD_PING);
    delay(5);
    PINGS[4] = Compute.readPing(RF_PING);
    delay(5);

    Serial.print(PINGS[1]);
    Serial.print("\t");
    Serial.print(PINGS[2]);
    Serial.print("\t");
    Serial.print(PINGS[3]);
    Serial.print("\t");
    Serial.print(PINGS[4]);
    Serial.println("\t");

    //    PINGS[5] = Compute.readPing(RI_PING);
    //    delay(1);
    //    PINGS[6] = Compute.readPing(RB_PING);

    if (PINGS[IFR_PING] < 12) {
      Serial.println("FR POINT");
      Compute.keyPost(TURNR, 16);
      break;
    }
    if (PINGS[IRF_PING] < 8) {
      Serial.print("RF POINT");
      Compute.keyPost(TURNR, 26);
      break;
    }
    /*
      delay(1);
      if(Compute.readPing(RI_PING) < 12){
      Compute.keyPost(TURNR, 32);
      break;
      }
    */
    if (PINGS[ILF_PING] < 8) {
      Serial.print("LF POINT");
      break;
    }

    if (PINGS[ILE_PING] < 8) {
      Serial.print("LE POINT");
      break;
    }

  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);

}
/*
   home avoider digunakan pada saat pertamakali start
   untuk menghindari home start sebelum memulai menghitung garis
*/
void findWall(void) {
#define SCAN  32
  int i = SCAN, point, PING_MIN, cVal;




  Compute.keyPost(SSPEED, S3);
  // ONE/SOME OF PING CLOSE TO WALL
  Compute.keyPost(SYNC);
  PING_MIN = 500;

  if (Compute.readPing(LF_PING) < 12) {
    Compute.keyPost(TURNR, 8);
    Compute.keyPost(SYNC);
    return;
  }
  delay(2);
  //  if (Compute.readPing(LE_PING) < 12)
  //    return;

  //  delay(2);
  if (Compute.readPing(FRD_PING) < 12) {
    Compute.keyPost(TURNR, 16);
    Compute.keyPost(SYNC);
    return;
  }

  delay(2);
  if (Compute.readPing(RF_PING) < 12) {
    Compute.keyPost(TURNR, 32);
    Compute.keyPost(SYNC);
    return;
  }

  delay(2);
  if (Compute.readPing(RI_PING) < 12) {
    Compute.keyPost(TURNR, 32);
    Compute.keyPost(SYNC);
    return;
  }

  // IF ALL PING FAR FROM WALL
  while (i-- > 0) {
    Compute.keyPost(TURNL, 1);
    Compute.keyPost(SYNC);
    cVal = Compute.readPing(FRD_PING);
    Serial.print("Current :");
    Serial.println(cVal);
    delay(5);
    if (cVal < PING_MIN && cVal > 12) {
      if (Compute.readPing(FRD_PING) < PING_MIN) {
        PING_MIN = cVal;
        Serial.print("MAX: ");
        Serial.println(cVal);
        point = i;
      }
    }
  }

  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  Serial.print("PING MIN: ");
  Serial.println(PING_MIN);
  while (1) {
    i = Compute.readPing(FRD_PING);
    Serial.print("Seek: ");
    Serial.println(i);
    if (i < (PING_MIN + 1))
      break;

    Compute.keyPost(TURNR, 1);
    Compute.keyPost(SYNC);
    delay(5);
  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);

  while (1) {
    Compute.tryPost(WALK, 1);
    if (Compute.readPing(FRD_PING) < 12) {
      Compute.keyPost(TURNR, 16);
      break;
    }
    delay(1);
    if (Compute.readPing(RF_PING) < 12) {
      Compute.keyPost(TURNR, 26);
      break;
    }
    /*
      delay(1);
      if(Compute.readPing(RI_PING) < 12){
      Compute.keyPost(TURNR, 32);
      break;
      }
    */
    delay(1);
    if (Compute.readPing(LF_PING) < 12)
      break;
    /*
      delay(1);
      if(Compute.readPing(LE_PING) < 12)
      break;
    */
    delay(1);
  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
}

/*
   fungsi perhitungan telusur kanan
*/

void rFollow(void) {
  int prox;
  float pid;
  float pf, prf, pl, pfml, pfmr;
  char counter = 0;

  static unsigned char MSG = CLR;

  while (analogRead(PROX1) > PTRESH) {

    //    while(1){
    prf = Compute.readPing(RF_PING);
    delay(5);
    pfml = Compute.readPing(FRML_PING);
    delay(5);
    pfmr = Compute.readPing(FRMR_PING);
    delay(5);
    pid = PID(prf, setPoint);

    //       Serial.println(pf);

    /* if (pf < 15 && pfml > 20){
       delay(10);
       for (int i = 0; i < 4; i++){
         if (IRDis.distance() <= 15)
           counter++;

         delay(10);
       }

      if (counter >= 3){
        int counter = 4;
         Compute.keyPost(TURNL, 14);
         Compute.keyPost(SYNC);
         delay(5);
         while(Compute.readPing(FRML_PING) > 15 && counter-- > 0){
           Compute.keyPost(WALK, 1);
           delay(5);
         }
       }
      }*/

    if (pfml < 17 ) {
      if (pfml < 5) {
        
        if (Compute.readPing(FRML_PING) <= 5 ) {
          Compute.keyPost(TURNL, 14);
          Compute.keyPost(SYNC);
          delay(20);
          continue;
        }
        else {
          delay(20);
          continue;
        }
      }
      Compute.keyPost(TURNL, 14);
      Compute.keyPost(SYNC);
    }
    else
      Compute.keyPost(SSPEED, 1);
    Compute.keyPost(FORWARD, (pid * (-1)));
    delay(20);
//    Serial.print(digitalRead(SHIELD_L));
//    Serial.print("\t");
//    Serial.print(digitalRead(INFRA_A));
//    Serial.print("\t");
//    Serial.println(digitalRead(INFRA_B));
//    Serial.print("\t");
//    Serial.println(digitalRead(SHIELD_R));
    if (digitalRead(SHIELD_L) == LOW)
    { Compute.keyPost(SSPEED, 1);
      Compute.keyPost(SLIDR, 8);
    }
    else if (digitalRead(INFRA_A) == LOW)
    {
      Compute.keyPost(SSPEED, 3);
      Compute.keyPost(BACK, 4);
      Compute.keyPost(SSPEED, 1);
      Compute.keyPost(TURNL, 8);
      Compute.keyPost(WALK, 2);
    }
    else if (digitalRead(INFRA_B) == LOW)
    {
      Compute.keyPost(SSPEED, 3);
      Compute.keyPost(BACK, 4);
      Compute.keyPost(SSPEED, 1);
      Compute.keyPost(TURNR, 8);
      Compute.keyPost(WALK, 2);
    }
   else if (digitalRead(SHIELD_R) == LOW)
    {
      Compute.keyPost(SSPEED, 1);
      Compute.keyPost(SLIDL, 8);
    }
        else if (digitalRead(INFRA_C) == LOW)
    {
          Compute.keyPost(TURNL, 14);
          Compute.keyPost(SYNC);
    }
    delay(20);
    Compute.keyPost(SYNC);
  }
}

void lFollow(void) {
  int prox;
  float pid;
  float pf, plf, pl, pfml;
  char counter = 0;

  static unsigned char MSG = CLR;

  while (analogRead(PROX1) > PTRESH) {
    //    while(1){
    plf = Compute.readPing(LF_PING);
    delay(5);
    pfml = Compute.readPing(FRML_PING);
    delay(5);
    pid = PID(plf, setPoint);
    Serial.print(plf);
    Serial.print("\t");
    Serial.println(pfml);
    

    /* if (pf < 15 && pfml > 20){
       delay(10);
       for (int i = 0; i < 4; i++){
         if (IRDis.distance() <= 15)
           counter++;

         delay(10);
       }

       if (counter >= 3){
         int counter = 4;
          Compute.keyPost(TURNR, 14);
          Compute.keyPost(SYNC);
          delay(5);
          while(Compute.readPing(FRML_PING) > 15 && counter-- > 0){
            Compute.keyPost(WALK, 1);
            delay(5);
          }
        }
      }*/

    if (pfml < 17 ) {
      if (pfml < 5) {
        if (Compute.readPing(FRML_PING) <= 5) {
          Compute.keyPost(TURNR, 14);
          Compute.keyPost(SYNC);
          delay(20);
          continue;
        }
        else {
          delay(20);
          continue;
        }
      }
      Compute.keyPost(TURNR, 14);
      Compute.keyPost(SYNC);
    }
    else
      Compute.keyPost(SSPEED, 1);
    Compute.keyPost(FORWARD, pid);
    delay(20);
//    Serial.print(digitalRead(SHIELD_L));
//    Serial.print("\t");
//    Serial.print(digitalRead(INFRA_A));
//    Serial.print("\t");
//    Serial.println(digitalRead(INFRA_B));
//    Serial.print("\t");
//    Serial.println(digitalRead(SHIELD_R));
    if (digitalRead(SHIELD_L) == LOW)
    { Compute.keyPost(SSPEED, 1);
      Compute.keyPost(SLIDR, 6);
      Compute.keyPost(SSPEED, 3);
      Compute.keyPost(STAND);
    }
    else if (digitalRead(INFRA_A) == LOW)
    {
      Compute.keyPost(SSPEED, 3);
      Compute.keyPost(BACK, 4);
      Compute.keyPost(SSPEED, 1);
      Compute.keyPost(TURNL, 8);
      Compute.keyPost(WALK, 2);
    }
    else if (digitalRead(INFRA_B) == LOW)
    {
      //Compute.keyPost(SSPEED,3);
      Compute.keyPost(SSPEED, 3);
      Compute.keyPost(BACK, 4);
      Compute.keyPost(SSPEED, 1);
      Compute.keyPost(TURNR, 8);
      Compute.keyPost(WALK, 2);
    }
   else if (digitalRead(SHIELD_R) == LOW)
    { Compute.keyPost(SSPEED, 3);
      Compute.keyPost(SLIDL, 6);
      Compute.keyPost(SSPEED, 3);
      Compute.keyPost(STAND);
    }
    else if (digitalRead(INFRA_C) == LOW)
    {
          Compute.keyPost(TURNR, 14);
          Compute.keyPost(SYNC);
    }
    delay(20);
    Compute.keyPost(SYNC);
  }
}

/*
   fungsi scan api pada ruangan
   jika terdapat api maka fungsi akan mengembalikan nilau TRUE
*/
bool scanRoom2(void) {

  /*
    Cek apakah robot masuk ruang dalam keadaan miring
      BUG yang mungkin akan muncul
        1. Kemiringan robot terdapat ditengah sehingga sulit untuk dilususkan walaupun dengan kompas
        2. Optimalisasi dengan orientasi PING))) juga akan terjadi kesalahan
      Cara mengatasi
        1. Optimalisasi PID untuk mengkondisikan posisi robot
        2. Cara 1 dan 2 pada BUG tetap harus dicoba
  */

  int nturn = 0, turnx = 30, counter = 5;

  Compute.keyPost(WALK,9);
  Compute.keyPost(GETUP);

  // Compute.keyPost(TURNL, 10);

  Compute.keyPost(SYNC);

  while (counter-- > 0) {
    if (readUV() == true) {
      return true;
    }
    delay(1);
  }

  /*
    while(turnx-- > 0){
    if (readUV() == true){
      Compute.keyPost(GETUP);
      Compute.keyPost(SYNC);
      return nturn;
    }
    Compute.keyPost(TURNR, 1);
    nturn++;
    }
  */
  Compute.keyPost(TURNL, 24);
  Compute.keyPost(GETUP);
  delay(500);
  Serial.print(Compute.readPing(FRMR_PING));
  if(Compute.readPing(FRMR_PING)<10){
    Compute.keyPost(TURNL,4);
    }
  Compute.keyPost(SYNC);
  while (analogRead(PROX1) > PTRESH)
    Compute.tryPost(WALK, 2);
  delay(100);
  lineVoidL();
  Compute.keyPost(SYNC);

  return false;
}

/*
  bool scanRoom(void){
  /*
    Cek apakah robot masuk ruang dalam keadaan miring
      BUG yang mungkin akan muncul
        1. Kemiringan robot terdapat ditengah sehingga sulit untuk dilususkan walaupun dengan kompas
        2. Optimalisasi dengan orientasi PING))) juga akan terjadi kesalahan
      Cara mengatasi
        1. Optimalisasi PID untuk mengkondisikan posisi robot
        2. Cara 1 dan 2 pada BUG tetap harus dicoba

  int prox;
  float pid;
  float pf, plf, pl;

  while(analogRead(PROX1) > PTRESHB){

    if (fireScan() == true)
      return true;

      plf = Compute.readPing(LF_PING);
      delay(5);
      pf = Compute.readPing(FR_PING);
      pid = PID(plf);

      if (pf < 15 ){
        Compute.keyPost(TURNR, 14);
        Compute.keyPost(SYNC);
      }
      else
        Compute.tryPost(FORWARD, pid);
      delay(10);
  }

  Compute.keyPost(WALK, 2);
  Compute.keyPost(SYNC);

  return false;
  }
*/

bool firing() {
#define R1  11.25     // 20 derajat dynamixel
#define R2  5.625     // 10 derajat dynamixel
#define R3  2.8125    // 5 derajat dynamixel

  char counter = 0;
  int
  s1, // scan kanan
  s2, // scan kiri
  x,  // putaran 20 derajat
  x1, // putaran 10 derajat
  x2; // putaran 5 derajat

  float
 //  fPos,   // flame position
  a,      // sudut awal
  a1,     // sisa sudut dari putaran 20
  a2;     // sisa sudut dari putaran 10

RSCAN:
  s1 = rightScan();
  s2 = leftScan();
  fPos = (s1 + s2) / 2.0;
  Serial.print(fPos);
  Serial.print("\t");

  fPos = fPos - COS;
  a = abs(fPos);
  /*
    Serial.println("DATA");
    Serial.println(fPos);
    Serial.println(a);
  */
  // No Fire Detected
  if (s1 == 0 || s2 == 0) {
    Compute.keyPost(GETUP);
    if (counter++ > 3)
      return false;
    goto RSCAN;
  }

  x = a / R1;
  a1 = a - (x * R1);
  x1 = a1 / R2;
  a2 = a1 - (x1 * R2);
  x2 = a2 / R3;

  //  if (x != 0)
  x = (x * 2) ;
  //  if (x1 != 0)
  x1 = (x1 * 2);
  //  if (x2 != 0)
  x2 = (x2 * 2);
  
 /*   Serial.print(x);
    Serial.print("\t");
    Serial.print(x1);
    Serial.print("\t");
    Serial.print(x2);
    Serial.print("\t");
  */
  if (fPos > 0) { //jika lebih dari 0 maka lilin berada di kanan robot
    if (x != 0) {
      Compute.keyPost(TURNL, x);
      Serial.println("kiri");
    }
    if (x1 != 0) {
      for (int i = 0; i < x1; i++)
        Compute.keyPost(TURNLA, 10);
      Serial.println("kiri1");
    }
    if (x2 != 0) {
      for (int i = 0; i < x2; i++)
        Compute.keyPost(TURNLA, 5);
      Serial.println("kiri2");
    }
    scanner.write(COS);
    //delay(1000);
  }
  else if (fPos < 0) { // jika kurang dari 0 maka lilin berada di kiri robot
    if (x != 0)
      Compute.keyPost(TURNR, x);
     Serial.println("kanan");
    if (x1 != 0)
      for (int i = 0; i < x1; i++)
        Compute.keyPost(TURNRA, 10);
        Serial.println("kanan1");

    if (x2 != 0)
      for (int i = 0; i < x2; i++)
        Compute.keyPost(TURNRA, 5);
    Compute.keyPost(TURNRA, 20);
    Serial.println("kanan2");

    scanner.write(COS);
    Serial.println(COS);
    //delay(1000);
  }
  return true;
}

/*
   fungsi untuk pemadaman api jika dari hasil scanroom terditeksi api
*/

void firePoint() {
  int pos = LLSCAN, tpa_max = 0, val, lookAt = 0, counter = 0;
  scanner.write(pos);
  delay(200);

  // get max point
  while (pos++ < RLSCAN) {
    scanner.write(pos);
    delay(5);
    for (int i = 0; i < 8; i++) {
      delay(1);
      val = Compute.tpaPixel(i);
      if (val > tpa_max) {
        for (int j = 0; j < 3; j++) {
          if (val > (Compute.tpaPixel(i) - 10) && val < (Compute.tpaPixel(i) + 10))
            counter++;
          delay(1);
        }
        if (counter > 2) {
          tpa_max = val;
          counter = 0;
        }
      }
      counter = 0;
    }
  }
  // get pos of max point
  while (pos-- > LLSCAN) {
    scanner.write(pos);
    delay(5);
    for (int i = 0; i < 8; i++) {
      delay(1);
      val = Compute.tpaPixel(i);
      if (val > (tpa_max - 10))
        break;  // close from current loop
    }
    if (val > (tpa_max - 10))
      break;
  }
  scanner.write(COS);
  delay(500);
  // pointing to fire
  if (pos > 95) { // if fire on right pos
    lookAt = 0;
    while (lookAt++ < MAP(30)) {
      for (int i = 0; i < 8; i++) {
        delay(2);
        val = Compute.tpaPixel(i);
        if (val > (tpa_max - 10))
          break;  // close from current loop
      }
      if (val > (tpa_max - 10))
        break;
        Compute.keyPost(LOOK_AT, (lookAt * (-1)));
      delay(1);
    }
  }
  else if (pos < 85) { // fire on left pos
    lookAt = 0;
    while (lookAt++ < MAP(30)) {
      for (int i = 0; i < 8; i++) {
        delay(2);
        val = Compute.tpaPixel(i);
        if (val > (tpa_max - 5))
          break;  // close from current loop
      }
      if (val > (tpa_max - 5))
        break;
      Compute.keyPost(LOOK_AT, lookAt);
      delay(1);
    }
  }
}
float PIDC(int spc) {
 float pvc= Compute.readCMPS();
  float error, P, D;
  error = spc - pvc;
  P = KP * error;
  D = ((KD * 10.0) / TS) * (error - lastErr);
  lastErr = error;
  //Serial.println(P+D);
  return (P + D);
}

bool fireEx(void) {
 int tmp;
 float CS,scan,pidc;
  Compute.keyPost(GETUP);
  Compute.keyPost(SSPEED,3);
  if (firing() == false)      
      return false;
      CS = Compute.readCMPS();
   while (Compute.readPing(FRD_PING)>20)
         {
         pidc = PIDC(CS);
         Compute.keyPost(SSPEED,2);
         Compute.tryPost(FORWARD,pidc);
         delay(10);
         if(FRD_PING<10)
           break;
           else
             Compute.tryPost(FORWARD,pidc);
           }         
    if (firing() == false)
    return false;
    Compute.keyPost(GETUP);
    Compute.keyPost(SYNC);
    firePoint();
    Compute.keyPost(GETUP);
    Compute.keyPost(SYNC);
    brush();
    Compute.keyPost(SYNC);


  return true;


}

/*

  // Compute.keyPost(GETUP);
  // Compute.keyPost(SYNC);

  //
  //  lastErr = 0;
  //  while(TPA_mon() != true && Compute.readPing(FRD_PING) > 20);
  //  lastErr = 0;
  //  Compute.keyPost(MANUAL);
  //
  //  Compute.keyPost(GETUP);
  //  if (firing() == false)
  //    return false;
  //
  //  Compute.keyPost(GETUP);
  //  Compute.keyPost(SYNC);
  // if (Compute.readPing(FRD_PING) > 15 && Compute.readPing(FRML_PING) > 15){

  //    Compute.keyPost(GETUP);
  //    Compute.keyPost(SYNC);
  //    if ( firing() == false)
  //      return false;
  //
  //    Compute.keyPost(GETUP);
  //    Compute.keyPost(SYNC);
  //  firePoint();
  //  Compute.keyPost(GETUP);
  //  Compute.keyPost(SYNC);

  Compute.keyPost(SYNC);

  return true;

  /*  while(1);

  /*
  s1 = rightScan();
  s2 = leftScan();

  fPos = (s1 + s2) / 2.0;
  scanner.write(fPos);

  tmp = 0;
  while(tmp++ < 15){
  Compute.keyPost(EXMODE, tmp);
  delay(50);
  }

  while(1);

  while(Compute.readPing(FR_PING) > 20){
  Compute.tryPost(WALK, 1);
  delay(5);
  }
  while(1);
*/


bool fireEx2(void) {
  int tmp;
  Compute.keyPost(SSPEED, 100);
  Compute.keyPost(GETUP);
  //  delay(1000);
   
  if (firing() == false)
    return false;

  //  delay(1000);

  Compute.keyPost(GETUP);
  if (TPA_level2() == false) {
    Compute.keyPost(AUTO_PILOT);
    Compute.keyPost(SYNC);
    delay(100);

    lastErr = 0;
    while (TPA_mon() != true && Compute.readPing(FRD_PING) > 20);
    lastErr = 0;
    Compute.keyPost(MANUAL);

    Compute.keyPost(GETUP);
  }
  if (firing() == false)
    return false;

  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  firePoint();
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  while(Compute.readPing(FRD_PING)>10)
        {
          Compute.keyPost(WALK,1);
          if(Compute.readPing(FRD_PING)<10)
            break;
          }
  //  delay(3000);
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  brush();
  Compute.keyPost(SYNC);

  return true;
  //  while(1);

  /*
    s1 = rightScan();
    s2 = leftScan();

    fPos = (s1 + s2) / 2.0;
    scanner.write(fPos);

    tmp = 0;
    while(tmp++ < 15){
      Compute.keyPost(EXMODE, tmp);
      delay(50);
    }

    while(1);

    while(Compute.readPing(FR_PING) > 20){
      Compute.tryPost(WALK, 1);
      delay(5);
    }
    while(1);
  */
}

/*
  [DEPRECATED]
  bool fireScan(void){
  return (analogRead(IRSENSE1) > tFire[0] || analogRead(IRSENSE2) > tFire[1] ||\
    analogRead(IRSENSE3) > tFire[2] || analogRead(IRSENSE4) > tFire[3] || \
    analogRead(IRSENSE5) > tFire[4]);
  }
*/

// baca byte pada TPA dan return true jika
// salah satu atau semua pixel TPA lebih dari
// ambient
bool TPA_scan(void) {
  int i, ambi, pv, counter = 0;

  ambi = Compute.tpaAmbi();

  for (i = 0; i < 8; i++) {
    pv = Compute.tpaPixel(i);
    if (abs(pv - ambi) > 10) {
      for (int j = 0; j < 5; j++) {
        if (abs(Compute.tpaPixel(i) - ambi) > 10)
          counter++;
        delay(2);
      }
      if (counter > 3) {
        return true;
      }
      else {
        counter = 0;
      }
    }
  }
  return false;
}

bool TPA_scan2(void) {
  int i, ambi, pv, counter = 0;

  ambi = Compute.tpaAmbi();

  for (i = 0; i < 8; i++) {
    pv = Compute.tpaPixel(i);
    if (pv > 70) {
      counter++;
    }
  }
  if (counter >= 2)
    return true;

  return false;
}

int rightScan(void) {
  int pos = LLSCAN, val;
  scanner.write(pos);
  delay(500);
  while (pos++ < RLSCAN) {
    scanner.write(pos);
    delay(10);
    if (TPA_scan() == true) {
      Serial.print(val);
      Serial.print("\t");
      Serial.println(pos);
      return pos;
    }
  }
  return 0;
}

int leftScan(void) {
  int pos = RLSCAN, val;
  scanner.write(pos);
  delay(500);
  while (pos-- > LLSCAN) {
    scanner.write(pos);
    delay(10);
    if (TPA_scan() == true) {
      Serial.print(val);
      Serial.print("\t");
      Serial.println(pos);
      return pos;
    }
    //Serial.println(val);
  }
  return 0;
}

bool TPA_mon(void) {
  posMon = RLSCAN;
  scanner.write(posMon);
  delay(500);

  while (posMon-- > LLSCAN) {
    scanner.write(posMon);
    delay(5);
    if (TPA_level() == true) {
      return true;
    }
  }
  posMon = RLSCAN;
  scanner.write(posMon);
  return false;
}

// fungsi pembantu TPA MON yang mengatur  PID
// pergerakan sesuai dengan posisi lilin (TRACKING)
// NEED TO UPGRADE
bool TPA_level2(void) {
  int pos = LLSCAN, val;
  scanner.write(pos);
  delay(500);
  while (pos++ < RLSCAN) {
    scanner.write(pos);
    delay(10);
    if (TPA_scan2() == true) {
      return true;
    }
  }
}
bool TPA_level(void) {
  int i, ambi, stpa, pv, counter = 0;
  bool dfire = false;
  float t_pid;
  char pixel = 0;

  ambi = Compute.tpaAmbi();
  //  stpa = Compute.tpaPixel(4);

  for (i = 0; i < 8; i++) {
    pv = Compute.tpaPixel(i);
    if (abs(pv - ambi) > 10)
      dfire = true;

    if (pv > 80) {

      for (int j = 0; j < 3; j++) {
        if (Compute.tpaPixel(i) > 80)
          counter++;
        delay(2);
      }
      if (counter > 2) {
        pixel++;
      }
      else {
        counter = 0;
      }
    }
  }
  if (dfire == true) {
    t_pid = PID(map(posMon, 30, 150, (setPoint - 30), (setPoint + 30)), setPoint);
    Compute.tryPost(FORWARD, t_pid);
  }
  if (pixel >= 2)
    return true;
  else
    return false;
}

bool TPA_Travel(void) {
  if (ctrlTravel == RIGHT) {
    posTravel++;
    scanner.write(posTravel);
    delay(5);
    if (TPA_scan2() == true)
      return true;

    if (posTravel == RTLSCAN)
      ctrlTravel = LEFT;
  }
  else if (ctrlTravel == LEFT) {
    posTravel--;
    scanner.write(posTravel);
    delay(1);
    if (TPA_scan2() == true)
      return true;

    if (posTravel == LTLSCAN)
      ctrlTravel = RIGHT;
  }
  return false;
}

bool gateLine(void) {
  if (analogRead(PROX1) == false && analogRead(PROX2) == true)
    return true;
  else
    return false;
}

int cmpsLDiff(int mark) {
  int cVal, res;
  cVal = (int)Compute.readCMPS();

  if (cVal < mark)
    res = (360 - mark) + cVal;
  else
    res = cVal - mark;

  return res;
}

int cmpsRDiff(int mark) {
  int cVal, res;
  cVal = Compute.readCMPS();

  if (cVal > mark)
    res = (360 - cVal) + mark;
  else
    res = mark - cVal;

  return res;
}

int wallDetect2(int *ptr) {
  int min;

  if (abs(ptr[0] - ptr[1]) < 10 && abs(ptr[0] - ptr[2]) < 10) {
    return ptr[1];
  }

  return 0xff;
}

int wallDetect(int *ptr) {
  int min;

  if (abs(ptr[0] - ptr[1]) < 10 && abs(ptr[0] - ptr[2]) < 10) {
    if (ptr[1] < ptr[0] && ptr[1] < ptr[2])
      return ptr[1];
  }

  return 0xff;
}

bool degDev(int src, int dst) {
  int pka, pki; // putar kiri, putar kanan

  pka = src - dst;
  if (pka < 0)
    pka = src + (360 - dst);
  pki = dst - src;
  if (pki < 0)
    pki = dst + (360 - src);

  if (pka < pki)
    return RIGHT;
  else
    return LEFT;
}

bool readUV(void) {
  int res;
  res = analogRead(UVTRON);

  if (res > UVTRESH)
    return true;
  else
    return false;
}

// USE COMPASS
bool tpaFire(void) {
  Compute.keyPost(SSPEED,3);
  Compute.keyPost(GETUP);
  Compute.keyPost(TURNL, 6);
  Compute.keyPost(SYNC);

  int nturn = 38;
  while (nturn > 0) {
    if (TPA_scan() == true)

      return true;

    if (Compute.tryPost(TURNR, 1) == true)
      nturn--;

    delay(1);
  }
  return false;
}

/*
  void chckHead(void){
  int DEG[4] = {234, 123, 75, 20};
  int cVal;
  char kuadran;

  cVal = Compute.readCMPS();
  if (cVal > 234 || cVal =< 20)
    kuadran = 1;
  else if (cVal > 123 && cVal =< 234)
    kuadran = 2;
  else if (cVal > 75 && cVal =< 123)
    kuadran = 3;
  else if (cVal > 20 && cVal =< 75)
    kuadran = 4;


  }
*/

void exitRoom(void) {
  Compute.keyPost(GETUP);
  delay(500);
  Compute.keyPost(TURNL, 18);
  Compute.keyPost(SYNC);
  while (1) {
    Compute.keyPost(WALK, 2);
    if (Compute.readPing(FRD_PING) < 12) {
      Compute.keyPost(TURNL, 16);
      Compute.keyPost(SYNC);
      break;
    }
    delay(5);
    if (Compute.readPing(RF_PING) < 12) {
      Compute.keyPost(TURNL, 4);
      Compute.keyPost(SYNC);
      break;
    }
    /*
      delay(1);
      if(Compute.readPing(RI_PING) < 12){
      Compute.keyPost(TURNR, 32);
      break;
      }
    */
    delay(5);
    if (Compute.readPing(LF_PING) < 12) {
      Compute.keyPost(TURNL, 32);
      Compute.keyPost(SYNC);
      break;
    }
    /*
      delay(1);
      if(Compute.readPing(LE_PING) < 12)
      break;
    */
    delay(5);
  }
  Compute.keyPost(GETUP);
  Compute.keyPost(SYNC);
  delay(5);
  int prox;
  float pid;
  float pf, prf, pl;

  static unsigned char MSG = CLR;
  Compute.keyPost(SSPEED, S2);
  Compute.keyPost(SYNC);

  while (analogRead(PROX1) > PTRESH) {

    //        while(1){
    prf = Compute.readPing(RF_PING);
    delay(5);
    pf = Compute.readPing(FRD_PING);
    pid = PID(prf, 12);

    if (pf < 10 ) {
      Compute.keyPost(TURNL, 14);
      Compute.keyPost(SYNC);
    }
    else
      Compute.keyPost(FORWARD, (pid * (-1)));
    delay(5);
  }
  //    Compute.keyPost(WALK, 2);
  Compute.keyPost(SYNC);


}

void goHome() {
  Compute.keyPost(SSPEED, S2);
  if (roomCounter <= 2) {
    lineVoidR();
    while (roomCounter-- > 0) {
      rFollow();
      if (roomCounter == 0)
        break;

      Compute.keyPost(WALK, 2);
      Compute.keyPost(GETUP);

      //  Compute.keyPost(TURNL, 10);

      Compute.keyPost(SYNC);

      /*
        while(turnx-- > 0){
          if (readUV() == true){
            Compute.keyPost(GETUP);
            Compute.keyPost(SYNC);
            return nturn;
          }
          Compute.keyPost(TURNR, 1);
          nturn++;
        }
      */
      Compute.keyPost(TURNL, 22);
      Compute.keyPost(SYNC);
      while (analogRead(PROX1) > PTRESH)
        Compute.tryPost(WALK, 1);

      lineVoidR();
    }
  }
  else {
    lineVoidL();
    while (roomCounter++ < 5) {
      //      Compute.keyPost(SSPEED, S2);
      lFollow();
      if (roomCounter == 5)
        break;

      Compute.keyPost(WALK, 2);
      Compute.keyPost(GETUP);

      //  Compute.keyPost(TURNL, 10);

      Compute.keyPost(SYNC);

      /*
        while(turnx-- > 0){
          if (readUV() == true){
            Compute.keyPost(GETUP);
            Compute.keyPost(SYNC);
            return nturn;
          }
          Compute.keyPost(TURNR, 1);
          nturn++;
        }
      */
      Compute.keyPost(TURNL, 22);
      Compute.keyPost(SYNC);
      while (analogRead(PROX1) > PTRESH)
        Compute.tryPost(WALK, 1);

      lineVoidL();
    }
  }
  Compute.keyPost(SSPEED, S3);
  Compute.keyPost(WALK, 6);
  Compute.keyPost(SYNC);
  Compute.keyPost(GETUP);
  while (1);
}

void lineVoidR(void) {
  int counter = 10;
  float pid, prf;

  /*
    if(Compute.readPing(RF_PING) > 20)
      counter = 40;
    else
      counter = 15;
  */

  while (counter-- > 0) {
    if (Compute.readPing(FRD_PING) < 15) {
      delay(10);
      if (Compute.readPing(FRD_PING) <= 15)
        break;
    }
    prf = Compute.readPing(RF_PING);


    pid = PID(prf, setPoint);

    Compute.keyPost(FORWARD, (pid * (-1)));
    delay(10);
  }
  delay(10);
  Compute.keyPost(SYNC);
}

void lineVoidL(void) {
  int counter = 10;
  float plf, pid;

  while (counter-- > 0) {
    if (Compute.readPing(FRD_PING) < 15) {
      delay(10);
      if (Compute.readPing(FRD_PING) <= 15)
        break;
    }

    plf = Compute.readPing(LF_PING);


    pid = PID(plf, setPoint);

    Compute.keyPost(FORWARD, pid);
    delay(10);
  }
  delay(10);
  Compute.keyPost(SYNC);
}

void pingFire(void) {
  while (1) {
    Compute.keyPost(WALK, 1);
    if (Compute.readPing(FRD_PING) < 10 || Compute.readPing(FRMR_PING) < 10 || Compute.readPing(FRML_PING) < 10 )
    {
      Compute.keyPost(GETUP);
      break;
    }
  }
}

void standBy(void) {
  int valFreq;
  while ((valFreq = analogRead(FSENS)) < FTRESH && digitalRead(STARTB) == HIGH)
    delay(1);

  digitalWrite(SLED, HIGH);
  delay(2000);
  digitalWrite(SLED, LOW);
}
void brush() {
  Compute.keyPost(LOOK_AT, 0);
  Compute.keyPost(FIREON);
  delay(1000);
  Compute.keyPost(LOOK_AT, -90);
  Compute.keyPost(FIREON);
  delay(500);
  Compute.keyPost(LOOK_AT, 90);
  Compute.keyPost(FIREON);
  delay(500);
  Compute.keyPost(LOOK_AT, 0);
  Compute.keyPost(FIREON);
  delay(2000);
  Compute.keyPost(GETUP);
  Compute.keyPost(FIREOFF);
}
void run() {
  float frd,frml,frmr;
  while (analogRead(PROX1) > PTRESH){
        
         frd = Compute.readPing(FRD_PING);
         delay(5);
         frml = Compute.readPing(FRML_PING);
         delay(5);        
         frmr = Compute.readPing(FRMR_PING);
         delay(5); 
      if (frd <=16 || frml <=16 || frmr <=16){
              Compute.keyPost(SSPEED, 3);
              Compute.keyPost(GETUP);
              tpaFire();
              firePoint();
              Compute.keyPost(GETUP);
              Compute.keyPost(WALK,2);
              Compute.keyPost(GETUP);
              //firePoint();
              delay(1000);
              brush();
              break;
          }
          else
          {
            Compute.keyPost(SSPEED,3);
            Compute.keyPost(WALK,1);
            }
          }
     }



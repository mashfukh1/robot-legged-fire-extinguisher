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

#include "LycosidCompute.h"
#include <HMC5883L.h>

HMC5883L compass;

void LycosidComputeClass::begin(long baud){
  Serial3.begin(baud);
}
void LycosidComputeClass::initCMPS(void){
  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}
float LycosidComputeClass::readPing(char pingPIN){
  // variabel untuk menyimpan hasil perhitungan
  long usecond;
  float distance;
  // Perhitungan OBJEK DEPAN
  // aktifkan mode transmiter
  pinMode(pingPIN, OUTPUT);
  // matikan pin untuk memastikan tidak ada gelombang yang keluar
  digitalWrite(pingPIN, LOW);
  delayMicroseconds(2);
  //  produksi gelombang selama 5 mikro detik
  digitalWrite(pingPIN, HIGH);
  delayMicroseconds(5);
  // hentikan gelombang
  digitalWrite(pingPIN, LOW);

  // aktifkan receiver
  pinMode(pingPIN, INPUT);
  // aktifkan mode receiver dan tangkap gelombang
  //  pulseIn akan menghitung waktu (dalam satuan mikro detik) perjalanan
  // gelombang dari keluar, mantul, dan masuk ke reciver
  usecond = pulseIn(pingPIN, HIGH, 500000);

  // PING menggunakan gelombang ultrasonik (bunyi) sedangkan kecepatan bunyi diudara adalah
  // v = 340 m/s
  // dengan diketahui waktu perjalanan gelombang maka dapat dicari jarak objek dimana gelombang memantul
  // dengan rumus
  // v = s / t
  //    v = kecepatan
  //    s = jarak
  //    t = waktu

  // sehingga jarak dihitung (dibagi 2 karena gelombang PP)
  if (usecond != 0)
    distance = (usecond / 29.0) / 2.0;
  else
    distance = 0xffff;

  // kembalikan hasil perhitungan
  return distance;
}

float LycosidComputeClass::readCMPS(void){
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI;

  return headingDegrees;
}

bool LycosidComputeClass::tryPost(const char cmd){
  char MSG = CLR;
  String data;

  data = cmd;
  if (Serial3.available()){
    MSG = Serial3.read();
    (void)Serial3.read();
  }

  if (MSG == GET){
    Serial3.print(data);
    return true;
  }
  else
    return false;
}

bool LycosidComputeClass::tryPost(const char cmd, const int param){
  char MSG = CLR;
  String data;

  data = cmd;
  data += param;

  if (Serial3.available()){
    MSG = Serial3.read();
    (void)Serial3.read();
  }

  if (MSG == GET){
    Serial3.print(data);
    return true;
  }
  else
    return false;
}

bool LycosidComputeClass::tryPost(const char cmd, const float param){
  char MSG = CLR;
  String data;

  data = cmd;
  data += param;

  if (Serial3.available()){
    MSG = Serial3.read();
    (void)Serial3.read();
  }

  if (MSG == GET){
    Serial3.print(data);
    return true;
  }
  else
    return false;
}

void LycosidComputeClass::keyPost(const char cmd){
  while(tryPost(cmd) == false)
    delay(1);
}

void LycosidComputeClass::keyPost(const char cmd, int param){
  while(tryPost(cmd, param) == false)
    delay(1);
}

void LycosidComputeClass::keyPost(const char cmd, float param){
  while(tryPost(cmd, param) == false)
    delay(1);
}

LycosidComputeClass Compute;

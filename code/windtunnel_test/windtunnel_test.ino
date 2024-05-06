/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include <PWMServo.h>
#include "ms4525do.h"


PWMServo motor1;
/* 
* An MS4525DO object
*/
bfs::Ms4525do pres;

float presure_data;
float temprature_data;
double P;
double PR;
double TR;
double V;
double VV;

int value = 5;
int count = 100;
void setup() {
  /* Serial to display data */
  Serial.begin(9600);
  motor1.attach(2, 900, 2100);
  motor1.write(0);
  delay(5000);
  //esc_calibration();  // only one time

  while (!Serial) {}
  Wire.begin();
  Wire.setClock(400000);
  /* 
  * I2C address of 0x28, on bus 0, with a -1 to +1 PSI range
  */
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while (1) {}
  }
}

void loop() {
  /* Read the sensor */
  get_sensor_data();
  calc_speed(presure_data, temprature_data);
  if (value <= 90 and count >= 100) {
    value++;
    count = 0;
    motor1.write(value);
  }
  count++;
  delay(10);
}

void get_sensor_data() {
  if (pres.Read()) {
    /* Display the data */
    presure_data = pres.pres_pa();
    presure_data = abs(presure_data);
    temprature_data = pres.die_temp_c();
/*
    Serial.print("pressure: ");
    Serial.print(presure_data, 6);
    Serial.print("\t");
    Serial.print("temprature: ");
    Serial.print(temprature_data, 6);
    Serial.print("\n");
    */
  }
}
void calc_speed(float P_dat, float T_dat) {
  PR = (double)((P_dat - 819.15) / (14744.7));
  PR = (PR - 0.49060678);
  PR = abs(PR);
  V = ((PR * 13789.5144) / 1.225);
  VV = (sqrt((V)));
  //VV -= 81.11070;

  Serial.println("-------------------------------------");
  Serial.print("raw Pressure:");
  Serial.println(P_dat);
  //Serial.println(P_dat,DEC);
  //Serial.println(P_dat,BIN);
  Serial.print("pressure psi:");
  Serial.println(PR, 10);
  Serial.print(" ");
  Serial.print("Temp:");
  Serial.println(T_dat);
  Serial.print("speed m/s :");
  Serial.println(VV, 5);
}


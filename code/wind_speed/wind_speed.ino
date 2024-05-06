

#include <Wire.h>  //I2C library 0x28H
#include <PWMServo.h>

byte fetch_pressure(unsigned int *p_Pressure);  //convert value to byte data type
PWMServo motor1;

byte _status;
unsigned int P_dat;
unsigned int T_dat;
double PR;
double TR;
double V;
double VV;
double VV_km;

int value = 5;
int count = 10;


#define TRUE 1
#define FALSE 0

void setup(void) {
  Serial.begin(9600);
  motor1.attach(2, 900, 2100);
  motor1.write(0);
  //esc_calibration();  // only one time
  delay(5000);
  Wire.begin();
  delay(500);
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");  // just to be sure things are working
}

void loop() {
  proces_sensor_info();
 

void proces_sensor_info() {
  _status = fetch_pressure(&P_dat, &T_dat);

  switch (_status) {
    case 0:
      Serial.print("Ok ");
      break;
    case 1:
      Serial.print("Busy");
      break;
    case 2:
      Serial.print("Slate");
      break;
    default:
      Serial.print("Error");
      break;
  }


  PR = (double)((P_dat - 819.15) / (14744.7));
  PR = (PR - 0.49060678);
  PR = abs(PR);
  V = ((PR * 13789.5144) / 1.225);
  VV = (sqrt((V)));
  VV -= 17.30;
  VV_km = VV * 3, 6;

  TR = (double)((T_dat * 0.09770395701));
  TR = TR - 50;


  Serial.print("\t");
  Serial.print("raw Pressure:");
  Serial.print(P_dat);
  Serial.print("\t");
  //Serial.println(P_dat,DEC);
  //Serial.println(P_dat,BIN);
 // Serial.print("pressure psi:");
  //Serial.print(PR, 10);
  //Serial.print("\t");
  //Serial.print("raw Temp:");
  //Serial.print(T_dat);
  Serial.print("\t");
  Serial.print("temp:");
  Serial.print(TR);
  Serial.print("\t");
  Serial.print("speed m/s :");
  Serial.print(VV, 5);
  Serial.print("\t");
  Serial.print("speed km/u :");
  Serial.print(VV_km, 5);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\n");
  
}

byte fetch_pressure(unsigned int *p_P_dat, unsigned int *p_T_dat) {


  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;

  address = 0x28;
  Wire.beginTransmission(address);
  Wire.endTransmission();
  delay(100);

  Wire.requestFrom((int)address, (int)4);  //Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte Temp_L = Wire.read();
  Wire.endTransmission();


  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;

  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;
  *p_T_dat = T_dat;
  return (_status);
}
void esc_calibration() {
  motor1.write(180);  // set to 2000us puls length
  while (!Serial) { delay(10); }
  Serial.println("max positon");
  delay(1000);

  motor1.write(0);  //set to 1000us puls length
  Serial.println("low position");
  delay(3000);
}
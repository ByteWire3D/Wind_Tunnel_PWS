#include <ESP32Servo.h>

int angleOUT_pin = 20;
Servo angle_out;

void setup() {
  Serial.begin(115200);
  pinMode(21, INPUT);
  // ESP32PWM::allocateTimer(0);  // set timers for the pwm signals
  // ESP32PWM::allocateTimer(1);
  ////  ESP32PWM::allocateTimer(2);
  //  ESP32PWM::allocateTimer(3);
  //  angle_out.setPeriodHertz(50);  // standard 50 hz esc signal
  //  angle_out.attach(angleOUT_pin, 1000, 2000);
}

void loop() {
  unsigned long pulseWidth = pulseIn(21, HIGH);
  // pulseWidth = constrain(pulseWidth, 1000, 2000);
  float angle = map(pulseWidth, 100, 500, 0, 4500);

  unsigned long input_pulse = map(angle, 0, 4500, 500, 2500);
  angle /= 100;
  Serial.print(pulseWidth);
  Serial.print("\t");
  Serial.print(input_pulse);
  Serial.print("\t");
  Serial.println(angle);
  delay(100);
}

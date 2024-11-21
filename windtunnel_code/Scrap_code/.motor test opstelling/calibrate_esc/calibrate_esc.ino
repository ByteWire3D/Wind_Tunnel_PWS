#include <ESP32Servo.h>

Servo motor;

const int switch_gnd_yellow = 2;
const int switch_gnd_green = 3;

const int motorPin = 20;  // Pin connected to the servo

const int minPulseWidth = 1000;  // Minimum pulse width in microseconds (1 ms)
const int maxPulseWidth = 2000;  // Maximum pulse width in microseconds (2 ms)
//const int maxPosition = 1000;    // Maximum position value (0 to 1000)

void setup() {
  Serial.begin(115200);

  pinMode(switch_gnd_yellow, INPUT_PULLUP);
  pinMode(switch_gnd_green, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor.setPeriodHertz(50);  // standard 50 hz servo
  motor.attach(motorPin, 1000, 2000);

  while (!Serial) { delay(10); }
  Serial.println("Serial port opened!");

  if (digitalRead(switch_gnd_yellow) == LOW) {
    esc_calibration();  //only first time
  }


  Serial.println("set pulse with to 1000 (0%)");
  motor.writeMicroseconds(1000);
  delay(5000);
}

void loop() {

  //do nothing
}

void esc_calibration() {
  motor.writeMicroseconds(maxPulseWidth);  // set to 2000us puls length
  Serial.println("max positon");
  Serial.println("now wait for the motor calibration beeps");
  
  while(digitalRead(switch_gnd_green) != LOW){ delay(10);} //if switch is not low wait

  motor.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
  Serial.println("low position");

}
#include <Servo.h>

Servo motor;             // Create a servo object
const int motorPin = A0;  // Pin connected to the servo

const int minPulseWidth = 900;  // Minimum pulse width in microseconds (1 ms)
const int maxPulseWidth = 2100;  // Maximum pulse width in microseconds (2 ms)
//const int maxPosition = 1000;    // Maximum position value (0 to 1000)

void setup() {
  Serial.begin(115200);

  motor.attach(motorPin, minPulseWidth, maxPulseWidth);  // Attach the servo to the specified pin
  while (!Serial) { delay(10); }

  Serial.println("Serial port opened!");
  Serial.println("set pulse with to 900 (0)");
  motor.writeMicroseconds(900);
  delay(5000);

}

void loop() {
  int position = 900;
  for (position = 900.0; position <= 1100; position++) {
    Serial.print("position, ");
    Serial.println(position);
    motor.writeMicroseconds(position);
    delay(50);  // Small delay to simulate real-time control
  }

  for (position = 1100; position >= 900.0; position--) {
    motor.writeMicroseconds(position);
    Serial.print("position, ");
    Serial.println(position);
    delay(15);  // Small delay to simulate real-time control
  }
  delay(5000);
}

//void command_motor(float position) {
  // Map the position (0 to 1000) to the pulse width (1000 us to 2000 us)
  //int pulseWidth = minPulseWidth + (position / maxPosition) * (maxPulseWidth - minPulseWidth);
  //Serial.println(pulseWidth);
 // int pulseWidth = constrain(position, minPulseWidth, maxPulseWidth);
  // Write the pulse width to the servo
 // motor.writeMicroseconds(pulseWidth);
//}

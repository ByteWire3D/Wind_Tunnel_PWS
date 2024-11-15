#include <PWMServo.h>

PWMServo motor1;


int value = 5;
int count;
 void setup() {
  Serial.begin(15200);  // Initialize serial communication
  motor1.attach(2, 900, 2100);
  //esc_calibration();  // only one time
  motor1.write(0);

  Serial.println("setup compleet!");
  delay(5000);
}

void loop() {
  command_motor();
  delay(50);
}

void command_motor() {
  if (value <= 90 and count >= 20) {
    value++;
    count = 0;
    motor1.write(value);
  }
  Serial.print(value);
  Serial.print("\n");
  count++;
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

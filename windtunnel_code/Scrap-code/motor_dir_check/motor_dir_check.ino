#include <ESP32Servo.h>

Servo motor1;
Servo motor2;

const int motorPin1 = 4;  // Pin connected to the servo
const int motorPin2 = 5;  // Pin connected to the servo

const int minPulseWidth = 1000;  // Minimum pulse width in microseconds (1 ms)
const int maxPulseWidth = 2000;  // Maximum pulse width in microseconds (2 ms)
//const int maxPosition = 1000;    // Maximum position value (0 to 1000)

const int killswitch_pin_pressed = 10;

volatile bool state = LOW;  // System initially off
volatile unsigned long last_interrupt_time = 0;


void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor1.setPeriodHertz(50);  // standard 50 hz servo
  motor2.setPeriodHertz(50);  // standard 50 hz servo
  motor1.attach(motorPin1, 1000, 2000);
  motor2.attach(motorPin2, 1000, 2000);

  pinMode(killswitch_pin_pressed, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(killswitch_pin_pressed), killswitch, FALLING);


  //esc_calibration();  //only first time


  Serial.println("set pulse with to 1000 (0%)");
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  delay(5000);
}

void loop() {
  if (state == HIGH) {
     Serial.println("system is active");
    motor1.writeMicroseconds(1300);  //set to 1000us puls length
    motor2.writeMicroseconds(1300);  //set to 1000us puls length
  }
  if (state == LOW) {
    Serial.println("system is not running");
    motor1.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
    motor2.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
  }
  //do nothing
}

void esc_calibration() {
  motor1.writeMicroseconds(maxPulseWidth);  // set to 2000us puls length
  motor2.writeMicroseconds(maxPulseWidth);  // set to 2000us puls length
  Serial.println("max positon");
  Serial.println("now wait for the motor calibration beeps");

  while (state == LOW) { delay(10); }  //if switch is not low wait

  motor1.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
  motor2.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
  Serial.println("low position");
}

void killswitch() {
  unsigned long interrupt_time = millis();

  // Debounce: Ignore interrupt if triggered within the last 200ms
  if (interrupt_time - last_interrupt_time > 200) {
    // Toggle the system state
    state = !state;

    // Motor control based on the new state
    if (state == LOW) {
      motor1.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
      motor2.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
    } else {
      //start test system
    }

    // Update the last interrupt time
    last_interrupt_time = interrupt_time;
  }
}
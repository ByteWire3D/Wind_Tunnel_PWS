const int killswitch_pin_pressed = 9;

volatile bool state = LOW;  // System initially off
volatile unsigned long last_interrupt_time = 0;

void setup() {
  Serial.begin(115200);
  pinMode(killswitch_pin_pressed, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(killswitch_pin_pressed), killswitch, FALLING);
}

void loop() {
  if (state == HIGH) {
    Serial.println("System running...");
    // Main operational code here (e.g., motor control when system is on)
    delay(2000);
  } else {
    Serial.println("System stopped.");
    // Additional actions if needed when the system is off
  }

  //delay(1000);  // Adjust delay as needed
}

void killswitch() {
  unsigned long interrupt_time = millis();

  // Debounce: Ignore interrupt if triggered within the last 200ms
  if (interrupt_time - last_interrupt_time > 200) {
    // Toggle the system state
    state = !state;

    // Motor control based on the new state
    if (state == LOW) {
      //micro.secondwrite(1000); //shut of the motor
    } else {
     //start test system
    }

    // Update the last interrupt time
    last_interrupt_time = interrupt_time;
  }
}

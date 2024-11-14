// Define variables for PID values
float p = 0.0;
float i = 0.0;
float d = 0.0;
bool system_status = LOW;

void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Check if system_status is LOW
  if (system_status == HIGH) {
    Serial.println("System status is not LOW. Commands are disabled.");
    delay(1000);
  }
  
  if (system_status == LOW) {
    // Check if there's incoming data from the serial monitor
    if (Serial.available()) {
      // Read the input as a string
      String input = Serial.readStringUntil('\n');
      input.trim();  // Remove extra spaces and newlines

      // Handle "set" command
      if (input.equalsIgnoreCase("set")) {
        setPIDValues();
      }
      // Handle "dump" command
      else if (input.equalsIgnoreCase("dump")) {
        dumpPIDValues();
      }
    }
  }
}

void setPIDValues() {
  Serial.print("Enter new P value: ");
  while (!Serial.available());  // Wait for user input
  p = Serial.parseFloat();      // Read float value from the serial monitor
  Serial.println(p);
  clearSerialBuffer();          // Clear any leftover characters
  delay(200);

  Serial.print("Enter new I value: ");
  while (!Serial.available());
  i = Serial.parseFloat();
  Serial.println(i);
  clearSerialBuffer();
  delay(200);

  Serial.print("Enter new D value: ");
  while (!Serial.available());
  d = Serial.parseFloat();
  Serial.println(d);
  clearSerialBuffer();
  delay(200);

  Serial.println("PID values updated.");
}

void dumpPIDValues() {
  Serial.println("Current PID Configuration:");
  Serial.print("P = ");
  Serial.println(p);
  Serial.print("I = ");
  Serial.println(i);
  Serial.print("D = ");
  Serial.println(d);
}

// Function to clear the serial buffer
void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();  // Read and discard all characters in the buffer
  }
}

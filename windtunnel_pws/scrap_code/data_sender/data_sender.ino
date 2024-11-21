// Pin definitions (optional, change as needed)
#define TX_PIN 21  // TX pin
#define RX_PIN 20  // RX pin (optional, if the board supports it)

// Simulated data for sending
int loopNumber = 0;
float ampere = 0.0;
float voltage = 3.3;
int motorSignal = 0;
float lift = 1.2;
float drag = 0.9;
unsigned long previousMillis = 0;  // To simulate timestamp

HardwareSerial MySerial(0);  //Create a new HardwareSerial class.

void setup() {
  Serial.begin(115200);  // Initialize serial communication
  MySerial.begin(115200, SERIAL_8N1, RX, TX);
  while (!MySerial) {
    delay(10);
  }

  // Optionally, set up TX/RX pin modes if needed


  Serial.println("Sender Ready");
}

void loop() {
  // Simulate sensor data and send it periodically
  unsigned long currentMillis = millis();

  // Send data every second (1000 ms)
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;

    // Update simulated sensor values
    loopNumber++;
    ampere += 0.1;
    voltage = 3.3 + (random(-10, 10) / 100.0);  // Small voltage fluctuation
    motorSignal = random(0, 255);
    lift += 0.05;
    drag += 0.03;

    // Send each value via Serial in the format "id,value"
    sendSerialData(1, String(loopNumber));     // Loop number
    sendSerialData(2, String(ampere));         // Ampere
    sendSerialData(3, String(voltage));        // Voltage
    sendSerialData(4, String(motorSignal));    // Motor signal
    sendSerialData(5, String(lift));           // Lift
    sendSerialData(6, String(drag));           // Drag
    sendSerialData(7, String(currentMillis));  // Timestamp
  }
}

// Function to send the data in the format "id,value"
void sendSerialData(int id, String value) {
  String message = String(id) + "," + value;
  MySerial.println(message);  // Send the formatted message over Serial
  Serial.println(message);    // Send the formatted message over Serial
}

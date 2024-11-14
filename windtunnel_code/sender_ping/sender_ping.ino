#include <HardwareSerial.h>

#define TX_PIN 21  // TX pin
#define RX_PIN 20  // RX pin (optional, if the board supports it)

HardwareSerial main_controller(0);

struct TestData {
  float calibrationValue_Lift;
  float calibrationValue_Drag;
  float calibrationValue_Ampere;
  float calibrationValue_Airspeed;
  int pid_loop_hz;
  int measurement_hz;
  int screen_hz;
  float p_gain;
  float i_gain;
  float d_gain;
  float delta_max;
  float start_angle;
  float end_angle;
  int time_step;
  int angle_steps;
  int total_steps;
  String test_fileName;
  String speed;
  String angle;
  String time_during_test;
  String test_datum;
  float windspeedList;
};

TestData data = {
  1.2,            // calibrationValue_Lift
  2.3,            // calibrationValue_Drag
  0.8,            // calibrationValue_Ampere
  3.5,            // calibrationValue_Airspeed
  50,             // pid_loop_hz
  100,            // measurement_hz
  60,             // screen_hz
  0.5,            // p_gain
  0.1,            // i_gain
  0.05,           // d_gain
  1.0,            // delta_max
  0.0,            // start_angle
  90.0,           // end_angle
  5,              // time_step
  18,             // angle_steps
  20,             // total_steps
  "TestFile123",  // test_fileName
  "Fast",         // speed
  "Steep",        // angle
  "10min",        // time_during_test
  "2024-10-16"    // test_datum
};

void serializeTestData(const TestData &data, uint8_t *buffer) {
  memcpy(buffer, &data, sizeof(TestData));
}

void sendTestData(HardwareSerial &serial, const TestData &data) {
  uint8_t buffer[sizeof(TestData)];
  serializeTestData(data, buffer);
  serial.write(buffer, sizeof(buffer));
}

bool receiveAcknowledgment(HardwareSerial &serial, const char *expectedAck) {
  if (serial.available() > 0) {
    char ack[4];
    serial.readBytes(ack, sizeof(ack) - 1);
    ack[3] = '\0';  // Null terminate the string
    return strcmp(ack, expectedAck) == 0;
  }
  return false;
}

void clearSerialBuffer(HardwareSerial &serial) {
  while (serial.available() > 0) {
    serial.read();  // Discard all incoming data
  }
}

bool performHandshake(HardwareSerial &serial, unsigned long timeout_ms) {
  const char ping[] = "PING";
  unsigned long start_time = millis();

  clearSerialBuffer(serial);  // Clear the buffer before starting handshake

  // Send the ping message
  serial.write(ping, strlen(ping));
  Serial.println("Ping sent. Waiting for acknowledgment...");

  // Wait for the acknowledgment within the specified timeout
  while (millis() - start_time < timeout_ms) {
    if (serial.available() > 0) {
      char ack[5];  // Adjusted to 5 to ensure enough room for null termination
      serial.readBytes(ack, 4);
      ack[4] = '\0';  // Null-terminate the string for safety
      if (strcmp(ack, "PONG") == 0) {
        Serial.println("Handshake successful.");
        return true;
      } else {
        Serial.println("Received unexpected data: ");
        Serial.println(ack);
        clearSerialBuffer(serial);  // Clear any unexpected data
      }
    }
  }

  Serial.println("Handshake failed.");
  return false;
}



void sendDataWithRetry(HardwareSerial &serial, const TestData &data, unsigned long max_wait_time_ms, unsigned long retry_interval_ms) {
  unsigned long start_time = millis();
  bool ackReceived = false;

  while ((millis() - start_time) < max_wait_time_ms && !ackReceived) {
    // Send data
    sendTestData(serial, data);
    unsigned long retry_start_time = millis();

    // Check for acknowledgment within the retry interval
    while ((millis() - retry_start_time) < retry_interval_ms) {
      if (receiveAcknowledgment(serial, "ACK")) {
        ackReceived = true;
        Serial.println("Acknowledgment received.");
        break;
      }
    }

    if (!ackReceived) {
      Serial.println("No acknowledgment received. Retrying...");
    }

    // Wait until the next retry interval
    while ((millis() - retry_start_time) < retry_interval_ms)
      ;
  }

  if (!ackReceived) {
    Serial.println("Failed to send data within the specified time.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give some time for the Serial Monitor to open

  main_controller.begin(115200, SERIAL_8N1, RX, TX);  //
  while (!main_controller) { delay(10); }
  // Perform the handshake before data communication
  if (!performHandshake(main_controller, 5000)) {
    Serial.println("Communication failed. Unable to proceed.");
    return;
  }


  // Send data with retries every 50ms, with a total wait time of ~47.6ms
  sendDataWithRetry(main_controller, data, 50, 10);
}

void loop() {
 // sendDataWithRetry(main_controller, data, 50, 10);
}

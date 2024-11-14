#include <HardwareSerial.h>

#define TX_PIN 21  // TX pin
#define RX_PIN 20  // RX pin (optional, if the board supports it)

HardwareSerial sd_card(0);  //Create a new HardwareSerial class.


struct conf_settings_testData {
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
};

conf_settings_testData sd_card_data;

struct  measurements {
   float voltage;
   float ampere;
};

void deserializeTestData(const uint8_t *buffer, conf_settings_testData &data) {
  memcpy(&data, buffer, sizeof(sd_card_data));
}

bool receiveTestData(HardwareSerial &serial, conf_settings_testData &data) {
  if (serial.available() >= sizeof(sd_card_data)) {
    uint8_t buffer[sizeof(sd_card_data)];
    serial.readBytes(buffer, sizeof(buffer));
    deserializeTestData(buffer, data);
    return true;
  }
  return false;
}

void sendAcknowledgment(HardwareSerial &serial, const char *ackMessage) {
  serial.write(ackMessage, strlen(ackMessage));
}

void clearSerialBuffer(HardwareSerial &serial) {
  while (serial.available() > 0) {
    serial.read();  // Discard all incoming data
  }
}

bool performHandshake(HardwareSerial &serial, unsigned long timeout_ms) {
  unsigned long start_time = millis();

  clearSerialBuffer(serial);  // Clear the buffer before starting handshake

  // Wait for the ping message within the specified timeout
  while (millis() - start_time < timeout_ms) {
    if (serial.available() > 0) {
      char ping[5]; // Adjusted to 5 to ensure enough room for null termination
      serial.readBytes(ping, 4);
      ping[4] = '\0';  // Null-terminate the string for safety
      if (strcmp(ping, "PING") == 0) {
        // Send back the PONG acknowledgment
        sendAcknowledgment(serial, "PONG");
        Serial.println("Ping received. Handshake complete.");
        return true;
      } else {
        Serial.println("Received unexpected data: ");
        Serial.println(ping);
        clearSerialBuffer(serial);  // Clear any unexpected data
      }
    }
  }

  Serial.println("Handshake failed.");
  return false;
}

void waitForTESTData(HardwareSerial &serial, conf_settings_testData &data, unsigned long max_wait_time_ms) {
  unsigned long start_time = millis();

  while ((millis() - start_time) < max_wait_time_ms) {
    if (receiveTestData(serial, data)) {
      sendAcknowledgment(serial, "ACK");
      Serial.println("Data received:");
      Serial.print("Calibration Lift: ");
      Serial.println(data.calibrationValue_Lift);
      Serial.print("Calibration Drag: ");
      Serial.println(data.calibrationValue_Drag);
      Serial.print("Calibration Ampere: ");
      Serial.println(data.calibrationValue_Ampere);
      Serial.print("Calibration Airspeed: ");
      Serial.println(data.calibrationValue_Airspeed);
      Serial.print("PID Loop Hz: ");
      Serial.println(data.pid_loop_hz);
      Serial.print("Measurement Hz: ");
      Serial.println(data.measurement_hz);
      Serial.print("Screen Hz: ");
      Serial.println(data.screen_hz);
      Serial.print("P Gain: ");
      Serial.println(data.p_gain);
      Serial.print("I Gain: ");
      Serial.println(data.i_gain);
      Serial.print("D Gain: ");
      Serial.println(data.d_gain);
      Serial.print("Delta Max: ");
      Serial.println(data.delta_max);
      Serial.print("Start Angle: ");
      Serial.println(data.start_angle);
      Serial.print("End Angle: ");
      Serial.println(data.end_angle);
      Serial.print("Time Step: ");
      Serial.println(data.time_step);
      Serial.print("Angle Steps: ");
      Serial.println(data.angle_steps);
      Serial.print("Total Steps: ");
      Serial.println(data.total_steps);
      Serial.print("Test File Name: ");
      Serial.println(data.test_fileName);
      Serial.print("Speed: ");
      Serial.println(data.speed);
      Serial.print("Angle: ");
      Serial.println(data.angle);
      Serial.print("Time During Test: ");
      Serial.println(data.time_during_test);
      Serial.print("Test Datum: ");
      Serial.println(data.test_datum);
      return;  // Exit the function as data has been received
    }
  }

  Serial.println("Failed to receive data within the time limit.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  sd_card.begin(115200, SERIAL_8N1, RX, TX);  // RX: GPIO 4, TX: GPIO 5 (change pins as needed)
  while(!sd_card){delay(10);}
  // Perform the handshake before data reception
  if (!performHandshake(sd_card, 5000)) {
    Serial.println("Communication failed. Unable to proceed.");
    return;
  }

  
  Serial.println("Waiting for data...");

  waitForTESTData(sd_card, sd_card_data, 2000);  // Wait for data with a maximum wait time of ~47.6ms
}

void loop() {
   //waitForData(sd_card, data, 50);  // Wait for data with a maximum wait time of ~47.6ms
}

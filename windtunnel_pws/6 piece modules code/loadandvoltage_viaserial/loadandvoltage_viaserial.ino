#include <Wire.h>
#include <Adafruit_INA219.h>
#include <HX711_ADC.h>


Adafruit_INA219 ina219;
HardwareSerial main_controller(0);  //Create a new HardwareSerial class.
#pragma pack(1)
struct Messurment_data {
  float lift_loadcell; // lift loadcell
  float drag_loadcell;
  float ampere;
  float voltage;
  float wattage;
  float mah_used;
};

struct conf_data {
  float calibrationValue_lift;
  float calibrationValue_drag;
  float calibrationValue_Ampere;
};
#pragma pack()
conf_data data_recv;


volatile unsigned long last_time = 0;

const int drag_loadcell_dout = 2;  //mcu > HX711 dout pin
const int drag_loadcell_sck = 3;   //mcu > HX711 sck pin

const int lift_loadcell_dout = 4;  //mcu > HX711 dout pin
const int lift_loadcell_sck = 5;   //mcu > HX711 sck pin

const int ratePin_lift = 8;
const int ratePin_drag = 9;
HX711_ADC lift(lift_loadcell_dout, lift_loadcell_sck);
HX711_ADC drag(drag_loadcell_dout, drag_loadcell_sck);

float calibrationValue_lift = 800.0;    // calibration value load lift
float calibrationValue_drag = -2900.0;  // calibration value load drag

float calibrationValue_Ampere = 1.387184608;
volatile boolean newDataReady = false;

float shuntvoltage;
float busvoltage;
float current_A;
float mah_consumed = 0.0;
float mah_used;
float wattage;

float lift_value;
float drag_value;
volatile int avrg_count = 0;
volatile unsigned long previous_millis = 0;
void setup() {
  Serial.begin(115200);
  main_controller.begin(9600);
  while (!main_controller) { delay(10); }
  pinMode(ratePin_lift, OUTPUT);
  pinMode(ratePin_drag, OUTPUT);

  digitalWrite(ratePin_lift, HIGH);
  digitalWrite(ratePin_drag, HIGH);
  if (!ina219.begin()) {
    Serial.println("Failed to initialize INA219");
    while (1)
      ;
  }

  delay(2000);
  performHandshake(main_controller, 5000);
  handleSetCommand(main_controller, data_recv);


  // Set the custom calibration for 80mV, 150A
  setCalibration_80mV_150A();
  setup_loadcells();


  Serial.println("Startup is complete");
}

void loop() {
  // check for new data/start next conversion:

  unsigned long current_millis = millis();
  if (lift.update()) newDataReady = true;
  drag.update();

  float loop_time = current_millis - previous_millis;
  float hz = 1000 / loop_time;
  
  if ((newDataReady)) {
    shuntvoltage += ina219.getShuntVoltage_mV();              // Measure shunt voltage in millivolts
    busvoltage += ina219.getBusVoltage_V();                   // Measure bus voltage
    current_A += shuntvoltage * 2 * calibrationValue_Ampere;  // Calculate current in amperes
    wattage = current_A * busvoltage;
    if (current_millis != previous_millis) {
      mah_consumed += current_A / 1000;
      mah_used += mah_consumed / 3600 * ((loop_time) / 1000);
    }
    lift_value += lift.getData();
    drag_value += drag.getData();
    Serial.print("shuntvolt: ");
    Serial.print(shuntvoltage);
    Serial.print("\t");
    Serial.print("volt: ");
    Serial.print(busvoltage);
    Serial.print("\t");
    Serial.print("currr: ");
    Serial.print(current_A);
    Serial.print("\t");
    Serial.print("wattage: ");
    Serial.print(wattage);
    Serial.print("\t");
    Serial.print("mah_used: ");
    Serial.print(mah_used);
    Serial.print("\t");
    Serial.print("lift val: ");
    Serial.print(lift_value);
    Serial.print("\t");
    Serial.print("drag val: ");
    Serial.print(drag_value);
    Serial.print("\t");
    Serial.print("hz: ");
    Serial.print(hz);
    Serial.print("\t");
    Serial.print("loop_time: ");
    Serial.println(loop_time);
    newDataReady = false;
    previous_millis = current_millis;

    avrg_count++;
    shuntvoltage / avrg_count;
    busvoltage / avrg_count;
    current_A / avrg_count;
    wattage / avrg_count;
    lift_value / avrg_count;
    drag_value/ avrg_count;

  } 
  if (handleGetCommand(main_controller)) {
    Messurment_data data{
      lift_value,
      drag_value,
      current_A,
      busvoltage,
      wattage,
      mah_used
    };
    sendDataWithRetry(main_controller, data, 50, 25);
    avrg_count = 0;
    shuntvoltage = 0;
    busvoltage = 0;
    current_A = 0;
    wattage = 0;
    lift_value = 0;
    drag_value = 0;
  }


  if (Serial.available()) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove extra spaces and newlines

    // Handle "calibrate lift" command
    if (input.equalsIgnoreCase("calibrate_lift")) {
      calibrate_lift();
      clearSerialBuffer(Serial);
    }

    // Handle "calibrate drag" command
    else if (input.equalsIgnoreCase("calibrate_drag")) {
      calibrate_drag();
      clearSerialBuffer(Serial);
    } else if (input.equalsIgnoreCase("set_lift")) {
      set_lift();
      clearSerialBuffer(Serial);
    } else if (input.equalsIgnoreCase("set_drag")) {
      set_drag();
      clearSerialBuffer(Serial);
    } else if (input.equalsIgnoreCase("tare")) {
      lift.tareNoDelay();
      drag.tareNoDelay();
      clearSerialBuffer(Serial);
    }
  }

  //check if last tare operation is complete
  if (lift.getTareStatus()) {
    Serial.println("Tare load cell lift complete");
  }
  if (drag.getTareStatus()) {
    Serial.println("Tare load cell drag complete");
  }
}

void setup_loadcells() {
  lift.begin();
  drag.begin();
  //lift.setReverseOutput();
  //drag.setReverseOutput();
  unsigned long stabilizingtime = 2000;  // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  byte lift_1_rdy = 0;
  byte drag_2_rdy = 0;
  while ((lift_1_rdy + drag_2_rdy) < 2) {  //run startup, stabilization and tare, both modules simultaniously
    if (!lift_1_rdy) lift_1_rdy = lift.startMultiple(stabilizingtime, _tare);
    if (!drag_2_rdy) drag_2_rdy = drag.startMultiple(stabilizingtime, _tare);
  }
  if (lift.getTareTimeoutFlag() || lift.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 lift wiring and pin designations");
  }
  if (drag.getTareTimeoutFlag() || drag.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 dragwiring and pin designations");
  }
  lift.setCalFactor(calibrationValue_lift);  // user set calibration value (float)
  drag.setCalFactor(calibrationValue_drag);  // user set calibration value (float)
}

void set_lift() {
  float cal_fac;
  bool next = false;
  Serial.println("set your new lift calibration value> ");
  while (next == false) {
    if (Serial.available() > 0) {
      cal_fac = Serial.parseFloat();
      if (cal_fac != 0) {
        Serial.print("new lift calibration value  is: ");
        Serial.println(cal_fac);
        lift.setCalFactor(cal_fac);  // user set calibration value (float)
        next = true;
        delay(4000);
      }
    }
  }
}

void set_drag() {
  float cal_fac;
  bool next = false;
  Serial.println("set your new drag calibration value> ");
  while (next == false) {
    if (Serial.available() > 0) {
      cal_fac = Serial.parseFloat();
      if (cal_fac != 0) {
        Serial.print("new dragg calibration value  is: ");
        Serial.println(cal_fac);
        drag.setCalFactor(cal_fac);  // user set calibration value (float)
        next = true;
        delay(4000);
      }
    }
  }
}

void calibrate_lift() {
  Serial.println("***");
  Serial.println("Start lift calibration:");
  Serial.println("Place the lift load cell on an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    lift.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') lift.tareNoDelay();
      }
    }
    if (lift.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    lift.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  lift.refreshDataSet();                                           //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = lift.getNewCalibration(known_mass);  //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'calibrate_lift' from serial monitor.");
  Serial.println("***");
}

void calibrate_drag() {
  Serial.println("***");
  Serial.println("Start drag calibration:");
  Serial.println("Place the drag load cell on an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    drag.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') drag.tareNoDelay();
      }
    }
    if (drag.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    drag.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  drag.refreshDataSet();                                           //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = drag.getNewCalibration(known_mass);  //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");


  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'calibrate_drag' from serial monitor.");
  Serial.println("***");
}

void setCalibration_80mV_150A() {
  // Calibration value calculated for 80mV range and 0.0005 ohm shunt
  uint16_t calibrationValue = 17893;

  // Configuration register for 16V bus voltage range and ±80mV shunt voltage range
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V | INA219_CONFIG_GAIN_2_80MV |  // Gain = /2 (±80mV range)
                    INA219_CONFIG_BADCRES_12BIT |                                  // Bus ADC resolution = 12-bit
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |                         // Shunt ADC resolution = 12-bit, 1 sample
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;                       // Continuous mode

  // Write calibration value to INA219
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(INA219_REG_CALIBRATION);   // Calibration register
  Wire.write(calibrationValue >> 8);    // MSB
  Wire.write(calibrationValue & 0xFF);  // LSB
  Wire.endTransmission();

  // Write configuration value to INA219
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(INA219_REG_CONFIG);  // Configuration register
  Wire.write(config >> 8);        // MSB
  Wire.write(config & 0xFF);      // LSB
  Wire.endTransmission();
}
template<typename T>
void waitForData(HardwareSerial &serial, T &data, unsigned long max_wait_time_ms, const char *deviceName) {
  unsigned long start_time = millis();

  while ((millis() - start_time) < max_wait_time_ms) {
    if (receiveData(serial, data)) {
      sendAcknowledgment(serial, "ACK");
      if (deviceName == "main_controller") {
        Serial.print("data recieved from:");
        Serial.println(deviceName);
        calibrationValue_lift = data.calibrationValue_lift;
        calibrationValue_drag = data.calibrationValue_drag;
        calibrationValue_Ampere = data.calibrationValue_Ampere;
        Serial.println("cal_lift: ");
        Serial.println(calibrationValue_lift);
        Serial.println("cal_drag: ");
        Serial.println(calibrationValue_drag);
        Serial.println("cal_amper: ");
        Serial.println(calibrationValue_Ampere);
      }
      return;
    }
  }

  Serial.print("Failed to receive data from ");
  Serial.print(deviceName);
  Serial.println(" within the time limit.");
}
template<typename T>
void serializeTestData(T &data, uint8_t *buffer) {
  memcpy(buffer, &data, sizeof(T));
}
template<typename T>
void sendTestData(HardwareSerial &serial, T &data) {
  uint8_t buffer[sizeof(T)];
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

void clearSerialBuffer(Stream &serial) {
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
//template<typename T>
bool handleGetCommand(HardwareSerial &serial) {
  if (serial.available() >= 3) {
    char command[4];
    serial.readBytes(command, 3);
    command[3] = '\0';

    if (strcmp(command, "GET") == 0) {
      Serial.println("GET command received");
      return true;
      // Call the function to send data with retry
     // sendDataWithRetry(serial, dataToSend, 50, 25); // this was commended fucking dumb mistake // never mind other function
    } else if (strcmp(command, "TGE") == 0) {
      Serial.println("TGE command received");
       return true;
     // sendDataWithRetry(serial, dataToSend, 50, 25);
    } else if (strcmp(command, "ETG") == 0) {
      Serial.println("ETG command received");
       return true;
     // sendDataWithRetry(serial, dataToSend, 50, 25);
    }else {
      return false;
    }
  }
}
template<typename T>
void handleSetCommand(HardwareSerial &serial, T &dataToRecieve) {
  while (!(serial.available() >= 3)) {
    delay(1);
  }
  char command[4];
  serial.readBytes(command, 3);
  command[3] = '\0';

  if (strcmp(command, "SET") == 0) {
    Serial.println("SET command received");
    // Call the function to send data with retry
    waitForData(main_controller, dataToRecieve, 1000, "main_controller");
  }
}
void sendAcknowledgment(Stream &serial, const char *ackMessage) {
  serial.write(ackMessage, strlen(ackMessage));
}
template<typename T>
void sendDataWithRetry(HardwareSerial &serial, T &data, unsigned long max_wait_time_ms, unsigned long retry_interval_ms) {
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
template<typename T>
bool receiveData(HardwareSerial &serial, T &data) {
  if (serial.available() >= sizeof(T)) {
    uint8_t buffer[sizeof(T)];
    serial.readBytes(buffer, sizeof(buffer));
    deserializeData(buffer, data);
    return true;
  }
  return false;
}
template<typename T>
void deserializeData(const uint8_t *buffer, T &data) {
  memcpy(&data, buffer, sizeof(T));
}

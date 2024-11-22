#include "FS.h"
#include "SD.h"
#include "SPI.h"

// SPI pin definitions for the Xiao C3
#define SCK 8
#define MISO 9
#define MOSI 10
#define CS 5

//#define TX_PIN 21  // TX pin
//#define RX_PIN 20  // RX pin (optional, if the board supports it)

HardwareSerial main_controller(0);  //Create a new HardwareSerial class.
#pragma pack(1)
struct Conf_Data {
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
  float deadband;
  float start_angle;
  float end_angle;
  int time_step;
  int angle_steps;
  int total_steps;
  int windspeedCount;
  String test_fileName;
  String speed;
  String angle;
  String time_during_test;
  String test_datum;
  float windspeedList[10];
};

struct TestData {
  float setpoint;
  float airspeed;
  float error;
  float output;

  float lift_loadcell;
  float drag_loadcell;
  float pitch;

  float ampere;
  float voltage;
  float wattage;
  float mah_used;

  unsigned long looptime;
  int loop_number;
};
#pragma pack()
TestData data_rcv;

String testFolderName = "";                                       // Will hold the current test folder name
int testCounter = 0;                                              // To track test numbers across boots
const char *testCounterFile = "/configuration/test_counter.txt";  // File to store test counter

// Calibration values
float calibrationValue_Lift;
float calibrationValue_Drag;
float calibrationValue_Ampere;
float calibrationValue_Airspeed;

//settings
int pid_loop_hz;
int measurement_hz;
int screen_hz;

float p_gain;
float i_gain;
float d_gain;

float delta_max;
float deadband;

//test values:
String test_fileName;
String speed;
String angle;
String time_during_test;

String FolderName;
String test_datum;

float start_angle;
float end_angle;

int time_step;
int angle_steps;

int total_steps;

float windspeedList[10];  //make the array bigg enough for all the windspeeds
int windspeedCount = 0;   //number in the list

String calibration_data;
String settings_data;
String test_data;

String file_firstline = "";

String string_data = "";
unsigned long previousMillis = 0;
float loopFrequency;
void setup() {
  Serial.begin(115200);
  main_controller.begin(115200);
  while (!main_controller) { delay(10); }
  delay(1000);
  if (!performHandshake(main_controller, 5000)) {
    Serial.println("Communication failed. Unable to proceed.");
    return;
  }
  // Initialize SPI and SD card
  setup_sdcard();

  // Read calibration values
  readCalibrationValues("/configuration/settings/calibration.txt");

  // Read settings
  readSettings("/configuration/settings/settings.txt");

  //read test format
  String filePath = findTextFile("/configuration/test format");

  if (filePath != "") {

    readTestformat(filePath.c_str());
  } else {
    //Serial.println("No .txt file found in the directory");
  }

  // Output the parsed values for testing
  print_conf_set_testinfo();
  send_data();

  // Read the test counter from the file and increment it
  testCounter = readTestCounter();

  // Create a new directory for the current test
  testFolderName = "/test_results/" + String(FolderName) + " (" + String(testCounter) + ") " + String(test_datum);
  createDir(SD, testFolderName.c_str());
  file_firstline = "Looptime;Loop Number;Setpoint;Airspeed;Error;Output;Lift;Drag;Ampere;Voltage;Wattage;Mah Used";
  //Serial.printf("Created folder: %s\n", testFolderName.c_str());
  appendFile(SD, (testFolderName + "/test_data.txt").c_str(), file_firstline);

  // Increment and update the test counter file
  updateTestCounter(testCounter + 1);
}

void loop() {
  unsigned long loopDuration = millis() - previousMillis;
  loopFrequency = 1000 / loopDuration;
  previousMillis = millis();
  waitForData(main_controller, data_rcv, 200, "main_controller");
}

void setup_sdcard() {
  SPI.begin(SCK, MISO, MOSI, CS);
  while (!SD.begin(CS, SPI, 80000000)) {

    delay(100);
  }

  // Check SD card type
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    //Serial.println("No SD card attached");
    return;
  }

  // Print card size
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  // Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Check if configuration folder exists
  if (!SD.exists("/configuration")) {
    //Serial.println("Configuration folder not found, creating...");
    SD.mkdir("/configuration");
  }
}

void send_data() {
  Conf_Data data = {
    calibrationValue_Lift,      // calibrationValue_Lift
    calibrationValue_Drag,      // calibrationValue_Drag
    calibrationValue_Ampere,    // calibrationValue_Ampere
    calibrationValue_Airspeed,  // calibrationValue_Airspeed
    pid_loop_hz,                // pid_loop_hz
    measurement_hz,             // measurement_hz
    screen_hz,                  // screen_hz
    p_gain,                     // p_gain
    i_gain,                     // i_gain
    d_gain,                     // d_gain
    delta_max,                  // delta_max
    deadband,
    start_angle,  // start_angle
    end_angle,    // end_angle
    time_step,    // time_step
    angle_steps,  // angle_steps
    total_steps,  // total_steps
    windspeedCount,
    test_fileName,     // test_fileName
    speed,             // speed
    angle,             // angle
    time_during_test,  // time_during_test
    test_datum         // test_datum
  };
  memcpy(data.windspeedList, windspeedList, sizeof(windspeedList));

  handleGetCommand(main_controller, data);
}

void print_conf_set_testinfo() {
  Serial.println("calibartion values -------> ");

  Serial.print("lift_cal: ");
  Serial.println(calibrationValue_Lift);
  Serial.print("drag_cal: ");
  Serial.println(calibrationValue_Drag);
  Serial.print("ampere_cal: ");
  Serial.println(calibrationValue_Ampere);
  Serial.print("aispeed_cal: ");
  Serial.println(calibrationValue_Airspeed);
  Serial.println();

  Serial.println("settings -------> ");

  Serial.print("pid_loop_hz: ");
  Serial.println(pid_loop_hz);
  Serial.print("measurement_hz: ");
  Serial.println(measurement_hz);
  Serial.print("screen_hz: ");
  Serial.println(screen_hz);
  Serial.print("p_gain: ");
  Serial.println(p_gain);
  Serial.print("i_gain: ");
  Serial.println(i_gain);
  Serial.print("d_gain: ");
  Serial.println(d_gain);
  Serial.print("delta_max: ");
  Serial.println(delta_max);
  Serial.println();

  Serial.println("testformat data -------> ");
  Serial.print("test_fileName: ");
  Serial.println(test_fileName);
  Serial.print("speed: ");
  Serial.println(speed);
  Serial.print("angle: ");
  Serial.println(angle);
  Serial.print("time_during_test: ");
  Serial.println(time_during_test);
  Serial.print("foldername: ");
  Serial.println(FolderName);
  Serial.print("test_datum: ");
  Serial.println(test_datum);
  Serial.print("pid_loop_hz: ");
  Serial.println(pid_loop_hz);
  Serial.print("measurement_hz: ");
  Serial.println(measurement_hz);
  Serial.print("screen_hz: ");
  Serial.println(screen_hz);
  Serial.print("start_angle: ");
  Serial.println(start_angle);
  Serial.print("end_angle: ");
  Serial.println(end_angle);
  Serial.print("time_step: ");
  Serial.println(time_step);
  Serial.print("angle_steps: ");
  Serial.println(angle_steps);
  Serial.print("total_steps: ");
  Serial.println(total_steps);

  Serial.print("Windspeed on sd:");
  for (int i = 0; i < windspeedCount; i++) {
    Serial.println(windspeedList[i]);
  }
}


// Read the test counter from the file
int readTestCounter() {
  int counter = 1;  // Default counter if file doesn't exist

  // Check if the test_counter file exists
  if (SD.exists(testCounterFile)) {
    File file = SD.open(testCounterFile);
    if (file) {
      String counterStr = file.readStringUntil('\n');
      counter = counterStr.toInt();
      file.close();
      //serial.printf("Test counter read from file: %d\n", counter);
    } else {
      //serial.println("Failed to open test_counter file.");
    }
  } else {
    //serial.println("test_counter file does not exist. Creating a new one.");
  }

  return counter;
}

// Update the test counter in the file
void updateTestCounter(int newCounter) {
  File file = SD.open(testCounterFile, FILE_WRITE);
  if (file) {
    file.printf("%d\n", newCounter);
    file.close();
    //serial.printf("Updated test_counter to: %d\n", newCounter);
  } else {
    //serial.println("Failed to open test_counter file for writing.");
  }
}

// Directory listing function
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  //serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    //serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    //serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      //serial.print("  DIR : ");
      //serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      //serial.print("  FILE: ");
      //serial.print(file.name());
      //serial.print("  SIZE: ");
      //serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Create a directory
void createDir(fs::FS &fs, const char *path) {

  if (fs.mkdir(path)) {
   
  } else {
    //serial.println("mkdir failed");
  }
}

// Append to a file
void appendFile(fs::FS &fs, const char *path, String message) {
  //serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //Serial.println("Failed to open file for appending");
    return;
  }
  if (file.println(message)) {

    Serial.print("Message appended at: ");
    Serial.print(loopFrequency);
    Serial.println("hz");
  } else {
    //Serial.println("Append failed");
  }
  file.close();
}

void readCalibrationValues(const char *path) {
  Serial.println("Reading calibration values...");

  File file = SD.open(path);
  if (!file) {
    Serial.println("Failed to open calibration file");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();  // Remove any leading/trailing whitespace
    if (line.startsWith("calibrationValue_Lift:")) {
      calibrationValue_Lift = extractFloatValue(line);
    } else if (line.startsWith("calibrationValue_Drag:")) {
      calibrationValue_Drag = extractFloatValue(line);
    } else if (line.startsWith("calibrationValue_Ampere:")) {
      calibrationValue_Ampere = extractFloatValue(line);
    } else if (line.startsWith("calibrationValue_Airspeed:")) {
      calibrationValue_Airspeed = extractFloatValue(line);
    }
  }
  file.close();
}

void readSettings(const char *path) {
  Serial.println("Reading settings...");

  File file = SD.open(path);
  if (!file) {
    Serial.println("Failed to open settings file");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();  // Remove any leading/trailing whitespace
    if (line.startsWith("pid_loop_hz:")) {
      pid_loop_hz = extractIntValue(line);
    } else if (line.startsWith("measurement_hz:")) {
      measurement_hz = extractIntValue(line);
    } else if (line.startsWith("screen_hz:")) {
      screen_hz = extractIntValue(line);
    } else if (line.startsWith("p_gain:")) {
      p_gain = extractFloatValue(line);
    } else if (line.startsWith("i_gain:")) {
      i_gain = extractFloatValue(line);
    } else if (line.startsWith("d_gain:")) {
      d_gain = extractFloatValue(line);
    } else if (line.startsWith("delta_max:")) {
      delta_max = extractFloatValue(line);
    } else if (line.startsWith("deadband:")) {
      deadband = extractFloatValue(line);
    }
  }
  file.close();
}
void readTestformat(const char *path) {
  File file = SD.open(path);
  if (!file) {
    Serial.println("could not open file");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();  // Verwijder eventuele spaties aan het begin/einde
    if (line.startsWith("speed:")) {
      speed = extractString(line);
    } else if (line.startsWith("angle:")) {
      angle = extractString(line);
    } else if (line.startsWith("time:")) {
      time_during_test = extractString(line);
    } else if (line.startsWith("speed:")) {
      speed = extractString(line);
    } else if (line.startsWith("FolderName:")) {
      FolderName = extractString(line);
    } else if (line.startsWith("test_datum:")) {
      test_datum = extractString(line);
    } else if (line.startsWith("pid_loop_hz:")) {
      pid_loop_hz = extractIntValue(line);
    } else if (line.startsWith("measurement_hz:")) {
      measurement_hz = extractIntValue(line);
    } else if (line.startsWith("screen_hz:")) {
      screen_hz = extractIntValue(line);
    } else if (line.startsWith("start_angle:")) {
      start_angle = extractIntValue(line);
    } else if (line.startsWith("end_angle:")) {
      end_angle = extractIntValue(line);
    } else if (line.startsWith("time_step:")) {
      time_step = extractIntValue(line);
    } else if (line.startsWith("angle_steps:")) {
      angle_steps = extractIntValue(line);
    } else if (line.startsWith("total_steps:")) {
      total_steps = extractIntValue(line);
    } else if (line.startsWith("windspeed:")) {
      // Haal het deel na de "windspeed:" op
      int separatorIndex = line.indexOf(':');
      String speeds = line.substring(separatorIndex + 1);
      speeds.trim();

      // Split de waarden op de ';'
      int startIndex = 0;
      int endIndex = speeds.indexOf(';');
      while (endIndex != -1) {
        // Haal de substring op en converteer deze naar een float
        String speedValue = speeds.substring(startIndex, endIndex);
        speedValue.trim();  // Verwijder eventuele extra spaties
        windspeedList[windspeedCount++] = speedValue.toFloat();

        // Ga naar de volgende waarde
        startIndex = endIndex + 1;
        endIndex = speeds.indexOf(';', startIndex);
      }
      // Voeg de laatste waarde toe (na de laatste ';')
      String lastSpeedValue = speeds.substring(startIndex);
      lastSpeedValue.trim();
      windspeedList[windspeedCount++] = lastSpeedValue.toFloat();
    }
  }

  file.close();
}


// Function to extract the float value after the identifier
float extractFloatValue(String line) {
  int separatorIndex = line.indexOf(':');
  if (separatorIndex != -1) {
    String valueStr = line.substring(separatorIndex + 1);
    valueStr.trim();            // Remove any extra spaces around the value
    return valueStr.toFloat();  // Convert to float
  }
  return 0.0;  // Return 0.0 if no value found
}

int extractIntValue(String line) {
  int separatorIndex = line.indexOf(':');
  if (separatorIndex != -1) {
    String valueStr = line.substring(separatorIndex + 1);
    valueStr.trim();          // Remove any extra spaces around the value
    return valueStr.toInt();  // Convert to float
  }
  return 0;  // Return 0.0 if no value found
}
String extractString(String line) {
  int separatorIndex = line.indexOf(':');
  if (separatorIndex != -1) {
    String valueStr = line.substring(separatorIndex + 1);
    valueStr.trim();  // Remove any extra spaces around the value
    return valueStr;  // Convert to float
  }
  return "";  // Return nothing if no value found
}

String findTextFile(const char *dirPath) {
  File dir = SD.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    Serial.println("Failed to open directory");
    return "";
  }

  File file = dir.openNextFile();
  while (file) {
    // Check if the file name ends with .txt
    test_fileName = file.name();
    if (test_fileName.endsWith(".txt")) {
      return String(dirPath) + "/" + test_fileName;
    }
    file = dir.openNextFile();
  }

  return "";  // Return empty if no .txt file is found
}
template<typename T>
void waitForData(HardwareSerial &serial, T &data, unsigned long max_wait_time_ms, const char *deviceName) {
  unsigned long start_time = millis();

  while ((millis() - start_time) < max_wait_time_ms) {
    if (receiveData(serial, data)) {
      sendAcknowledgment(serial, "ACK");
      if (deviceName == "main_controller") {
        //Serial.print("data recieved from:");
        // Serial.println(deviceName);
        string_data = String(data.looptime) + ";" + String(data.loop_number) + ";" + String(data.setpoint) + ";" + String(data.airspeed) + ";" + String(data.error) + ";" + String(data.output) + ";" + String(data.lift_loadcell) + ";" + String(data.drag_loadcell) + ";" + String(data.ampere) + ";" + String(data.voltage) + ";" + String(data.wattage) + ";" + String(data.mah_used);
        appendFile(SD, (testFolderName + "/test_data.txt").c_str(), string_data);
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
template<typename T>
void handleGetCommand(HardwareSerial &serial, T &dataToSend) {
  while (!(serial.available() >= 3)) {
    delay(1);
  }
  char command[4];
  serial.readBytes(command, 3);
  command[3] = '\0';

  if (strcmp(command, "GET") == 0) {
    Serial.println("GET command received");
    // Call the function to send data with retry
    sendDataWithRetry(serial, dataToSend, 50, 10);
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

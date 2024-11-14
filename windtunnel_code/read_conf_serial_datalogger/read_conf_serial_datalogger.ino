#include "FS.h"
#include "SD.h"
#include "SPI.h"

// SPI pin definitions for the Xiao C3
#define SCK 8
#define MISO 9
#define MOSI 10
#define CS 5

#define TX_PIN 21  // TX pin
#define RX_PIN 20  // RX pin (optional, if the board supports it)

HardwareSerial main_controller(0);  //Create a new HardwareSerial class.

String testFolderName = "";                                       // Will hold the current test folder name
int testCounter = 0;                                              // To track test numbers across boots
const char *testCounterFile = "/configuration/test_counter.txt";  // File to store test counter

// Calibration values
float calibrationValue_Lift = 500;
float calibrationValue_Drag = 203;
float calibrationValue_Ampere = 23;
float calibrationValue_Airspeed= 93;

//settings
int pid_loop_hz = 21;
int measurement_hz = 212;
int screen_hz = 5;

float p_gain = 621;
float i_gain= 12;
float d_gain =83;

float delta_max= 32;

//test values:
String test_fileName ="yes";
String speed= "varing";
String angle"dynamic";
String time_during_test= "constant";

String FolderName = " ???";
String test_datum= "16.10.2024";

float start_angle = 83;
float end_angle =12;
const float max_angle = 45;

int time_step = 01;
int angle_steps = 92;

int total_steps =1932;

float windspeedList[10];  //make the array bigg enough for all the windspeeds
int windspeedCount = 0;   //number in the list

String calibration_data ;
String settings_data;
String test_data;



void setup() {
  Serial.begin(115200);
  main_controller.begin(115200, SERIAL_8N1, RX, TX);
 // while(!Serial){delay(10);}
  while (!main_controller){delay(10);}

  // Initialize SPI and SD card
  setup_sdcard();

  // Read calibration values
  readCalibrationValues("/configuration/settings/calibration.txt");

  // Read settings
  readSettings("/configuration/settings/settings.txt");

  //read test format
  String filePath = findTextFile("/configuration/test format");

  if (filePath != "") {
    //Serial.print("Found file: ");
    //Serial.println(filePath);
    readTestformat(filePath.c_str());
  } else {
    //Serial.println("No .txt file found in the directory");
  }

  // Output the parsed values for testing
  print_conf_set_testinfo();
  send_conf_set_testformat();
 // send_data();
  // Read the test counter from the file and increment it
  testCounter = readTestCounter();

  // Create a new directory for the current test
  testFolderName = "/test_results/" + String(FolderName) + " (" + String(testCounter) + ") " + String(test_datum);
  createDir(SD, testFolderName.c_str());
  //Serial.printf("Created folder: %s\n", testFolderName.c_str());

  // Increment and update the test counter file
  updateTestCounter(testCounter + 1);
}

void loop() {
   send_conf_set_testformat();
   delay(1000);
  // Wait for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any newline characters
    processSerialInput(input);
  }
}

void setup_sdcard() {
  SPI.begin(SCK, MISO, MOSI, CS);
  while (!SD.begin(CS)) {
    //Serial.println("Card Mount Failed");
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

void send_conf_set_testformat() {
  while (!main_controller) {
    delay(10);
  }
  main_controller.println("calibrationValue_Lift:" +String(calibrationValue_Lift));
  main_controller.print("calibrationValue_Drag:");
  main_controller.println(calibrationValue_Drag);
  main_controller.print("calibrationValue_Ampere:");
  main_controller.println(calibrationValue_Ampere);
  main_controller.print("calibrationValue_Airspeed:");
  main_controller.println(calibrationValue_Airspeed);

  main_controller.print("pid_loop_hz:");
  main_controller.println(pid_loop_hz);
  main_controller.print("measurement_hz:");
  main_controller.println(measurement_hz);
  main_controller.print("screen_hz:");
  main_controller.println(screen_hz);
  main_controller.print("p_gain:");
  main_controller.println(p_gain);
  main_controller.print("i_gain:");
  main_controller.println(i_gain);
  main_controller.print("d_gain:");
  main_controller.println(d_gain);
  main_controller.print("delta_max:");
  main_controller.println(delta_max);
  //main_controller.println();

  main_controller.print("test_fileName:");
  main_controller.println(test_fileName);
  main_controller.print("speed:");
  main_controller.println(speed);
  main_controller.print("angle:");
  main_controller.println(angle);
  main_controller.print("time_during_test:");
  main_controller.println(time_during_test);
  main_controller.print("test_datum:");
  main_controller.println(test_datum);
  main_controller.print("pid_loop_hz:");
  main_controller.println(pid_loop_hz);
  main_controller.println("measurement_hz:"+String(measurement_hz));
  main_controller.println("screen_hz:"+String(screen_hz));
  main_controller.println("start_angle:"+String(start_angle));
  main_controller.println("end_angle:"+String(end_angle));
  main_controller.println("time_step:"+String(time_step));
  main_controller.println("angle_steps:"+String(angle_steps));
  main_controller.println("total_steps:"+String(total_steps));


  //main_controller.println("Windspeed:");
  //for (int i = 0; i < windspeedCount; i++) {
   // Serial.println(windspeedList[i]);
 // }
}

void send_data() {
  while (!Serial) {
    delay(10);
  }
  main_controller.print(calibration_data);
  main_controller.print("\n");
  main_controller.print(settings_data);
  main_controller.print("\n");
  main_controller.print(test_data);
  main_controller.print("\n");
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

  Serial.println("Windspeed on sd:");
  for (int i = 0; i < windspeedCount; i++) {
    Serial.println(windspeedList[i]);
  }
}
// Process incoming serial data
void processSerialInput(String input) {
  int delimiterIndex = input.indexOf(',');
  if (delimiterIndex == -1) {
    ////serial.println("Invalid data format.");
    return;
  }

  String id = input.substring(0, delimiterIndex);
  String value = input.substring(delimiterIndex + 1);

  // Validate and process data based on ID
  if (id == "1") {  // Loop Number
    appendFile(SD, (testFolderName + "/loop_number.txt").c_str(), value.c_str());
  } else if (id == "2") {  // Ampere
    appendFile(SD, (testFolderName + "/ampere.txt").c_str(), value.c_str());
  } else if (id == "3") {  // Voltage
    appendFile(SD, (testFolderName + "/voltage.txt").c_str(), value.c_str());
  } else if (id == "4") {  // Motor Signal
    appendFile(SD, (testFolderName + "/motor_signal.txt").c_str(), value.c_str());
  } else if (id == "5") {  // Lift
    appendFile(SD, (testFolderName + "/lift.txt").c_str(), value.c_str());
  } else if (id == "6") {  // Drag
    appendFile(SD, (testFolderName + "/drag.txt").c_str(), value.c_str());
  } else if (id == "7") {  // Timestamp
    appendFile(SD, (testFolderName + "/timestamp.txt").c_str(), value.c_str());
  } else {
    ////serial.println("Invalid ID.");
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
  //serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    //serial.println("Dir created");
  } else {
    //serial.println("mkdir failed");
  }
}

// Append to a file
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //serial.println("Failed to open file for appending");
    return;
  }
  if (file.println(message)) {
    //serial.println("Message appended");
  } else {
    //serial.println("Append failed");
  }
  file.close();
}
/*
void decode_CalibrationValues(String file) {

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

void decode_Settings(String file) {

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
    }
  }
  file.close();
}

void decode_Testformat(String file) {

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
*/
/*
void readCalibrationValues(const char *path) {
  //serial.println("Reading calibration values...");

  File file = SD.open(path);
  if (!file) {
    //serial.println("Failed to open calibration file");
    return;
  }

  while (file.available()) {
    calibration_data = file.readStringUntil('\n');
    calibration_data.trim();  // Remove any leading/trailing whitespace
  }
  file.close();
}

void readSettings(const char *path) {
  //serial.println("Reading settings...");

  File file = SD.open(path);
  if (!file) {
    //serial.println("Failed to open settings file");
    return;
  }

  while (file.available()) {
    settings_data = file.readStringUntil('\n');
    settings_data.trim();  // Remove any leading/trailing whitespace
  }
  file.close();
}
void readTestformat(const char *path) {
  File file = SD.open(path);
  if (!file) {
    //serial.println("could not open file");
    return;
  }

  while (file.available()) {
    test_data = file.readStringUntil('\n');
    test_data.trim();  // Remove any leading/trailing whitespace
  }

  file.close();
}
*/
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
void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();  // Read and discard all characters in the buffer
  }
}

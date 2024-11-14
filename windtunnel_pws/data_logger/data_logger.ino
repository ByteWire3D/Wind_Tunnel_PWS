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

HardwareSerial MySerial(0);  // Create a new HardwareSerial class.

String testFolderName = "";  // Will hold the current test folder name
int testCounter = 0;         // To track test numbers across boots
const char *testCounterFile = "/test_counter.txt";  // File to store test counter

void setup() {
  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  while (!MySerial) {
    delay(10);
  }

  // Initialize SPI and SD card
  SPI.begin(SCK, MISO, MOSI, CS);
  while (!SD.begin(CS)) {
    Serial.println("Card Mount Failed");
    delay(100);
  }

  // Check SD card type
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  // Print card size
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Read the test counter from the file and increment it
  testCounter = readTestCounter();

  // Create a new directory for the current test
  testFolderName = "/test_" + String(testCounter);
  createDir(SD, testFolderName.c_str());
  Serial.printf("Created folder: %s\n", testFolderName.c_str());

  // Increment and update the test counter file
  updateTestCounter(testCounter + 1);
}

void loop() {
  // Wait for serial input
  if (MySerial.available()) {
    String input = MySerial.readStringUntil('\n');
    input.trim();  // Remove any newline characters
    processSerialInput(input);
  }
}

// Process incoming serial data
void processSerialInput(String input) {
  int delimiterIndex = input.indexOf(',');
  if (delimiterIndex == -1) {
    Serial.println("Invalid data format.");
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
    Serial.println("Invalid ID.");
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
      Serial.printf("Test counter read from file: %d\n", counter);
    } else {
      Serial.println("Failed to open test_counter file.");
    }
  } else {
    Serial.println("test_counter file does not exist. Creating a new one.");
  }

  return counter;
}

// Update the test counter in the file
void updateTestCounter(int newCounter) {
  File file = SD.open(testCounterFile, FILE_WRITE);
  if (file) {
    file.printf("%d\n", newCounter);
    file.close();
    Serial.printf("Updated test_counter to: %d\n", newCounter);
  } else {
    Serial.println("Failed to open test_counter file for writing.");
  }
}

// Directory listing function
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Create a directory
void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

// Append to a file
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.println(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

#include "FS.h"
#include "SD.h"
#include "SPI.h"

// SPI pin definitions for the Xiao C3
#define SCK 8
#define MISO 9
#define MOSI 10
#define CS 3

String testFolderName = ""; // Will hold the current test folder name
int testCounter = 0; // To track test numbers across boots

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Initialize SPI and SD card
  SPI.begin(SCK, MISO, MOSI, CS);
  if (!SD.begin(CS)) {
    Serial.println("Card Mount Failed");
    return;
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

  // List existing directories and determine the next test number
  listDir(SD, "/", 0);
  testCounter = getNextTestNumber();
  
  // Create a new directory for the current test
  testFolderName = "/test" + String(testCounter);
  createDir(SD, testFolderName.c_str());
  Serial.printf("Created folder: %s\n", testFolderName.c_str());
}

void loop() {
  // Wait for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
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
  if (id == "1") { // Loop Number
    appendFile(SD, (testFolderName + "/loop_number.txt").c_str(), value.c_str());
  } else if (id == "2") { // Ampere
    appendFile(SD, (testFolderName + "/ampere.txt").c_str(), value.c_str());
  } else if (id == "3") { // Voltage
    appendFile(SD, (testFolderName + "/voltage.txt").c_str(), value.c_str());
  } else if (id == "4") { // Motor Signal
    appendFile(SD, (testFolderName + "/motor_signal.txt").c_str(), value.c_str());
  } else if (id == "5") { // Lift
    appendFile(SD, (testFolderName + "/lift.txt").c_str(), value.c_str());
  } else if (id == "6") { // Drag
    appendFile(SD, (testFolderName + "/drag.txt").c_str(), value.c_str());
  } else if (id == "7") { // Timestamp
    appendFile(SD, (testFolderName + "/timestamp.txt").c_str(), value.c_str());
  } else {
    Serial.println("Invalid ID.");
  }
}

// Function to list directories and determine next test folder number
int getNextTestNumber() {
  int highestTestNumber = 0;
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open root directory");
    return highestTestNumber;
  }

  File file = root.openNextFile();
  while (file) {
    String fileName = String(file.name());
    if (file.isDirectory() && fileName.startsWith("/test")) {
      int currentTestNumber = fileName.substring(5).toInt();
      if (currentTestNumber > highestTestNumber) {
        highestTestNumber = currentTestNumber;
      }
    }
    file = root.openNextFile();
  }

  return highestTestNumber + 1;
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

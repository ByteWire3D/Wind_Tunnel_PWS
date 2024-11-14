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
  

  // Output the parsed values for testing
  print_conf_set_testinfo();
  send_conf_set_testformat();
 // send_data();

  // Create a new directory for the current test

}

void loop() {
   

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

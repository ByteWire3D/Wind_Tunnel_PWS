#define TX_PIN 21  // TX pin
#define RX_PIN 20  // RX pin (optional, if the board supports it)


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

//test values:
String test_fileName;
String speed;
String angle;
String time_during_test;

String FolderName;
String test_datum;

float start_angle;
float end_angle;
const float max_angle = 45;

int time_step;
int angle_steps;

int total_steps;

float windspeedList[10];  //make the array bigg enough for all the windspeeds
int windspeedCount = 0;   //number in the list

String calibration_data;
String settings_data;
String test_data;

int recieved = 0;
HardwareSerial sd_card(0);  //Create a new HardwareSerial class.


void setup() {
  Serial.begin(115200);
  sd_card.begin(115200, SERIAL_8N1, RX, TX);

  while (!Serial) { delay(10); }
  while (!sd_card) { delay(10); }
  Serial.println("setup done");
  while (recieved != 22) {
    if (sd_card.available()) {
      String input = sd_card.readStringUntil('\n');
      input.trim();  // Remove extra spaces and newlines
      processInput(input);
     // clearhardwareBuffer();
    }
  }
  print_conf_set_testinfo();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void processInput(String input) {
  int delimiterIndex = input.indexOf(':');
  if (delimiterIndex == -1) {
    Serial.println("Invalid data format.");
    return;
  }

  String id = input.substring(0, delimiterIndex);
  String value = input.substring(delimiterIndex + 1);
  // Validate and process data based on ID
  if (id == "total_steps") {  // Ampere
    total_steps = value.toInt();
    recieved++;
  } else if (id == "angle_steps") {  // Voltage
    angle_steps = value.toInt();
    recieved++;
  } else if (id == "time_step") {  // Motor Signal
    time_step = value.toInt();
    recieved++;
  } else if (id == "end_angle") {  // Lift
    end_angle = value.toInt();
    recieved++;
  } else if (id == "start_angle") {  // Drag
    start_angle = value.toInt();
    recieved++;
  } else if (id == "screen_hz") {  // Timestamp
    screen_hz = value.toInt();
    recieved++;
  } else if (id == "pid_loop_hz") {  // Timestamp
    pid_loop_hz = value.toInt();
    recieved++;
  }else if (id == "measurement_hz") {  // Timestamp
    measurement_hz = value.toInt();
    recieved++;
  }

  else if (id == "test_datum") {  // Timestamp
    test_datum = value;
    recieved++;
  } else if (id == "time_during_test") {  // Timestamp
    time_during_test = value;
    recieved++;
  } else if (id == "angle") {  // Timestamp
    angle = value;
    recieved++;
  } else if (id == "speed") {  // Timestamp
    speed = value;
    recieved++;
  } else if (id == "test_fileName") {  // Timestamp
    test_fileName = value;
    recieved++;
  } else if (id == "delta_max") {  // Timestamp
    delta_max = value.toFloat();
    recieved++;
  } else if (id == "d_gain") {  // Timestamp
    d_gain = value.toFloat();
    recieved++;
  } else if (id == "i_gain") {  // Timestamp
    i_gain = value.toFloat();
    recieved++;
  } else if (id == "p_gain") {  // Timestamp
    p_gain = value.toFloat();
    recieved++;
  } else if (id == "calibrationValue_Airspeed") {  // Timestamp
    calibrationValue_Airspeed = value.toFloat();
    recieved++;
  } else if (id == "calibrationValue_Ampere") {  // Timestamp
    calibrationValue_Ampere = value.toFloat();
    recieved++;
  } else if (id == "calibrationValue_Drag") {  // Timestamp
    calibrationValue_Drag = value.toFloat();
    recieved++;
  } else if (id == "calibrationValue_Lift") {  // Timestamp
    calibrationValue_Lift = value.toFloat();
    recieved++;
  } else {
    Serial.println("Invalid ID.");
  }
}

void decode_CalibrationValues(String line) {

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

void decode_Settings(String line) {
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


void decode_Testformat(String line) {
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

void clearhardwareBuffer() {
  while (sd_card.available()) {
    sd_card.read();  // Read and discard all characters in the buffer
  }
}

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

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <SPI.h>

HardwareSerial main_controller(0);  //Create a new HardwareSerial class.

#define TFT_RST 3
#define TFT_DC 4  // define data/command pin
#define TFT_CS 5  // define chip select pin
#define DIM_PIN 6


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

String file_name;

float setpoint;
float airspeed;
float error;

float output;
float baseline;
int motor_1;
int motor_2;

float lift;
float drag;
float pitch;

float ampere;
float voltage;
float wattage;
float mah_used;

int status;
int mode;

String error_flags;
int time_seconds;
String display_min;
String display_sec;

float p_out;
float i_out;
float d_out;

float p_gain;
float i_gain;
float d_gain;

int header_pos;

int start_x_left;
int start_x_middel;


int word_spacing;
int list_spacing;

int start_y;

int word_count = 0;
int list_count = 0;
volatile long prev_millis = 0;
#pragma pack(1)
struct display_data_recv {
  String file_name;
  float setpoint;
  float airspeed;
  float error;
  float output;
  float baseline;
  int motor_1;
  int motor_2;
  float lift;
  float drag;
  float pitch;
  float ampere;
  float voltage;
  float wattage;
  float mah_used;
  int status;
  int mode;
  int time_seconds;
  float p_out;
  float i_out;
  float d_out;
  float p_gain;
  float i_gain;
  float d_gain;
};
#pragma pack()
String status_text = "";
String mode_text = "";
display_data_recv display_recv;

bool recieve = false;
int count = 0;
void setup() {
  Serial.begin(115200);
  main_controller.begin(9600);

  tft.init(240, 280);  // Init ST7789 280x240
  tft.setRotation(1);
  tft.setTextWrap(false);
  pinMode(DIM_PIN, OUTPUT);
  digitalWrite(DIM_PIN, HIGH);
  tft.fillScreen(ST77XX_BLACK);
}


void loop() {
  recieveData_nodelay(main_controller, display_recv, "main_controller");
  if (millis() - prev_millis >= 500) {
    prev_millis = millis();
    if (recieve) {
      recieve != recieve;
    }
    if (!recieve) {
      count++;
    }
    if (count >= 4) {
      status = 3;
      show_display();
    }
  }
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
  if (id == "file_name") {  // Loop Number
    file_name = value.c_str();
  }

  else if (id == "setpoint") {  // Ampere
    setpoint = value.toFloat();
  } else if (id == "airspeed") {  // Ampere
    airspeed = value.toFloat();
  } else if (id == "error") {  // Voltage
    error = value.toFloat();
  }

  else if (id == "output") {  // Motor Signal
    output = value.toFloat();
  } else if (id == "baseline") {  // Lift
    baseline = value.toFloat();
  } else if (id == "motor_1") {  // Lift
    motor_1 = value.toInt();
  } else if (id == "motor_2") {  // Drag
    motor_2 = value.toInt();
  }

  else if (id == "lift") {  // Timestamp
    lift = value.toFloat();
  } else if (id == "drag") {  // Timestamp
    drag = value.toFloat();
  } else if (id == "pitch") {  // Timestamp
    pitch = value.toFloat();
  }

  else if (id == "p_gain") {  // Timestamp
    p_gain = value.toFloat();
  } else if (id == "i_gain") {  // Timestamp
    i_gain = value.toFloat();
  } else if (id == "d_gain") {  // Timestamp
    d_gain = value.toFloat();
  }

  else if (id == "p_out") {  // Timestamp
    p_out = value.toFloat();
  } else if (id == "i_out") {  // Timestamp
    i_out = value.toFloat();
  } else if (id == "d_out") {  // Timestamp
    d_out = value.toFloat();
  }

  else if (id == "mode") {  // Timestamp
    mode = value.toInt();
  } else if (id == "time") {  // Timestamp
    time_seconds = value.toInt();
  }

  else if (id == "ampere") {  // Timestamp
    ampere = value.toFloat();
  } else if (id == "voltage") {  // Timestamp
    voltage = value.toFloat();
  } else if (id == "wattage") {  // Timestamp
    wattage = value.toFloat();
  } else if (id == "mah_used") {  // Timestamp
    mah_used = value.toFloat();
  }

  else {
    Serial.println("Invalid ID.");
  }
}

void show_display() {
  start_x_left = 16;
  start_x_middel = 150;


  word_spacing = 10;
  list_spacing = 18;

  start_y = (240 - (word_spacing * 13 + list_spacing * 3)) / 2;

  show_header();
  show_sse();
  show_obmm();
  show_ldp();
  show_avwm();

  list_count = 0, word_count = 0;
  show_status();
  show_mode();
  show_time();
  show_pid_out();
  show_pid();

  list_count = 0, word_count = 0;
}

void show_header() {
  String header_text = "file: " + file_name;
  header_pos = (280 - calculate_Length(header_text, 2)) / 2;  // centers header
  tft.setCursor(header_pos, 0);
  tft.setTextColor(0x5D39, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print("file: " + file_name);
  tft.fillRoundRect(start_x_left, 18, 280 - start_x_left * 2, 3, 3, ST77XX_RED);  // y = 24
}

void show_sse() {
  if (setpoint <= 5) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }

  if (setpoint > 5 && setpoint <= 9) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (setpoint > 9) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y);  // links boven in
  tft.setTextSize(1);
  tft.print("Setpoint: ");

  tft.setCursor(start_x_left + calculate_Length("setpoint: ", 1), start_y);  // links boven in
  tft.setTextSize(1);
  tft.print(setpoint);

  tft.setCursor(start_x_left + calculate_Length("setpoint: ", 1) + calculate_Length(String(setpoint), 1), start_y);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" m/s ");

  word_count++;
  if (airspeed <= 5) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }

  if (airspeed > 5 && airspeed <= 9) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (airspeed > 9) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Airspeed: ");

  tft.setCursor(start_x_left + calculate_Length("airspeed: ", 1), start_y + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print(airspeed);

  tft.setCursor(start_x_left + calculate_Length("airspeed: ", 1) + calculate_Length(String(airspeed), 1), start_y + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" m/s ");

  word_count++;

  if (error <= 0.1) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }

  if (error > 0.1 && error <= 0.3) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (error > 0.3) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Error: ");

  tft.setCursor(start_x_left + calculate_Length("Error: ", 1), start_y + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print(error);

  tft.setCursor(start_x_left + calculate_Length("Error: ", 1) + calculate_Length(String(error), 1), start_y + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" m/s ");
  word_count++;
  list_count++;
}

void show_obmm() {
  if (output <= 1300) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (output > 1300 && output <= 1600) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (output > 1600) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print("Output: ");

  tft.setCursor(start_x_left + calculate_Length("output: ", 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print(output);

  tft.setCursor(start_x_left + calculate_Length("output: ", 1) + calculate_Length(String(output), 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" ms ");

  word_count++;

  if (baseline <= 1300) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (baseline > 1300 && baseline <= 1600) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (baseline > 1600) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print("Baseline: ");

  tft.setCursor(start_x_left + calculate_Length("baseline: ", 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print(baseline);

  tft.setCursor(start_x_left + calculate_Length("baseline: ", 1) + calculate_Length(String(baseline), 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" ms");

  word_count++;

  if (motor_1 <= 1300) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (motor_1 > 1300 && motor_1 <= 1600) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (motor_1 > 1600) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print("Motor 1: ");

  tft.setCursor(start_x_left + calculate_Length("motor 1: ", 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print(motor_1);

  tft.setCursor(start_x_left + calculate_Length("motor 1: ", 1) + calculate_Length(String(motor_1), 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" ms ");

  word_count++;

  if (motor_2 <= 1300) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (motor_2 > 1300 && motor_2 <= 1600) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (motor_2 > 1600) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print("Motor 2: ");

  tft.setCursor(start_x_left + calculate_Length("motor 2: ", 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.print(motor_2);

  tft.setCursor(start_x_left + calculate_Length("motor 2: ", 1) + calculate_Length(String(motor_2), 1), start_y + word_spacing * word_count + list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" ms ");

  word_count++;
  list_count++;
}

void show_ldp() {
  if (lift <= 300) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (lift > 300 && lift <= 600) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (lift > 600) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Lift: ");

  tft.setCursor(start_x_left + calculate_Length("lift: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(lift);

  tft.setCursor(start_x_left + calculate_Length("lift: ", 1) + calculate_Length(String(lift), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" gram ");
  word_count++;

  if (drag <= 300) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (drag > 300 && drag <= 600) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (drag > 600) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Drag: ");

  tft.setCursor(start_x_left + calculate_Length("Drag: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(drag);

  tft.setCursor(start_x_left + calculate_Length("drag: ", 1) + calculate_Length(String(drag), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" gram ");
  word_count++;

  if (pitch <= 15) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (pitch > 15 && pitch <= 30) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (pitch > 30) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Pitch: ");

  tft.setCursor(start_x_left + calculate_Length("Pitch: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(pitch);

  tft.setCursor(start_x_left + calculate_Length("pitch: ", 1) + calculate_Length(String(pitch), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" deg ");
  word_count++;
  list_count++;
}

void show_avwm() {
  if (ampere <= 20) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (ampere > 20 && ampere <= 40) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (ampere > 40) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + list_spacing * list_count + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Ampere: ");

  tft.setCursor(start_x_left + calculate_Length("Ampere: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(ampere);

  tft.setCursor(start_x_left + calculate_Length("Ampere: ", 1) + calculate_Length(String(ampere), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" amps ");
  word_count++;



  if (voltage <= 3.8 * 3) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }
  if (voltage > 3.8 * 3 && voltage <= 4 * 3) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (voltage > 4 * 3) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + list_spacing * list_count + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Voltage: ");

  tft.setCursor(start_x_left + calculate_Length("Voltage: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(voltage);

  tft.setCursor(start_x_left + calculate_Length("Voltage: ", 1) + calculate_Length(String(voltage), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" volt ");

  word_count++;

  if (wattage <= 12 * 20) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (wattage > 12 * 20 && wattage <= 12 * 40) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (wattage > 40 * 12) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + list_spacing * list_count + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Wattage: ");

  tft.setCursor(start_x_left + calculate_Length("wattage: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(wattage);

  tft.setCursor(start_x_left + calculate_Length("wattage: ", 1) + calculate_Length(String(wattage), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" watt ");

  word_count++;

  if (mah_used <= 1500) {
    tft.setTextColor(0x1386, ST77XX_BLACK);
  }
  if (mah_used > 1500 && mah_used <= 3500) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  }
  if (mah_used > 3500) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  }

  tft.setCursor(start_x_left, start_y + list_spacing * list_count + word_spacing * word_count);  // links boven in
  tft.setTextSize(1);
  tft.print("Mah Used: ");

  tft.setCursor(start_x_left + calculate_Length("mah_used: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.print(mah_used);

  tft.setCursor(start_x_left + calculate_Length("mah_used: ", 1) + calculate_Length(String(mah_used), 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.print(" mah ");

  word_count++;
}
void show_status() {
  tft.setCursor(start_x_middel, start_y + list_count * list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("Status: ");
  tft.setCursor(start_x_middel + calculate_Length("Status: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  if (status == 1) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    status_text = "Running!";
  } else if (status == 0) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    status_text = "Stand-by";
  } else if (status == 3) {
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    status_text = "No data??";
  }
  tft.print(status_text + "                           ");
  list_count++;
}

void show_mode() {
  tft.setCursor(start_x_middel, start_y + list_count * list_spacing);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("Mode: ");
  tft.setCursor(start_x_middel + calculate_Length("mode: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  if (mode == 1) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    mode_text = "pid controller armed!";
  } else if (mode == 2) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    mode_text = "testing active";
  } else if (mode == 3) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    mode_text = "armed (waiting)";
  }
  tft.print(mode_text + "                           ");
  list_count++;
}

void show_time() {
  tft.setCursor(start_x_middel, start_y + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("Time= ");
  convert_seconds(time_seconds);
  list_count++;
}
void show_pid_out() {
  tft.setCursor(start_x_middel, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("P-out: ");
  tft.setCursor(start_x_middel + calculate_Length("p-out: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(String(p_out) + "     ");
  word_count++;

  tft.setCursor(start_x_middel, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("I-out: ");
  tft.setCursor(start_x_middel + calculate_Length("i-out: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(String(i_out) + "     ");
  word_count++;

  tft.setCursor(start_x_middel, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("D-out: ");
  tft.setCursor(start_x_middel + calculate_Length("d-out: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(String(d_out) + "     ");
  word_count++;
  list_count++;
}

void show_pid() {
  tft.setCursor(start_x_middel, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("P-gain: ");
  tft.setCursor(start_x_middel + calculate_Length("P_gain: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(String(p_gain) + "     ");
  word_count++;

  tft.setCursor(start_x_middel, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("I-gain: ");
  tft.setCursor(start_x_middel + calculate_Length("I_gain: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(String(i_gain) + "     ");
  word_count++;

  tft.setCursor(start_x_middel, start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.print("D-gain: ");
  tft.setCursor(start_x_middel + calculate_Length("D_gain: ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.print(String(d_gain) + "     ");
  word_count++;
}

void convert_seconds(int seconds) {

  int t = seconds;

  int s = t % 60;

  t = (t - s) / 60;
  int m = t % 60;

  //Serial.print(s);
  // Serial.print("\t");
  // Serial.println(m);
  tft.setCursor(start_x_middel + calculate_Length("Time= ", 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  tft.setTextSize(1);
  if (m < 10) {
    display_min = "0" + String(m) + ":";
    tft.print(display_min);
  }
  if (m >= 10) {
    display_min = String(m) + ":";
    tft.print(display_min);
  }

  tft.setCursor(start_x_middel + calculate_Length("Time= ", 1) + calculate_Length(display_min, 1), start_y + word_spacing * word_count + list_spacing * list_count);  // links boven in
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_SLPIN, ST77XX_BLACK);
  if (s < 10) {
    display_sec = "0" + String(s);
    tft.print(display_sec);
  }
  if (s >= 10) {
    display_sec = s;
    tft.print(display_sec);
  }
}


void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

int calculate_Length(String word, int text_size) {
  int length = 0;
  switch (text_size) {
    case 1:
      for (int i = 0; word[i] != '\0'; i++) {
        if (word[i] == ' ') {
          length += 5;
        } else {
          length += 6;
        }
      }
      return length;

    case 2:
      for (int i = 0; word[i] != '\0'; i++) {
        if (word[i] == ' ') {
          length += 10;
        } else {
          length += 12;
        }
      }
      return length;
  }
}

template<typename T>
void recieveData_nodelay(HardwareSerial &serial, T &data, const char *deviceName) {
  if (receiveData(serial, data)) {
    if (deviceName == "main_controller") {
      recieve = true;
      count = 0;
      file_name = data.file_name;
      setpoint = data.setpoint;

      airspeed = data.airspeed;
      error = data.error;

      output = data.output;
      baseline = data.baseline;
      motor_1 = data.motor_1;
      motor_2 = data.motor_2;

      lift = data.lift;
      drag = data.drag;
      pitch = data.pitch;

      ampere = data.ampere;
      voltage = data.voltage;
      wattage = data.wattage;
      mah_used = data.mah_used;

      status = data.status;
      mode = data.mode;

      time_seconds = data.time_seconds;

      p_out = data.p_out;
      i_out = data.i_out;
      d_out = data.d_out;

      p_gain = data.p_gain;
      i_gain = data.i_gain;
      d_gain = data.d_gain;

      Serial.print("File Name: ");
      Serial.println(data.file_name);

      Serial.print("Setpoint: ");
      Serial.println(data.setpoint);

      Serial.print("Airspeed: ");
      Serial.println(data.airspeed);

      Serial.print("Error: ");
      Serial.println(data.error);

      Serial.print("Output: ");
      Serial.println(data.output);

      Serial.print("Baseline: ");
      Serial.println(data.baseline);

      Serial.print("Motor 1: ");
      Serial.println(data.motor_1);

      Serial.print("Motor 2: ");
      Serial.println(data.motor_2);

      Serial.print("Lift: ");
      Serial.println(data.lift);

      Serial.print("Drag: ");
      Serial.println(data.drag);

      Serial.print("Pitch: ");
      Serial.println(data.pitch);

      Serial.print("Ampere: ");
      Serial.println(data.ampere);

      Serial.print("Voltage: ");
      Serial.println(data.voltage);

      Serial.print("Wattage: ");
      Serial.println(data.wattage);

      Serial.print("mAh Used: ");
      Serial.println(data.mah_used);

      Serial.print("Status: ");
      Serial.println(data.status);

      Serial.print("Mode: ");
      Serial.println(data.mode);

      Serial.print("Time Seconds: ");
      Serial.println(data.time_seconds);

      Serial.print("P_out: ");
      Serial.println(data.p_out);

      Serial.print("I_out: ");
      Serial.println(data.i_out);

      Serial.print("D_out: ");
      Serial.println(data.d_out);

      Serial.print("P_gain: ");
      Serial.println(data.p_gain);

      Serial.print("I_gain: ");
      Serial.println(data.i_gain);

      Serial.print("D_gain: ");
      Serial.println(data.d_gain);

      show_display();
      Serial.print("status: ");
      Serial.println(status);
    }
    return;
  }
}


void clearSerialBuffer(Stream &serial) {
  while (serial.available() > 0) {
    serial.read();  // Discard all incoming data
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

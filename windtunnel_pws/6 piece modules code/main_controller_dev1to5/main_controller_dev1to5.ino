#include <SoftwareSerial.h>
#include <ESP32Servo.h>

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
int time_seconds;
float baseline;
int motor_1;
int motor_2;

float p_out;
float i_out;
float d_out;

int system_status;
String status = "";
String mode = "";
#pragma pack(1)
// Define data structures for each device
struct Config_data {
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

struct config_loadcell_ina219 {
  float calibrationValue_Lift;
  float calibrationValue_Drag;
  float calibrationValue_Ampere;
};

struct config_pid_controller {
  float calibrationValue_Airspeed;
  int pid_loop_hz;

  float delta_max;
  float deadband;

  float p_gain;
  float i_gain;
  float d_gain;

  int windspeedCount;
  float windspeedList[10];
};

struct SDcard_log_data {
  bool log_data;
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



struct Messurment_data {
  float lift_loadcell;
  float drag_loadcell;
  float ampere;
  float voltage;
  float wattage;
  float mah_used;
};


struct pid_controller_data {
  int system_status;
  float setpoint;
  float airspeed;
  float error;
  float output;
  float baseline;
  float p_out;
  float i_out;
  float d_out;
};

struct setpoint_data {
  float setpoint;
};

struct Display_data {
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
  String mode;
  int time_seconds;
  float p_out;
  float i_out;
  float d_out;
  float p_gain;
  float i_gain;
  float d_gain;
};
#pragma pack()

Config_data sd_data;
Messurment_data meassurment_data;

pid_controller_data pid_data;

#define TX_PIN 21  // TX pin for HardwareSerial
#define RX_PIN 20  // RX pin for HardwareSerial

// Configure HardwareSerial with custom TX and RX pins
HardwareSerial sd_card(0);               // Create a new HardwareSerial instance
SoftwareSerial pid_controller(6, 5);     // RX, TX pins
SoftwareSerial meassuring_device(4, 3);  // RX, TX pins 3,4,5,6,7  en 21 en 20 werken voor serial com

SoftwareSerial display(8, 7);  // RX, TX pins 7 = tx display only recieves data pin 8 is not connected

unsigned long previousMillis = 0;  // Store last loop time
float loopFrequency = 0;           // Store loop frequency

unsigned long loopDuration = 0;
int servo_pin = 10;
Servo servo;
int count_display = 0;
int count_logger = 0;
template<typename T>
void waitForData(Stream &serial, T &data, unsigned long max_wait_time_ms, const char *deviceName) {
  unsigned long start_time = millis();

  while ((millis() - start_time) < max_wait_time_ms) {
    if (receiveData(serial, data)) {
      sendAcknowledgment(serial, "ACK");
      if (deviceName == "sd_card") {

        calibrationValue_Lift = sd_data.calibrationValue_Lift;
        calibrationValue_Drag = sd_data.calibrationValue_Drag;
        calibrationValue_Ampere = sd_data.calibrationValue_Ampere;
        calibrationValue_Airspeed = sd_data.calibrationValue_Airspeed;
        pid_loop_hz = sd_data.pid_loop_hz;
        measurement_hz = sd_data.measurement_hz;
        screen_hz = sd_data.screen_hz;
        p_gain = sd_data.p_gain;
        i_gain = sd_data.i_gain;
        d_gain = sd_data.d_gain;
        delta_max = sd_data.delta_max;
        deadband = sd_data.deadband;
        start_angle = sd_data.start_angle;
        end_angle = sd_data.end_angle;
        windspeedCount = sd_data.windspeedCount;
        time_step = sd_data.time_step;
        angle_steps = sd_data.angle_steps;
        total_steps = sd_data.total_steps;
        test_fileName = sd_data.test_fileName;
        speed = sd_data.speed;
        angle = sd_data.angle;
        time_during_test = sd_data.time_during_test;
        test_datum = sd_data.test_datum;
        memcpy(windspeedList, sd_data.windspeedList, sizeof(windspeedList));

        Serial.println("Data received:");
        Serial.print("Calibration Lift: ");
        Serial.println(calibrationValue_Lift);
        Serial.print("Calibration Drag: ");
        Serial.println(calibrationValue_Drag);
        Serial.print("Calibration Ampere: ");
        Serial.println(calibrationValue_Ampere);
        Serial.print("Calibration Airspeed: ");
        Serial.println(calibrationValue_Airspeed);
        Serial.print("PID Loop Hz: ");
        Serial.println(pid_loop_hz);
        Serial.print("Measurement Hz: ");
        Serial.println(measurement_hz);
        Serial.print("Screen Hz: ");
        Serial.println(screen_hz);
        Serial.print("P Gain: ");
        Serial.println(p_gain);
        Serial.print("I Gain: ");
        Serial.println(i_gain);
        Serial.print("D Gain: ");
        Serial.println(d_gain);
        Serial.print("Delta Max: ");
        Serial.println(delta_max);
        Serial.print("Deadband: ");
        Serial.println(deadband);
        Serial.print("Start Angle: ");
        Serial.println(start_angle);
        Serial.print("End Angle: ");
        Serial.println(end_angle);
        Serial.print("Time Step: ");
        Serial.println(time_step);
        Serial.print("Angle Steps: ");
        Serial.println(angle_steps);
        Serial.print("Total Steps: ");
        Serial.println(total_steps);
        Serial.print("Test File Name: ");
        Serial.println(test_fileName);
        Serial.print("Speed: ");
        Serial.println(speed);
        Serial.print("Angle: ");
        Serial.println(angle);
        Serial.print("Time During Test: ");
        Serial.println(time_during_test);
        Serial.print("Test Datum: ");
        Serial.println(test_datum);
        Serial.print("Windspeed recieved:");
        for (int i = 0; i < windspeedCount; i++) {
          Serial.println(windspeedList[i]);
        }


      } else if (deviceName == "meassuring_device") {
        lift_loadcell = meassurment_data.lift_loadcell;
        drag_loadcell = meassurment_data.drag_loadcell;

        ampere = meassurment_data.ampere;
        voltage = meassurment_data.voltage;
        wattage = meassurment_data.wattage;
        mah_used = meassurment_data.mah_used;
        /*
        Serial.print("Data received:");
        Serial.print("Lift Loadcell: ");
        Serial.print(lift_loadcell);
        Serial.print("\t");
        Serial.print("Drag Loadcell: ");
        Serial.print(drag_loadcell);
        Serial.print("\t");
        Serial.print("Voltage: ");
        Serial.print(voltage);
        Serial.print("\t");
        Serial.print("Ampere: ");
        Serial.print(ampere);
        Serial.print("\t");
        Serial.print("Wattage: ");
        Serial.print(wattage);
        Serial.print("\t");
        Serial.print("mAh Used: ");
        Serial.print(mah_used);
        Serial.print("\n");
*/
      } else if (deviceName == "pid_controller") {
        system_status = pid_data.system_status;
        Serial.print("System Status: ");
        Serial.println(system_status);

        setpoint = pid_data.setpoint;
        Serial.print("Setpoint: ");
        Serial.println(setpoint);

        airspeed = pid_data.airspeed;
        Serial.print("Airspeed: ");
        Serial.println(airspeed);

        error = pid_data.error;
        Serial.print("Error: ");
        Serial.println(error);

        output = pid_data.output;
        Serial.print("Output: ");
        Serial.println(output);

        baseline = pid_data.baseline;
        Serial.print("Baseline: ");
        Serial.println(baseline);

        p_out = pid_data.p_out;
        Serial.print("P_out: ");
        Serial.println(p_out);

        i_out = pid_data.i_out;
        Serial.print("I_out: ");
        Serial.println(i_out);

        d_out = pid_data.d_out;
        Serial.print("D_out: ");
        Serial.println(d_out);
      }

      return;
    }
  }

  Serial.print("Failed to receive data from ");
  Serial.print(deviceName);
  Serial.println(" within the time limit.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("serial port open!");
  sd_card.begin(115200);
  pid_controller.begin(9600);
  meassuring_device.begin(9600);
  //pid_angle_motor.begin(9600);
  display.begin(9600);
  unsigned long start_check = millis();
  // Handshake with each device

  ESP32PWM::allocateTimer(0);  // set timers for the pwm signals
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);  // standard 50 hz esc signal
  servo.attach(servo_pin, 1000, 2000);

  if (!performHandshake(sd_card, 5000)) {
    Serial.println("sd_card communication failed.");
  }
  Serial.print("time it took to get a handshake with sd_card: ");
  Serial.println(millis() - start_check);
  start_check = millis();
  meassuring_device.listen();  // Switch to meassuring_device before communication
  if (!performHandshake(meassuring_device, 5000)) {
    Serial.println("meassuring_device communication failed.");
  }

  Serial.print("time it took to get a handshake with meassuring device: ");
  Serial.println(millis() - start_check);

  start_check = millis();
  // pid_controller.listen(); // Switch to pid_controller before communication
  if (!performHandshake(pid_controller, 5000)) {
    Serial.println("pid_controller communication failed.");
  }

  Serial.print("time it took to get a handshake with pid_controller: ");
  Serial.println(millis() - start_check);


  sendCommand(sd_card, "GET", "sd_card");
  waitForData(sd_card, sd_data, 2000, "sd_card");

  Serial.println("cal_lift: ");
  Serial.println(calibrationValue_Lift);
  Serial.println("cal_drag: ");
  Serial.println(calibrationValue_Drag);
  Serial.println("cal_amper: ");
  Serial.println(calibrationValue_Ampere);

  sendCommand(meassuring_device, "SET", "meassuring device");
  send_measuring_device_conf_data();

  sendCommand(pid_controller, "SET", "pid_controller");
  // send(setpoint)  // add the send setpoint function that sends a number or a wordt so the pid contorller can change it,
  // also add the mode as: testing and not testing, and sending conf etc
  // add the setpoint list in the conf
  send_pid_controller_conf_data();
  Serial.println("All handshakes complete. Waiting for data...");
while(!receiveAcknowledgment(pid_controller, "ACK"){
    sendCommand(pid_controller, "armed", "pid_controller");
    delay(500);
}
mode = "pid controller armed";
}

void loop() {
  if (millis() - previousMillis >= 200) {
    loopDuration = millis() - previousMillis;
    loopFrequency = 1000 / loopDuration;
    previousMillis = millis();
    sendCommand(pid_controller, "GET", "pid_controller");
    waitForData(pid_controller, pid_data, 50, "pid_controller");

    sendCommand(meassuring_device, "GET", "measuring_device");
    waitForData(meassuring_device, meassurment_data, 50, "meassuring_device");
    if (system_status == 1) {

      int speed_step = 1;
      // send_setpoint(pid_controller, speed_step);

      float angle = 23.34;
      pitch = angle;
      command_angle_motor(angle);

      // 5hz -->
      if (count_display >= 5) {
        count_display = 0;
        send_display_data();
      }
      count_display++;


      send_datalogger();

      Serial.print("loopfrequency :  ");
      Serial.print(loopFrequency);
      Serial.print("status :  ");
      Serial.print(system_status);
      Serial.println(" !");

      // Add additional processing for the received data here
    }
  }
  if(system_status == 0){

  }
}
void send_datalogger() {
  looptime = millis();

  SDcard_log_data datatosend{
    setpoint,
    airspeed,
    error,
    output,
    lift_loadcell,
    drag_loadcell,
    pitch,
    ampere,
    voltage,
    wattage,
    mah_used,
    looptime,
    loop_number
  };

  sendDataWithRetry(sd_card, datatosend, 50, 30);
}
void send_setpoint(Stream &serial, int i) {
  float setpoint_ = windspeedList[i];
  setpoint_data data{
    setpoint_
  };
  sendDataWithRetry(serial, data, 50, 10);
}

void send_measuring_device_conf_data() {
  config_loadcell_ina219 datatosend{
    calibrationValue_Lift,
    calibrationValue_Drag,
    calibrationValue_Ampere
  };
  sendDataWithRetry(meassuring_device, datatosend, 50, 10);
}

void command_angle_motor(float angle) {
  float pulsewidth = map(angle, -45, 45, 1000, 2000);
  servo.writeMicroseconds(pulsewidth);
}

void send_display_data() {
  float motor_1 = output;
  float motor_2 = output;
  time_seconds = millis() / 1000;
  Display_data datatosend{
    test_fileName,
    setpoint,
    airspeed,
    error,
    output,
    baseline,
    motor_1,
    motor_2,
    lift_loadcell,
    drag_loadcell,
    pitch,
    ampere,
    voltage,
    wattage,
    mah_used,
    system_status,
    mode,
    time_seconds,
    p_out,
    i_out,
    d_out,
    p_gain,
    i_gain,
    d_gain
  };
  sendData(display, datatosend);
}
void send_pid_controller_conf_data() {
  config_pid_controller datatosend{
    calibrationValue_Airspeed,
    pid_loop_hz,
    delta_max,
    deadband,
    p_gain,
    i_gain,
    d_gain,
    windspeedCount,
    windspeedList
  };
  sendDataWithRetry(pid_controller, datatosend, 50, 10);
}
// Serialization and deserialization functions for each device
template<typename T>
void deserializeData(const uint8_t *buffer, T &data) {
  memcpy(&data, buffer, sizeof(T));
}

template<typename T>
bool receiveData(Stream &serial, T &data) {
  if (serial.available() >= sizeof(T)) {
    uint8_t buffer[sizeof(T)];
    serial.readBytes(buffer, sizeof(buffer));
    deserializeData(buffer, data);
    return true;
  }
  return false;
}
template<typename T>
void serializeData(T &data, uint8_t *buffer) {
  memcpy(buffer, &data, sizeof(T));
}
template<typename T>
void sendData(Stream &serial, T &data) {
  uint8_t buffer[sizeof(T)];
  serializeData(data, buffer);
  serial.write(buffer, sizeof(buffer));
}

bool receiveAcknowledgment(Stream &serial, const char *expectedAck) {
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


template<typename T>
void sendDataWithRetry(Stream &serial, T &data, unsigned long max_wait_time_ms, unsigned long retry_interval_ms) {
  unsigned long start_time = millis();
  bool ackReceived = false;

  while ((millis() - start_time) < max_wait_time_ms && !ackReceived) {
    // Send data
    sendData(serial, data);
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

void sendAcknowledgment(Stream &serial, const char *ackMessage) {
  serial.write(ackMessage, strlen(ackMessage));
}


bool performHandshake(Stream &serial, unsigned long timeout_ms) {
  unsigned long start_time = millis();

  clearSerialBuffer(serial);  // Clear the buffer before starting handshake

  // Wait for the ping message within the specified timeout
  while (millis() - start_time < timeout_ms) {
    if (serial.available() > 0) {
      char ping[5];  // Adjusted to 5 to ensure enough room for null termination
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
void sendCommand(Stream &serial, const char *command, String name) {
  // Send the "GET" command
  serial.write(command, strlen(command));
  // Serial.print(command);
  // Serial.print(" command sent to:");
  // Serial.println(name);
}

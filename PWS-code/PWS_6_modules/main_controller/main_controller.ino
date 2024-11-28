#include <SoftwareSerial.h>
#include <ESP32Servo.h>

float rev_airspeed;

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

int system_status = 0;
String status = "";
int mode;
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
/*
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
*/

struct Display_data {
  float setpoint;
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

unsigned long prev_millis = 0;  // Store last loop time
unsigned long loopDuration = 0;
int servo_pin = 10;
Servo servo;
int angle_pin = 9;
int count_display = 0;
int count_logger = 0;

float recieved_angle;
bool executed = false;

const int killswitch_pin_notpressed = 8;  // killswitch pin
volatile bool kill_switch_status = LOW;   // System initially off
volatile unsigned long last_interrupt_time = 0;

unsigned long prevMillisTest = 0;
unsigned long prevMillisUpdate = 0;
unsigned long prevMillisDisplay = 0;

const unsigned long updateInterval = 200; // 200 ms update interval
const unsigned long pid_wait_Duration = 20000; // 7500 ms per test step


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
        // system_status = pid_data.system_status;
        setpoint = pid_data.setpoint;
        rev_airspeed = pid_data.airspeed;
        error = pid_data.error;
        output = pid_data.output;
        baseline = pid_data.baseline;
        p_out = pid_data.p_out;
        i_out = pid_data.i_out;
        d_out = pid_data.d_out;
        /*
        Serial.print("System Status: ");
        Serial.println(system_status);


        Serial.print("Setpoint: ");
        Serial.println(setpoint);


        Serial.print("Airspeed: ");
        Serial.println(airspeed);


        Serial.print("Error: ");
        Serial.println(error);


        Serial.print("Output: ");
        Serial.println(output);

        Serial.print("Baseline: ");
        Serial.println(baseline);


        Serial.print("P_out: ");
        Serial.println(p_out);


        Serial.print("I_out: ");
        Serial.println(i_out);


        Serial.print("D_out: ");
        Serial.println(d_out);
        */
      }

      return;
    }
  }

  //Serial.print("Failed to receive data from ");
 // Serial.print(deviceName);
 // Serial.println(" within the time limit.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("serial port open!");
  sd_card.begin(115200);
  pid_controller.begin(9600);
  meassuring_device.begin(9600);
  display.begin(9600);
  unsigned long start_check = millis();
  // Handshake with each device

  ESP32PWM::allocateTimer(0);  // set timers for the pwm signals
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);  // standard 50 hz esc signal
  servo.attach(servo_pin, 500, 2500);
  pinMode(angle_pin, INPUT);
  pinMode(killswitch_pin_notpressed, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(killswitch_pin_notpressed), killswitch, RISING);

  command_angle_motor(0);

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

  if (!performHandshake(pid_controller, 5000)) {
    Serial.println("pid_controller communication failed.");
  }

  Serial.print("time it took to get a handshake with pid_controller: ");
  Serial.println(millis() - start_check);


  sendCommand(sd_card, "GET", "sd_card");
  waitForData(sd_card, sd_data, 2000, "sd_card");

  sendCommand(meassuring_device, "SET", "meassuring device");
  send_measuring_device_conf_data();

  sendCommand(pid_controller, "SET", "pid_controller");

  send_pid_controller_conf_data();
  Serial.println("All handshakes complete. Waiting for data...");
  while (!receiveAcknowledgment(pid_controller, "ACK")) {
    sendCommand(pid_controller, "armed", "pid_controller");
    delay(500);
    Serial.println("sending armed command");
  }
  mode = 1;  // "pid controller armed"
  Serial.println("entering loop");
}

void loop() {
// Add necessary state variables

if (kill_switch_status == HIGH) { // Test active
    mode = 2; // "testing active"
    system_status = 1;
    //Serial.println("System on, running the test --->");

    for (int i = 0; i < total_steps; i++) {
        setpoint = windspeedList[i];
        if (kill_switch_status == LOW) break;

        send_setpoint(pid_controller, i);
        
       // Serial.println(setpoint);
        recieved_angle = read_target_from_pwm();
        //Serial.println(recieved_angle);

        // Adjust motor until the angle is within tolerance
        while (abs(start_angle - recieved_angle) > 0.5) {
            command_angle_motor(0);
            recieved_angle = read_target_from_pwm();
          //  Serial.print("Received angle:");
            //Serial.println(recieved_angle);
            if (millis() - prevMillisUpdate >= updateInterval) {
                prevMillisUpdate = millis();
                // Allow for other operations during angle adjustment
                if (kill_switch_status == LOW) break;
            }
        }

        prevMillisTest = millis();
        while (millis() - prevMillisTest <= pid_wait_Duration) {
            if (kill_switch_status == LOW) break;

            if (millis() - prevMillisUpdate >= updateInterval) {
                 prevMillisUpdate = millis();
                if (kill_switch_status == LOW) break;

                 // Send and receive PID data
                 //sendCommand(pid_controller, "GET", "pid_controller");
                // waitForData(pid_controller, pid_data, 90, "pid_controller");
                

                 // Send and receive measuring device data
                 sendCommand(meassuring_device, "GET", "meassuring_device");
                 waitForData(meassuring_device, meassurment_data, 90, "meassuring_device");

                 command_angle_motor(0);
                 pitch = read_target_from_pwm();
                 Serial.print("Waiting for PID,");
                 Serial.print("\t");
                 // Serial.print("current_time, ");
                 Serial.print(millis() / 1000);
                 Serial.print(",\t");

                // Serial.print("setpoint, ");
                 Serial.print(setpoint);
                 Serial.print(",\t");

                 //Serial.print("lift, ");
                 Serial.print(lift_loadcell);
                 Serial.print(",\t");

                // Serial.print("drag; ");
                 Serial.print(drag_loadcell);
                 Serial.print(",\t");

            //     Serial.print("pitch; ");
                 Serial.print(pitch);
                 Serial.print(",\t");

                // Serial.print("ampere; ");
                 Serial.print(ampere);
                 Serial.print(",\t");

                // Serial.print("voltage; ");
                 Serial.print(voltage);
                 Serial.print(",\t");

                 //Serial.print("wattage; ");
                 Serial.print(wattage);
                 Serial.print(",\t");

               //  Serial.print("mah_used; ");
                 Serial.print(mah_used);
                 Serial.print("\n");

                // Update display data every 4 cycles
                 if (count_display >= 4) {
                   count_display = 0;
                   send_display_data();
                }
                count_display++;

                // Log data
               // send_datalogger();
            }
        }

        // Perform angle sweep
        static float currentAngle = start_angle;
        while (currentAngle <= end_angle) {
            if (kill_switch_status == LOW) break;


            // Wait for the next step
            if (millis() - prevMillisUpdate >= updateInterval) {
                prevMillisUpdate = millis();

               // Serial.print("Angle:");
                //Serial.println(currentAngle);

            // Perform step actions
           // sendCommand(pid_controller, "GET", "pid_controller");
          //  waitForData(pid_controller, pid_data, 90, "pid_controller");

            sendCommand(meassuring_device, "GET", "meassuring_device");
            waitForData(meassuring_device, meassurment_data, 90, "meassuring_device");

            command_angle_motor(currentAngle);
            pitch = read_target_from_pwm();

                 Serial.print("Testing,");
                 Serial.print("\t");
                 // Serial.print("current_time, ");
                 Serial.print(millis() / 1000);
                 Serial.print(",\t");

                // Serial.print("setpoint, ");
                 Serial.print(setpoint);
                 Serial.print(",\t");

                 //Serial.print("lift, ");
                 Serial.print(lift_loadcell);
                 Serial.print(",\t");

                // Serial.print("drag; ");
                 Serial.print(drag_loadcell);
                 Serial.print(",\t");

            //     Serial.print("pitch; ");
                 Serial.print(pitch);
                 Serial.print(",\t");

                // Serial.print("ampere; ");
                 Serial.print(ampere);
                 Serial.print(",\t");

                // Serial.print("voltage; ");
                 Serial.print(voltage);
                 Serial.print(",\t");

                 //Serial.print("wattage; ");
                 Serial.print(wattage);
                 Serial.print(",\t");

               //  Serial.print("mah_used; ");
                 Serial.print(mah_used);
                 Serial.print("\n");

                // Update display data every 4 cycles
                 if (count_display >= 4) {
                   count_display = 0;
                   send_display_data();
                }
                count_display++;
            // Log data
          //  send_datalogger();

              currentAngle += 0.2; // Increment angle after delay
            }
        }

        currentAngle = start_angle; // Reset angle for the next step
    }

    // Test is done, set all values to 0 and turn off
    test_is_done();
}

if (kill_switch_status == LOW) { // Armed but not active
    mode = 3; // "armed (waiting)"
    system_status = 0;

    if (millis() - prevMillisUpdate >= updateInterval) {
        prevMillisUpdate = millis();

        // Send and receive PID data
       // sendCommand(pid_controller, "GET", "pid_controller");
        //waitForData(pid_controller, pid_data, 100, "pid_controller");

        // Send and receive measuring device data
        sendCommand(meassuring_device, "GET", "meassuring_device");
        waitForData(meassuring_device, meassurment_data, 100, "meassuring_device");

        command_angle_motor(0);

        // Update display data every 4 cycles
       // if (count_display >= 4) {
        //    count_display = 0;
        //    send_display_data();
        //    Serial.println("System off");
        //}
        //count_display++;
    }
}

}

void test_is_done() {
  send_setpoint(pid_controller, -1);
  command_angle_motor(0);
  kill_switch_status = LOW;
  count_display = 4;
  system_status = 0;
}
void send_datalogger() {
  looptime = millis();

  SDcard_log_data datatosend{
    setpoint,
    rev_airspeed,
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

void send_measuring_device_conf_data() {
  config_loadcell_ina219 datatosend{
    calibrationValue_Lift,
    calibrationValue_Drag,
    calibrationValue_Ampere
  };
  sendDataWithRetry(meassuring_device, datatosend, 50, 30);
}

void command_angle_motor(float angle) {
  angle *= 100;
  unsigned long pulsewidth = map(angle, 0, 4500, 500, 2500);
  //Serial.println(pulsewidth);
  servo.writeMicroseconds(pulsewidth);
}

float read_target_from_pwm() {
  // Read the pulse width in microseconds
  unsigned long pulseWidth = pulseIn(angle_pin, HIGH);
  Serial.print(pulseWidth);
  Serial.print("\t");

  float angle = map(pulseWidth, 100, 500, 0, 4500);
  angle /= 100;
  return angle;
}
void send_setpoint(Stream &serial, int i) {
  const char *command;
  if (i == -1) {
    command = "fff";
  } else if (i == 0) {
    command = "s:0";
  } else if (i == 1) {
    command = "s:1";
  } else if (i == 2) {
    command = "s:2";
  } else if (i == 3) {
    command = "s:3";
  } else if (i == 4) {
    command = "s:4";
  } else if (i == 5) {
    command = "s:5";
  } else if (i == 6) {
    command = "s:6";
  } else if (i == 7) {
    command = "s:7";
  } else if (i == 8) {
    command = "s:8";
  } else if (i == 9) {
    command = "s:9";
  }
  // Send the "...." command
  serial.write(command, strlen(command));
}
void send_display_data() {
  /*
  float motor_1 = output;
  float motor_2 = output;
  time_seconds = millis() / 1000;
  Display_data datatosend{
    test_fileName,
    setpoint,
    rev_airspeed,
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
  */
   time_seconds = millis() / 1000;
   Display_data datatosend{
    setpoint,
    lift_loadcell,
    drag_loadcell,
    pitch,
    ampere,
    voltage,
    wattage,
    mah_used,
    system_status,
    mode,
    time_seconds
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
    windspeedCount
  };
  memcpy(datatosend.windspeedList, windspeedList, sizeof(windspeedList));
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
    Serial.println(ack);
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
void killswitch() {
  unsigned long interrupt_time = millis();
  // Debounce: Ignore interrupt if triggered within the last 200ms
  if (interrupt_time - last_interrupt_time > 200) {
    // Toggle the system state
    kill_switch_status = !kill_switch_status;
    Serial.print("killswitch pressed!!! new status: ");
    Serial.println(kill_switch_status);
    // Motor control based on the new state
    if (kill_switch_status == LOW) {  // kill the motors
                                      // reset the test loop so: return to beginn
    }
    if (kill_switch_status == HIGH) {
    }
    // Update the last interrupt time
    last_interrupt_time = interrupt_time;
  }
}
void sendCommand(Stream &serial, const char *command, String name) {
  // Send the "GET" command
  serial.write(command, strlen(command));
  // Serial.print(command);
  // Serial.print(" command sent to:");
  // Serial.println(name);
}

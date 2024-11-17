#include <Wire.h>
#include <ESP32Servo.h>
#pragma pack(1)
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

struct pid_controller_data {
  int status;
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
#pragma pack()
config_pid_controller data_recv;
setpoint_data setpoint_recv;

int pid_loop_hz;
int status;
const int killswitch_pin_pressed = 10;  // killswitch pin
volatile bool system_status = LOW;      // System initially off
volatile unsigned long last_interrupt_time = 0;

float interval;  // loop time
volatile unsigned long previous_time = 0;

// || constants for the motors ||
Servo motor1;
Servo motor2;

int motor_signal1;
int motor_signal2;

const int motorPin1 = 4;
const int motorPin2 = 5;

const int minPulseWidth = 1000;  // Minimum pulse width in microseconds (1 ms)
const int maxPulseWidth = 2000;  // Maximum pulse width in microseconds (2 ms)

float setpoint = 0;
float airspeed;
float aprox_motor_signal = 1000;
const float airspeed_to_motor_signal = 29.11;
// || every constant for the windspeed meter ||
const int sensorAddress = 0x28;  // Address of MS4525 airspeed sensor
const float rho = 1.225;         // Air density in kg/m^3 (replace with actual value for your conditions)
float zeroAirspeedPressure = 0;  // Raw pressure at zero airspeed
float slope = 1.0;
const int bufferSize = 50;         // Number of readings for moving average
float pressureBuffer[bufferSize];  // Buffer to store pressure readings
int bufferIndex = 0;               // Index for the buffer

const int airspeedBufferSize = 50;
float airspeedBuffer[airspeedBufferSize];
int airspeedBufferIndex = 0;
float correction_value = 1.903185767;

float x_k = 0.0;  // Estimated state (filtered airspeed)
float P_k = 1.0;  // Estimated error covariance
float Q = 0.005;  // Process noise (small, as the airspeed is expected to change smoothly)
float R = 5;      // Measurement noise (based on 0.2 m/s uncertainty)
float K_k = 0.0;  // Kalman gain
float z_k = 0.0;  // Measurement (airspeed)

// || pid value's ||

// PID constants
/* example pid constats:
float Kp = 1.0;
float Ki = 0.05;
float Kd = 0.01;
*/
//test constatnts:


float baseline_motor_signal = 1000;  // Initial baseline motor signal (can start at minPulseWidth)
float previous_output = 1000;
bool is_in_deadband = false;        // Track if the system is currently in the deadband
float baseline_adjust_rate = 0.05;  // Rate at which the baseline is adjusted when outside deadband

float Kp = 28.54;
float Ki = 0.09;
float Kd = 38.54;

float previousError = 0;
float integral;
float derivative;

float error;
float Pout;
float Iout;
float Dout;

float output_offset;
float output;

float airspeed_filtered;
float deadband = 0.001;


float maxDelta = 5;  // Maximum allowed change in output per loop (in microseconds)
float pid_output;
float delta;

int windspeedCount;
float windspeedList[10];

volatile int avrg_count = 0;
float avrg_airspeed;
bool main_controller_status = false;
HardwareSerial main_controller(0);  //Create a new HardwareSerial class.

int set = 0;
int led_pin = 8;
void setup() {
  Serial.begin(115200);
  main_controller.begin(9600);
  while (!main_controller) { delay(10); }

  Wire.begin();

  ESP32PWM::allocateTimer(0);  // set timers for the pwm signals
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor1.setPeriodHertz(50);  // standard 50 hz esc signal
  motor2.setPeriodHertz(50);  // standard 50 hz esc signal
  motor1.attach(motorPin1, 1000, 2000);
  motor2.attach(motorPin2, 1000, 2000);

  delay(3000);
  performHandshake(main_controller, 5000);
  handleSetCommand(main_controller, data_recv);

  pinMode(killswitch_pin_pressed, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(killswitch_pin_pressed), killswitch, FALLING);



  hz_inteval(pid_loop_hz);  //convertes hz to looptime
  // Calibrate the sensor
  // calibrate();
  while (system_status == HIGH) {
    delay(100);
    digitalWrite(led_pin, HIGH);
    Serial.println("system status high");
  }
  digitalWrite(led_pin, LOW);
  Serial.println("entering the loop --- >");
  Serial.println(system_status);

  while (!main_controller_status) {
    handleCommand(main_controller);
    Serial.println("waiting");
  }
}

void loop() {
  handleCommand(main_controller);
  unsigned long current_time = millis();

  if (system_status == HIGH && main_controller_status == HIGH) {
    digitalWrite(led_pin, HIGH);
    status = 1;
    setpoint = windspeedList[set];
    pid_controller_data data_send{
      status,
      setpoint,
      avrg_airspeed,
      error,
      output,
      baseline_motor_signal,
      Pout,
      Iout,
      Dout,
    };
    handleGetCommand(main_controller, data_send);

    if (current_time - previous_time >= interval) {
      previous_time = millis();
      Serial.println("system is on!-->>>>>");
      Serial.println(setpoint);
      //Read raw pressure values from sensor

      // airspeed = calc_airspeed_moving_filter();
      // Serial.print(airspeed);
      // Serial.println(",");

      //float airspeed_kalman = calc_airspeed_kalman_filter();  // kalman
      //  airspeed_filtered = filtered_airspeed();  // kalman + moving filter
      airspeed_filtered = 0;
      avrg_count++;
      avrg_airspeed += airspeed_filtered;

      avrg_airspeed / avrg_count;
      //Serial.print(airspeed_kalman);
      // Serial.print(",");
      Serial.print(airspeed_filtered);
      Serial.println(",");

      //command_motors(1200);
      //airspeed_pid(airspeed, setpoint); // only pid algoritm
      moving_baseline_pid(airspeed_filtered, windspeedList[set]);  // automatic baseline finder + normal pid
    }
  }
  if (system_status == LOW) {
    if (current_time - previous_time >= 1000) {
      previous_time = millis();
      Serial.println("system is off");
    }
    digitalWrite(led_pin, LOW);
    //airspeed_filtered = filtered_airspeed();  // kalman + moving filter
    airspeed_filtered = 0;
    avrg_count++;
    avrg_airspeed += airspeed_filtered;

    avrg_airspeed / avrg_count;

    status = 0;
    pid_controller_data data_send{
      status,
      setpoint,
      avrg_airspeed,
      error,
      output,
      baseline_motor_signal,
      Pout,
      Iout,
      Dout,
    };
    handleGetCommand(main_controller, data_send);

    // Serial.println(status);
    if (Serial.available()) {
      // Read the input as a string
      String input = Serial.readStringUntil('\n');
      input.trim();  // Remove extra spaces and newlines

      // Handle "set" command
      if (input.equalsIgnoreCase("set")) {
        setPIDValues();
      } else if (input.equalsIgnoreCase("setpoint")) {
        Serial.print("Enter new setpoint value: ");
        while (!Serial.available())
          ;                              // Wait for user input
        setpoint = Serial.parseFloat();  // Read float value from the serial monitor
        Serial.println(setpoint);
        clearSerialBuffer(Serial);  // Clear any leftover characters
        delay(200);
      }
      // Handle "dump" command
      else if (input.equalsIgnoreCase("dump")) {
        dumpPIDValues();
      }
    }


    motor1.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
    motor2.writeMicroseconds(minPulseWidth);  //set to 1000us puls length
  }
}


void hz_inteval(int loop_freqenty) {
  interval = 1000 / loop_freqenty;
}
//float calc_unfiltered_airspeed()
float calc_airspeed_moving_filter() {
  int16_t rawPressure = readRawPressure();




  // Update the buffer with the new reading
  pressureBuffer[bufferIndex] = rawPressure;
  bufferIndex = (bufferIndex + 1) % bufferSize;
  // Calculate the moving average
  float sumPressure = 0.0;
  for (int i = 0; i < bufferSize; i++) {
    sumPressure += pressureBuffer[i];
    //Serial.print(pressureBuffer[i]);
    //Serial.print("\n");
  }
  float averagePressure = sumPressure / bufferSize;




  // Calculate corrected differential pressure and make it absolute (no negative values)
  float correctedPressure = abs(slope * (averagePressure - zeroAirspeedPressure));




  // Convert pressure to airspeed using Bernoulli's equation
  float airspeed = sqrt(2 * correctedPressure / rho);
  return airspeed;
}
float calc_airspeed_kalman_filter() {
  int16_t rawPressure = readRawPressure();


  float correctedPressure = abs(rawPressure - zeroAirspeedPressure);
  //float correctedPressure = rawPressure;


  float airspeed = sqrt(2 * correctedPressure / rho);


  z_k = airspeed;  // Replace with your airspeed sensor reading function


  // Step 2: Prediction step
  P_k = P_k + Q;  // Predicted error covariance


  // Step 3: Update step
  // Compute Kalman gain
  K_k = P_k / (P_k + R);


  // Update estimate with measurement
  x_k = x_k + K_k * (z_k - x_k);


  // Update error covariance
  P_k = (1 - K_k) * P_k;


  // Serial.print("Measured Airspeed: ");
  // Serial.print(z_k);
  //Serial.print(" | Filtered Airspeed: ");
  //Serial.println(x_k);


  return x_k;
}


float filtered_airspeed() {
  airspeed = calc_airspeed_kalman_filter();


  airspeedBuffer[airspeedBufferIndex] = airspeed;
  airspeedBufferIndex = (airspeedBufferIndex + 1) % airspeedBufferSize;


  float sumAirspeed = 0.0;
  for (int i = 0; i < airspeedBufferSize; i++) {
    sumAirspeed += airspeedBuffer[i];
    //Serial.print(pressureBuffer[i]);
    //Serial.print("\n");
  }


  float average_airspeed = sumAirspeed / airspeedBufferSize;
  float corrected_airspeed = average_airspeed / correction_value;
  return corrected_airspeed;
}




int16_t readRawPressure() {
  Wire.requestFrom(sensorAddress, 2);  // Request 2 bytes from sensor
  while (Wire.available() < 2)
    ;                                                   // Wait for 2 bytes to be available
  int16_t pressure = (Wire.read() << 8) | Wire.read();  // Combine two bytes into an int16_t
  return pressure;
}




void calibrate() {
  // Perform calibration with no airflow (zero airspeed)
  if (zeroAirspeedPressure == 0) {
    Serial.println("Calibrating... ensure no airflow.");

    float sumPressure = 0.0;
    const int numReadings = 100;

    // Take multiple readings to get an average zero pressure
    for (int i = 0; i < numReadings; i++) {
      sumPressure += readRawPressure();
      delay(50);  // Small delay between readings
    }




    zeroAirspeedPressure = sumPressure / numReadings;
    Serial.print("Zero airspeed pressure: ");
    Serial.println(zeroAirspeedPressure);




    // Perform slope calibration
    slope = 1.0;  // Example slope factor (replace with actual value)
    Serial.print("Slope calibration factor: ");
    Serial.println(slope);
    delay(1000);




    for (int i = 0; i < bufferSize; i++) {
      pressureBuffer[i] = zeroAirspeedPressure;
    }
    // fill buffer with the average pressure
  }
}
void killswitch() {
  unsigned long interrupt_time = millis();




  // Debounce: Ignore interrupt if triggered within the last 200ms
  if (interrupt_time - last_interrupt_time > 200) {
    // Toggle the system state
    system_status = !system_status;




    // Motor control based on the new state
    if (system_status == LOW) {  // kill the motors
      motor1.writeMicroseconds(minPulseWidth);
      motor2.writeMicroseconds(minPulseWidth);
      motor_signal1 = 1000;
      motor_signal2 = 1000;
      //setpoint = 0;
      integral = 0;
      derivative = 0;
      previousError = 0;
      Pout = 0;
      Iout = 0;
      Dout = 0;
      output = 1000;
      baseline_motor_signal = 1000;
      previous_output = 1000;
    }
    if (system_status == HIGH) {
    }
    // Update the last interrupt time
    last_interrupt_time = interrupt_time;
  }
}
float better_contrain(float value, float range_low, float range_high) {
  float value_offset = 0;
  if (value <= range_low) {
    value_offset = value;
    value = range_low;
    return value_offset;
  }
  if (value >= range_high) {
    value_offset = value - range_high;
    value = range_high;
    return value_offset;

  } else {
    return 0;
  }
}

void moving_baseline_pid(float airspeed, float setpoint) {
  // Calculate error
  error = setpoint - airspeed;
  if (abs(error) < 0.1) {
    baseline_adjust_rate = 0.01;
  } else {
    baseline_adjust_rate = 0.05;
  }
  //Proportional term
  Pout = Kp * error;


  // Integral term
  integral += (error * (interval / 1000));  // Assuming a loop time of 50ms
  Iout = Ki * integral;


  // Derivative term
  derivative = (error - previousError) / (interval / 1000);  // Assuming a loop time of 50ms
  Dout = Kd * derivative;


  // Calculate total output using the last baseline as the starting point
  // output = baseline_motor_signal + Pout + Iout + Dout;


  pid_output = baseline_motor_signal + Pout + Iout + Dout;


  // Calculate the difference between the desired PID output and the previous output
  delta = pid_output - previous_output;
  if (abs(delta) > maxDelta) {
    delta = (delta > 0) ? maxDelta : -maxDelta;  // Limit the delta to maxDelta
  }


  // Update the output with the limited change
  output = previous_output + delta;
  // If the error is within the deadband, save the output as the new baseline
  if (abs(error) < deadband) {
    if (!is_in_deadband) {
      // Entering deadband, save the current output as the stable baseline
      baseline_motor_signal = output;
      is_in_deadband = true;
    }
    // Output remains stable at the baseline
    output = baseline_motor_signal;
  } else {
    // Outside the deadband, continue normal PID adjustments
    is_in_deadband = false;


    // Gradually adjust the baseline towards the current output
    baseline_motor_signal += baseline_adjust_rate * (output - baseline_motor_signal);
  }


  //output = round(output);
  // baseline_motor_signal = round(baseline_motor_signal);
  // Constrain the output and command motors
  output_offset = better_contrain(output, minPulseWidth, 1900);  // Keep the output within safe limits
  output -= output_offset;


  // Update previous error and previous output
  previousError = error;
  previous_output = output;


  // Serial.print(setpoint);
  // Serial.print(",");


  // Serial.println(airspeed);


  // Debugging info
  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print("airspeed: ");
  Serial.print(airspeed);
  Serial.print("\t");
  Serial.print("error: ");
  Serial.print(error);
  Serial.print("\t");
  Serial.print("Pout: ");
  Serial.print(Pout);
  Serial.print("\t");
  Serial.print("Iout: ");
  Serial.print(Iout);
  Serial.print("\t");
  Serial.print("Dout: ");
  Serial.print(Dout);
  Serial.print("\t");
  Serial.print("Output: ");
  Serial.print(output);
  Serial.print("\t");
  Serial.print("Baseline: ");
  Serial.print(baseline_motor_signal);
  Serial.print("\n");


  // Command motors with the final output
  command_motors(output);
}

void airspeed_pid(float airspeed, float setpoint) {
  // Calculate error
  error = setpoint - airspeed;
  if (abs(error) < 1) {
    // Proportional term
    Pout = Kp * error;


    // Integral term
    integral += (error * (interval / 1000));  // Assuming a loop time of 50ms
    Iout = Ki * integral;




    // Derivative term
    derivative = (error - previousError) / (interval / 1000);  // Assuming a loop time of 50ms
    Dout = Kd * derivative;


    //aprox_motor_signal = 1000 + setpoint * airspeed_to_motor_signal;




    // Calculate total output
    output = aprox_motor_signal + Pout + Iout + Dout;


    if (abs(error) < deadband) {
      if (!is_in_deadband) {
        // Entering deadband, save the current output as the stable baseline
        aprox_motor_signal = output;
        is_in_deadband = true;
      }
      // Output remains stable at the baseline
      output = aprox_motor_signal;
    } else {
      // Outside the deadband, continue normal PID adjustments
      is_in_deadband = false;


      // Gradually adjust the baseline towards the current output
      //baseline_motor_signal += baseline_adjust_rate * (output - baseline_motor_signal);
    }


    output_offset = better_contrain(output, minPulseWidth, 1300);
    output -= output_offset;
    // Update previous error
    previousError = error;
    output = round(output);
    Serial.print("setpoint: ");
    Serial.print(setpoint);
    Serial.print("\t");
    Serial.print("airspeed: ");
    Serial.print(airspeed);
    Serial.print("\t");


    Serial.print("error: ");
    Serial.print(error);
    Serial.print("\t");
    Serial.print("Pout: ");
    Serial.print(Pout);
    Serial.print("\t");
    Serial.print("Iout: ");
    Serial.print(Iout);
    Serial.print("\t");
    Serial.print("Dout: ");
    Serial.print(Dout);
    Serial.print("\t");
    Serial.print("Output: ");
    Serial.print(output);
    Serial.print("\n");
  } else {
    output = aprox_motor_signal;
  }




  //command motors:
  command_motors(output);
}


void command_motors(float output) {
  motor_signal1 = output;
  motor_signal2 = output;


  motor1.writeMicroseconds(motor_signal1);
  motor2.writeMicroseconds(motor_signal2);
}


void setPIDValues() {


  /*
  Serial.print("Enter new aprox_motor_signal value: ");
  while (!Serial.available())
    ;                                        // Wait for user input
  aprox_motor_signal = Serial.parseFloat();  // Read float value from the serial monitor
  Serial.println(aprox_motor_signal);
  clearSerialBuffer();  // Clear any leftover characters
  delay(200);
*/
  Serial.print("Enter new P value: ");
  while (!Serial.available())
    ;                        // Wait for user input
  Kp = Serial.parseFloat();  // Read float value from the serial monitor
  Serial.println(Kp);
  clearSerialBuffer(Serial);  // Clear any leftover characters
  delay(200);




  Serial.print("Enter new I value: ");
  while (!Serial.available())
    ;
  Ki = Serial.parseFloat();
  Serial.println(Ki);
  clearSerialBuffer(Serial);
  delay(200);




  Serial.print("Enter new D value: ");
  while (!Serial.available())
    ;
  Kd = Serial.parseFloat();
  Serial.println(Kd);
  clearSerialBuffer(Serial);
  delay(200);




  Serial.println("PID values updated.");
}


void dumpPIDValues() {
  Serial.println("Current Configuration:");
  Serial.print("setpoint = ");
  Serial.println(setpoint);
  Serial.print("aprox_motor_signal = ");
  Serial.println(aprox_motor_signal);
  Serial.print("P = ");
  Serial.println(Kp);
  Serial.print("I = ");
  Serial.println(Ki);
  Serial.print("D = ");
  Serial.println(Kd);
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

        correction_value = data.calibrationValue_Airspeed;
        pid_loop_hz = data.pid_loop_hz;

        maxDelta = data.delta_max;
        deadband = data.deadband;

        Kp = data.p_gain;
        Ki = data.i_gain;
        Kd = data.d_gain;

        windspeedCount = data.windspeedCount;
        memcpy(windspeedList, data.windspeedList, sizeof(windspeedList));
        Serial.println("cal_airspeed: ");
        Serial.println(correction_value);
        Serial.println("pid_loop-hz: ");
        Serial.println(pid_loop_hz);
        Serial.println("maxDelta: ");
        Serial.println(maxDelta);
        Serial.println("deadband: ");
        Serial.println(deadband);
        Serial.println("Kp: ");
        Serial.println(Kp);
        Serial.println("Ki: ");
        Serial.println(Ki);
        Serial.println("Kd: ");
        Serial.println(Kd);
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
template<typename T>
void handleGetCommand(HardwareSerial &serial, T &datatosend) {
  if (serial.available() >= 3) {
    char command[4];
    serial.readBytes(command, 3);
    command[3] = '\0';

    if (strcmp(command, "GET") == 0) {
      Serial.println("GET command received");
      avrg_count = 0;
      avrg_airspeed = 0;
      sendDataWithRetry(main_controller, datatosend, 50, 25);
    }  else if (strcmp(command, "TGE") == 0) {
      Serial.println("TGE command received");
      avrg_count = 0;
      avrg_airspeed = 0;
      sendDataWithRetry(main_controller, datatosend, 50, 25);
    } else if (strcmp(command, "ETG") == 0) {
      Serial.println("ETG command received");
      avrg_count = 0;
      avrg_airspeed = 0;
      sendDataWithRetry(main_controller, datatosend, 50, 25);
    }else if (strcmp(command, "s:0") == 0) {
      Serial.println("s:0 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 0;
    } else if (strcmp(command, "s:1") == 0) {
      Serial.println("s:1 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 1;
    } else if (strcmp(command, "s:2") == 0) {
      Serial.println("s:2 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 2;
    } else if (strcmp(command, "s:3") == 0) {
      Serial.println("s:3 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 3;
    } else if (strcmp(command, "s:4") == 0) {
      Serial.println("s:4 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 4;
    } else if (strcmp(command, "s:5") == 0) {
      Serial.println("s:5 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 5;
    } else if (strcmp(command, "s:6") == 0) {
      Serial.println("s:6 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 6;
    } else if (strcmp(command, "s:7") == 0) {
      Serial.println("s:7 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 7;
    } else if (strcmp(command, "s:8") == 0) {
      Serial.println("s:8 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 8;
    } else if (strcmp(command, "s:9") == 0) {
      Serial.println("s:9 command received");
      sendAcknowledgment(main_controller, "ACK");
      set = 9;
    } 
     else {
      Serial.print("other data recieved: ");
      Serial.println(command);
      clearSerialBuffer(serial);
    }
  }
}

void handleCommand(HardwareSerial &serial) {
  if (serial.available() >= 5) {
    char command[6];
    serial.readBytes(command, 5);
    command[5] = '\0';

    if (strcmp(command, "armed") == 0) {
      Serial.println("armed command received");
      main_controller_status = HIGH;
      sendAcknowledgment(main_controller, "ACK");
    }  else {
      Serial.print("other data recieved: ");
      Serial.println(command);
      clearSerialBuffer(serial);
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

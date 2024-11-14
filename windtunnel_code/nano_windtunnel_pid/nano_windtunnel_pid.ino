#include <Wire.h>
#include <Servo.h>

// Constants
const int sensorAddress = 0x28;        // Address of MS4525 airspeed sensor
const float rho = 1.225;               // Air density in kg/m^3 (replace with actual value for your conditions)
float zeroAirspeedPressure = 7656.35;  // Raw pressure at zero airspeed
float slope = 1.0;
const int bufferSize = 50;         // Number of readings for moving average
float pressureBuffer[bufferSize];  // Buffer to store pressure readings
int bufferIndex = 0;               // Index for the buffer

// PID constants
/* example pid constats:
float Kp = 1.0;
float Ki = 0.05;
float Kd = 0.01;
*/
//test constatnts:
float Kp = 15;
float Ki = 0.5;  //0.30;
float Kd = 0.1;  //0.05;

float previousError = 0;
float integral = 0;

float error;
float Pout;
float Iout;
float Dout;

float offset;
float output_offset;
float output;

float derivative;

// Servo setup
Servo motor;
const int motorPin = A0;

bool loop_continue = true;
int kill_switch_pin = D2;
// Function prototypes
/*
float calc_airspeed();
int16_t readRawPressure();
void calibrate();
int pid(float airspeed, float setpoint);
*/
unsigned long previousMillis = 0;  // Stores the last time the loop ran
float interval;
int next = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  motor.attach(motorPin, 1000, 2000);
  //esc_calibration();
  motor.writeMicroseconds(1000);  // Initialize ESC to 0 throttle (usually 0 degrees)
  while (!Serial) { delay(10); }
  Serial.println("wait for arming");
  delay(1000);
  // Initialize the pressure buffer
  for (int i = 0; i < bufferSize; i++) {
    pressureBuffer[i] = 0.0;
  }
  loop_timer(20);  // takes hz value and turns it into a interval time
  // Calibrate the sensor
  calibrate();
  Serial.println("starting in:");
  dramatic_countdown();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float airspeed = calc_airspeed();
    if (airspeed <= 2) {
      next = 1;
    }
    if (next == 1 && loop_continue = true) {
      float setpoint = 5;  // setpoint in m/s

      float motorPosition = pid(airspeed, setpoint);
      motor.writeMicroseconds(motorPosition);

      // Print airspeed and motor position for debugging
      //Serial.print("Airspeed: ");
      //Serial.print(abs(setpoint - airspeed));
      //Serial.println(",");
      //Serial.print(" m/s, ESC Throttle: ");
      //Serial.println(motorPosition);
      //Serial.println(",");
    }
  }
}

int loop_timer(int hz) {
  interval = 1000 / hz;
  return interval;
}

float calc_airspeed() {
  int16_t rawPressure = readRawPressure();

  // Update the buffer with the new reading
  pressureBuffer[bufferIndex] = rawPressure;
  bufferIndex = (bufferIndex + 1) % bufferSize;

  // Calculate the moving average
  float sumPressure = 0.0;
  for (int i = 0; i < bufferSize; i++) {
    sumPressure += pressureBuffer[i];
  }
  float averagePressure = sumPressure / bufferSize;

  // Calculate corrected differential pressure and make it absolute (no negative values)
  float correctedPressure = abs(slope * (averagePressure - zeroAirspeedPressure));

  // Convert pressure to airspeed using Bernoulli's equation
  float airspeed = sqrt(2 * correctedPressure / rho);
  return airspeed;
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
  }
}
void esc_calibration() {
  motor.writeMicroseconds(2000);  // set to 2000us puls length at boot up, tells max position to the esc
  while (!Serial) { delay(10); }
  Serial.println("max positon");
  delay(6000);

  motor.writeMicroseconds(1000);  //set to 1000us puls length, tells low position to esc
  Serial.println("low position");
  delay(3000000);
}

float pid(float airspeed, float setpoint) {
  // Calculate error
  error = setpoint - airspeed;
  if (error > 0.5) {
    offset += 0.4;
  }
  if (error < -0.5) {
    offset -= 0.4;
  }
  // Proportional term
  Pout = Kp * error;

  // Integral term
  integral += error * (interval / 1000);  // Assuming a loop time of 50ms
  Iout = Ki * integral;

  // Derivative term
  derivative = (error - previousError) / (interval / 1000);  // Assuming a loop time of 50ms
  Dout = Kd * derivative;

  // Calculate total output
  output = 1000 + Pout + Iout + Dout + offset;
  output_offset = better_contrain(output, 1000, 2000);
  output -= output_offset;
  // Update previous error
  previousError = error;

  Serial.print(offset);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(Pout);
  Serial.print("\t");
  Serial.print(Iout);
  Serial.print("\t");
  Serial.print(Dout);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\t");
  Serial.print(airspeed);
  Serial.print("\n");

  // Constrain output to servo limits (0 to 180)
  //output = constrain(output, 0, 180);
  return output;  // return output as int
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
void dramatic_countdown() {
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  Serial.println("here we go: initializing loop");
}

void kill_switch(){
  int kill_switch = digitalRead(kill_switch_pin);
  if(kill_switch = 0){
    motor.writeMicroseconds(1000);
    delay(2000);
    loop_continue = false;
  }
}

#include <Wire.h>
#include <ESP32Servo.h>

const int killswitch_pin_pressed = 9;  // killswitch pin
volatile bool system_status = LOW;     // System initially off
volatile unsigned long last_interrupt_time = 0;

int interval;  // loop time
volatile unsigned long last_time = 0;

// || constants for the motors ||
Servo motor1;
Servo motor2;

int motor_signal1;
int motor_signal2;

const int motorPin1 = 5;
const int motorPin2 = 6;


const int minPulseWidth = 1000;  // Minimum pulse width in microseconds (1 ms)
const int maxPulseWidth = 2000;  // Maximum pulse width in microseconds (2 ms)

float aprox_motor_signal;
const float airspeed_to_motor_signal = ;
// || every constant for the windspeed meter ||
const int sensorAddress = 0x28;  // Address of MS4525 airspeed sensor
const float rho = 1.225;         // Air density in kg/m^3 (replace with actual value for your conditions)
float zeroAirspeedPressure = 0;  // Raw pressure at zero airspeed
float slope = 1.0;
const int bufferSize = 50;         // Number of readings for moving average
float pressureBuffer[bufferSize];  // Buffer to store pressure readings
int bufferIndex = 0;               // Index for the buffer

// || pid value's ||

// PID constants
/* example pid constats:
float Kp = 1.0;
float Ki = 0.05;
float Kd = 0.01;
*/
//test constatnts:
float Kp = 2.5;
float Ki = 0.06;
float Kd = 0.01;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  ESP32PWM::allocateTimer(0);  // set timers for the pwm signals
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor1.setPeriodHertz(50);  // standard 50 hz esc signal
  motor2.setPeriodHertz(50);  // standard 50 hz esc signal
  motor1.attach(motorPin1, 1000, 2000);
  motor2.attach(motorPin2, 1000, 2000);

  pinMode(killswitch_pin_pressed, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(killswitch_pin_pressed), killswitch, FALLING);

  hz_inteval(10);  //convertes hz to looptime
  // Calibrate the sensor
  calibrate();
}

void loop() {
  // Read raw pressure values from sensor
  float airspeed = calc_airspeed();

  // Print airspeed
  Serial.print(airspeed);
  Serial.println(",");

  delay(50);  // Adjust delay as needed for your application
}

void hz_inteval(int loop_freqenty) {
  interval = 1000 / loop_freqenty;
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
void airspeed_pid(float airspeed, float setpoint, float aprox_motor_signal) {
  // Calculate error
  error = setpoint - airspeed;
  // Proportional term
  Pout = Kp * error;

  // Integral term
  integral += error * (interval / 1000);  // Assuming a loop time of 50ms
  Iout = Ki * integral;

  // Derivative term
  derivative = (error - previousError) / (interval / 1000);  // Assuming a loop time of 50ms
  Dout = Kd * derivative;

  // Calculate total output
  output = aprox_motor_signal + Pout + Iout + Dout;
  output_offset = better_contrain(output, minPulseWidth, maxPulseWidth);
  output -= output_offset;
  // Update previous error
  previousError = error;
  /*
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
  Serial.print("\n");
  */
  
  motor_signal1 = round(output);
  motor_signal2 = round(output);
  //command motors:
  motor1.writeMicroseconds(motor_signal1);
  motor2.writeMicroseconds(motor_signal2);
}

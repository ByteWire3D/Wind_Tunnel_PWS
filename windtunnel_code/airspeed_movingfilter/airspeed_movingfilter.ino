#include <Wire.h>

const int sensorAddress = 0x28;        // Address of MS4525 airspeed sensor
const float rho = 1.225;               // Air density in kg/m^3 (replace with actual value for your conditions)
float zeroAirspeedPressure = 0;  // Raw pressure at zero airspeed
float slope = 1.0;
const int bufferSize = 50;  // Number of readings for moving average
float pressureBuffer[bufferSize];  // Buffer to store pressure readings
int bufferIndex = 0;  // Index for the buffer

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the pressure buffer
  for (int i = 0; i < bufferSize; i++) {
    pressureBuffer[i] = 0.0;
  }

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
  while (Wire.available() < 2);        // Wait for 2 bytes to be available
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

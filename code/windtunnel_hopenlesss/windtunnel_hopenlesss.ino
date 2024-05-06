#include <Wire.h>
#include <PWMServo.h>

#define MS4525_ADDRESS 0x28  // I2C address of MS4525 sensor

PWMServo motor1;

unsigned long previousMillis = 0;  // Variable to store the last time the loop was executed
int state = 0;
int looptime;
bool dataReady;
int value = 5;
int count;
float pressurePa;
float temperaturee;
void setup() {
  Serial.begin(15200);  // Initialize serial communication
  motor1.attach(2, 900, 2100);
  //esc_calibration();  // only one time
  motor1.write(0);

  Wire.begin();       // Initialize I2C communication
  loop_freqenty(20);  // sets the loop freqenty to 20hz

  Serial.println("setup compleet!");
  delay(5000);
}

void loop() {
  switch (state) {
    case 0:
      send_command();
      state = 1;
      break;
    case 1:
      unsigned long currentMillis = millis();

      // Execute the loop at a frequency of 200 Hz
      if (currentMillis - previousMillis >= looptime) {  // 1000 ms / 200 Hz = 5 ms
        previousMillis = currentMillis;                  // Update previousMillis
        readWindSpeed();
        //requestData();
        //calculateData();
        //command_motor();
        state = 0;
      }
      break;  // Read wind speed and temperature
  }

  // No need for delay() function as we're using millis() to control loop frequency
}
void send_command() {
  Wire.beginTransmission(MS4525_ADDRESS);
  Wire.write(0x00);  // Send command to start measurement
  Wire.endTransmission();
}
void loop_freqenty(int freqenty) {
  looptime = 1000 / freqenty;
}
void command_motor() {
  if (value <= 90 and count >= 20) {
    value++;
    count = 0;
    motor1.write(value);
  }
  Serial.print(value);
  Serial.print("\n");
  count++;
}

// Function to read wind speed from MS4525 sensor
void readWindSpeed() {
  Wire.requestFrom(MS4525_ADDRESS, 4);  // Request 4 bytes of data
  if (Wire.available() == 4) {
    byte msb_pressure = Wire.read();
    byte lsb_pressure = Wire.read();
    byte msb_temperature = Wire.read();
    byte lsb_temperature = Wire.read();

    int rawPressure = (msb_pressure << 8) | lsb_pressure;           // Combine bytes to form raw pressure value
    int rawTemperature = (msb_temperature << 8) | lsb_temperature;  // Combine bytes to form raw temperature value

    // Conversion factors from datasheet
    float pressure = (rawPressure - 1638.0) * 0.000045;             // Convert raw pressure value to kPa
    float temperature = (rawTemperature - 1638.0) * 0.003222 - 50;  // Convert raw temperature value to degrees Celsius

    // Calculate wind speed using pressure difference (differential pressure)
    float windSpeed = sqrt(2 * pressure / 1.225);  // Assuming air density of 1.225 kg/m^3

    // Print wind speed and temperature
    Serial.print("presure ");
    Serial.print(pressure);
    Serial.print("\t");
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.print(" m/s");
    Serial.print("\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" C");
    Serial.print("\t");
     Serial.print("\n");
  } else {
    Serial.println("Error reading data from MS4525 sensor.");
  }
}
void esc_calibration() {
  motor1.write(180);  // set to 2000us puls length
  while (!Serial) { delay(10); }
  Serial.println("max positon");
  delay(1000);

  motor1.write(0);  //set to 1000us puls length
  Serial.println("low position");
  delay(3000);
}

void requestData() {
  Wire.requestFrom(MS4525_ADDRESS, 4);  // Request 4 bytes of data
  if (Wire.available() == 4) {
    byte msbPressure = Wire.read();
    byte lsbPressure = Wire.read();
    byte msbTemperature = Wire.read();
    byte lsbTemperature = Wire.read();
    pressurePa = ((msbPressure << 8) | lsbPressure) & 0x3FFF;            // Masking the MSB to remove the status bits
    temperaturee = ((msbTemperature << 8) | lsbTemperature) * 0.003222;  // Convert to Celsius
    dataReady = true;
  }
}

void calculateData() {
  if (dataReady) {
    // Simulated calculation of airspeed
    float airspeed = sqrt(2 * pressurePa / 1.225);  // Placeholder calculation assuming air density of 1.225 kg/m^3
    Serial.print("Wind Speed: ");
    Serial.print(airspeed);
    Serial.print(" m/s");
    Serial.print("\t");
    Serial.print("Temperature: ");
    Serial.print(temperaturee);
    Serial.print(" C");
    Serial.print("\t");
  }
}

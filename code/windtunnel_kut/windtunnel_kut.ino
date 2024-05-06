#include <Wire.h>
#include <PWMServo.h>

PWMServo motor1;
int value = 5;
int count;

#define MS4525D0_I2C_ADDR1 0x28
#define MS4525D0_I2C_ADDR2 0x36
#define MS4525D0_I2C_ADDR3 0x46

class AP_Airspeed_MS4525 {
public:
  AP_Airspeed_MS4525() {}

  bool init();
  void update();
  float getAirSpeed() {
    return _air_speed;
  }
  float getPressure() {
    return _pressure;
  }
  float getTemperature() {
    return _temperature;
  }

private:
  uint32_t _measurement_started_ms;
  float _air_speed;
  float _pressure;
  float _temperature;
  uint8_t _address;

  void _measure();
  void _collect();
};

bool AP_Airspeed_MS4525::init() {
  Wire.begin();

  const uint8_t addresses[] = { MS4525D0_I2C_ADDR1, MS4525D0_I2C_ADDR2, MS4525D0_I2C_ADDR3 };
  for (uint8_t addr : addresses) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      _address = addr;
      return true;
    }
  }

  Serial.println("No MS4525 sensor found.");
  return false;
}

void AP_Airspeed_MS4525::_measure() {
  Wire.beginTransmission(_address);
  Wire.write(0);
  Wire.endTransmission();
  _measurement_started_ms = millis();
}

void AP_Airspeed_MS4525::_collect() {
  uint8_t data[4];
  Wire.requestFrom(_address, (uint8_t)4);
  if (Wire.available() >= 4) {
    for (uint8_t i = 0; i < 4; i++) {
      data[i] = Wire.read();
    }
    // Process data and calculate airspeed
    int16_t dp_raw = ((data[0] << 8) + data[1]) & 0x3FFF;
    int16_t dT_raw = (((data[2] << 8) + data[3]) & 0xFFE0) >> 5;

    // Constants for conversion
    const float P_max = 2.5f;
    const float P_min = -2.5f;
    const float PSI_to_Pa = 6894.757f;
    const float slope = 65.0f;
    const float temp_slope = 0.887f;
    const float air_density = 1.225f;  // kg/m^3
    const float P0 = 101325.0f;        // Sea-level pressure in Pa

    // Convert raw data to differential pressure and temperature
    float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
    _pressure = diff_press_PSI * PSI_to_Pa;
    _temperature = ((200.0f * dT_raw) / 2047) - 50;

    //Apply voltage correction
    float voltage_diff = (analogRead(A0) * 5.0) / 1023.0;
    voltage_diff = constrain(voltage_diff, -0.7, 0.5);
    _pressure -= voltage_diff * slope;
    _temperature -= voltage_diff * temp_slope;

    // Calculate airspeed
    //_air_speed = sqrt(2 * (_pressure - P0) / air_density);
    _air_speed = sqrt(2 * _pressure / 1.225);
  }
}

void AP_Airspeed_MS4525::update() {
  _measure();
  delay(10);  // Wait for measurement
  _collect();
}

// Example usage
AP_Airspeed_MS4525 airspeedSensor;

void setup() {
  Serial.begin(9600);
  motor1.attach(2, 900, 2100);
  //esc_calibration();  // only one time
  motor1.write(0);
  if (airspeedSensor.init()) {
    Serial.println("MS4525 sensor initialized.");
  }
  Serial.println("setup compleet!");
  delay(5000);
}

void loop() {
  airspeedSensor.update();
  float airspeed = airspeedSensor.getAirSpeed();
  float pressure = airspeedSensor.getPressure();
  float temperature = airspeedSensor.getTemperature();
  command_motor();
  Serial.print("Airspeed: ");
  Serial.print(airspeed);
  Serial.print(" m/s, Pressure: ");
  Serial.print(pressure);
  Serial.print(" Pa, Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  delay(100);
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

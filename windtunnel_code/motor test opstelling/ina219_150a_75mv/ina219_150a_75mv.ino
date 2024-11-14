#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setCalibration_80mV_150A() {
  // Calibration value calculated for 80mV range and 0.0005 ohm shunt
  uint16_t calibrationValue = 17893;

  // Configuration register for 16V bus voltage range and ±80mV shunt voltage range
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_2_80MV |        // Gain = /2 (±80mV range)
                    INA219_CONFIG_BADCRES_12BIT |      // Bus ADC resolution = 12-bit
                    INA219_CONFIG_SADCRES_12BIT_1S_532US | // Shunt ADC resolution = 12-bit, 1 sample
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; // Continuous mode

  // Write calibration value to INA219
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(INA219_REG_CALIBRATION); // Calibration register
  Wire.write(calibrationValue >> 8);  // MSB
  Wire.write(calibrationValue & 0xFF); // LSB
  Wire.endTransmission();

  // Write configuration value to INA219
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(INA219_REG_CONFIG);  // Configuration register
  Wire.write(config >> 8);        // MSB
  Wire.write(config & 0xFF);      // LSB
  Wire.endTransmission();
}

void setup(void) {
  Serial.begin(115200);

  // Initialize the INA219 (this sets up the I2C communication)
  ina219.begin();

  // Set the custom calibration for 80mV, 150A
  setCalibration_80mV_150A();
  //ina219.setCalibration_32V_2A();
}

void loop(void) {
  float shuntvoltage = ina219.getShuntVoltage_mV();  // Measure shunt voltage in millivolts
  float busvoltage = ina219.getBusVoltage_V();        // Measure bus voltage
  float current_A = ina219.getCurrent_mA() / 1000.0;  // Calculate current in amperes

  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Current:       "); Serial.print(current_A); Serial.println(" A");
  Serial.println("");

  delay(2000);
}

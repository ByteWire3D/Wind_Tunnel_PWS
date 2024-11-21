

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <HX711_ADC.h>

#define TX_PIN 21  // TX pin
#define RX_PIN 20  // RX pin (optional, if the board supports it)



Adafruit_INA219 ina219;

HardwareSerial main_controller(0);  //Create a new HardwareSerial class.

volatile unsigned long last_time = 0;


const int lift_loadcell_dout = 2;  //mcu > HX711 dout pin
const int lift_loadcell_sck = 3;   //mcu > HX711 sck pin

const int dragg_loadcell_dout = 4;  //mcu > HX711 dout pin
const int dragg_loadcell_sck = 5;   //mcu > HX711 sck pin

HX711_ADC Lift(lift_loadcell_dout, lift_loadcell_sck);
HX711_ADC Dragg(dragg_loadcell_dout, dragg_loadcell_sck);

float calibrationValue_Lift = 696.0;   // calibration value load cell 1
float calibrationValue_Dragg = 733.0;  // calibration value load cell 2

static boolean newDataReady = 0;

float shuntvoltage;
float busvoltage;
float current_A;
float mah_consumed;

float lift;
float dragg;


void setup() {
  Serial.begin(115200);
  main_controller.begin(115200, SERIAL_8N1, RX, TX);
  delay(10);

  ina219.begin();

  // Set the custom calibration for 80mV, 150A
  setCalibration_80mV_150A();
  setup_loadcells();
  while (!Serial) { delay(10); }
  // while (!main_controller) { delay(10); }
  Serial.println("Startup is complete");
}

void loop() {
  // check for new data/start next conversion:
  if (Lift.update()) newDataReady = true;
  Dragg.update();

  if (main_controller.avialable()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove extra spaces and newlines
    if (input.equalsIgnoreCase("get")) {

      shuntvoltage = ina219.getShuntVoltage_mV();   // Measure shunt voltage in millivolts
      busvoltage = ina219.getBusVoltage_V();        // Measure bus voltage
      current_A = ina219.getCurrent_mA() / 1000.0;  // Calculate current in amperes

      if ((newDataReady)) {
        {
          lift = Lift.getData();
          dragg = Dragg.getData();
          Serial.print("Load_cell lift output val: ");
          Serial.print(lift);
          Serial.print("    Load_cell dragg output val: ");
          Serial.println(dragg);
          newDataReady = 0;
        }
      }
    }
  }
  if (Serial.available()) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove extra spaces and newlines

    // Handle "calibrate lift" command
    if (input.equalsIgnoreCase("calibrate_lift")) {
      calibrate_lift();
    }
    // Handle "calibrate dragg" command
    else if (input.equalsIgnoreCase("calibrate_dragg")) {
      calibrate_dragg();
    }
    else if(input.equalsIgnoreCase("tare"))[
      Lift.tareNoDelay();
      Dragg.tareNoDelay();
    ]
  }
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      Lift.tareNoDelay();
      Dragg.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (Lift.getTareStatus() == true) {
    Serial.println("Tare load cell lift complete");
  }
  if (Dragg.getTareStatus() == true) {
    Serial.println("Tare load cell dragg complete");
  }
}

void setup_loadcells() {
  Lift.begin();
  Dragg.begin();
  //Lift.setReverseOutput();
  //Dragg.setReverseOutput();
  unsigned long stabilizingtime = 2000;  // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  byte Lift_1_rdy = 0;
  byte Dragg_2_rdy = 0;
  while ((Lift_1_rdy + Dragg_2_rdy) < 2) {  //run startup, stabilization and tare, both modules simultaniously
    if (!Lift_1_rdy) Lift_1_rdy = Lift.startMultiple(stabilizingtime, _tare);
    if (!Dragg_2_rdy) Dragg_2_rdy = Dragg.startMultiple(stabilizingtime, _tare);
  }
  if (Lift.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (Dragg.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  Lift.setCalFactor(calibrationValue_Lift);    // user set calibration value (float)
  Dragg.setCalFactor(calibrationValue_Dragg);  // user set calibration value (float)
}

void setCalibration_80mV_150A() {
  // Calibration value calculated for 80mV range and 0.0005 ohm shunt
  uint16_t calibrationValue = 17893;

  // Configuration register for 16V bus voltage range and ±80mV shunt voltage range
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V | INA219_CONFIG_GAIN_2_80MV |  // Gain = /2 (±80mV range)
                    INA219_CONFIG_BADCRES_12BIT |                                  // Bus ADC resolution = 12-bit
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |                         // Shunt ADC resolution = 12-bit, 1 sample
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;                       // Continuous mode

  // Write calibration value to INA219
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(INA219_REG_CALIBRATION);   // Calibration register
  Wire.write(calibrationValue >> 8);    // MSB
  Wire.write(calibrationValue & 0xFF);  // LSB
  Wire.endTransmission();

  // Write configuration value to INA219
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(INA219_REG_CONFIG);  // Configuration register
  Wire.write(config >> 8);        // MSB
  Wire.write(config & 0xFF);      // LSB
  Wire.endTransmission();
}

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();  // Read and discard all characters in the buffer
  }
}
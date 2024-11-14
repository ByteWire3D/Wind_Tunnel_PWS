//motor test.
/*
 10hz
 10steps
 10sec per step

 so 100 loop interations per step
 and 1000 loops intotal
 total mesurements:
 - 1000 messuremenst of volt (V)
 - 1000 messuremenst of trusth (grams)
 - 1000 messuremenst of amp (A)
 - 1000 trottle values (ms pulse width values)
- 1000 loop number (as a reffereunce of the time)
 so 5000 values minimum

// how to program will fuction

when the voltage that is mesurred by the voltage meter is > 6v the program will go in record  mode:

  every loop (100ms), the following things will be mesurred: ampere, voltage, trusth and the motor signal will be recoreded.
  
  when the ten_sec_counter reaches 100, the step counter will go up by one and the motor pulse width signal will go up by 100 (10 steps), so from 1000 to 1100 etc.
  
  when the step counter reaches 10 the motor signal whill go back to 1000 to stop the motor.

  All of these mesurements will be stored in the memory of the esp32 xiao c3, using the preffrences to store the data on the onboard memory. 

when the voltage meter, mesurres a voltage lower than 6v it will wait for a serial connection with the pc and then dump all the record data one by one in the serial motiotor so it can easely be copied into a exel.

///////////////////// SAVETY \\\\\\\\\\\\\\\\\\\\\\\\\\\\

for this testing program there is a switch that when pressed will toggle the system_status of the program from running to stopped, 
the switching is handeld by a interupt function, so shutting the program of whill happen instantly
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <HX711_ADC.h>
#include <ESP32Servo.h>
#include <BleCombo.h>

Adafruit_INA219 ina219;

Servo motor;

int loop_time;
volatile unsigned long last_time = 0;

const int killswitch_pin_pressed = 9;

const int HX711_dout = 4;  //mcu > HX711 dout pin
const int HX711_sck = 5;   //mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int motorPin = 20;  // Pin connected to the servo

const int minPulseWidth = 1000;     // Minimum pulse width in microseconds (1 ms)
const int maxPulseWidth = 2000;     // Maximum pulse width in microseconds (2 ms)

volatile bool system_status = LOW;  // System initially off
bool stop_for_copying = LOW;
bool dump_memory = false;
static boolean newDataReady = 0;

volatile unsigned long last_interrupt_time = 0;

int loop_numbers[1000];
int trottle_values[1000];
float busvoltages[1000];
float shuntvoltages[1000];
float amps[1000];
float trusth_values[1000];

int loop_counter = 0;
int motor_signal = 1100;
float shuntvoltage;
float busvoltage;
float current_A;
float motor_trusth_grams;

int step_counter = 0;
int ten_sec_counter = 0;

void setup() {
  Serial.begin(115200);

  ina219.begin();
  Keyboard.begin();

  // Set the custom calibration for 80mV, 150A
  setCalibration_80mV_150A();
  pinMode(killswitch_pin_pressed, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(killswitch_pin_pressed), killswitch, FALLING);

  init_loadcell();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor.setPeriodHertz(50);  // standard 50 hz servo
  motor.attach(motorPin, 1000, 2000);

  hz_looptime(10);  //convertes hz to looptime

  Serial.println("set pulse with to 1000 (0%)");
  motor.writeMicroseconds(1000);
}

void loop() {
  if (system_status == LOW && stop_for_copying == LOW) {
    Serial.println("System is not running");
    motor.writeMicroseconds(minPulseWidth);  //shut off motor
    delay(1000);
  }
  if (dump_memory) {
    dump_memory_BLE();
  }
  if (system_status == HIGH && stop_for_copying == LOW) {
    // check for new data/start next conversion:
    if (LoadCell.update()) { newDataReady = true; }
    unsigned long time = millis();
    if (time - last_time > loop_time) {
      //put all the stuff that needs to happen 10 times per second
      /*
      Serial.print(ten_sec_counter);
      Serial.print("\t");
      Serial.print(step_counter);
      Serial.print("\t");
      Serial.print(motor_signal);
      Serial.print("\t");
      Serial.print(system_status);
      Serial.print("\n");
*/
      shuntvoltage = ina219.getShuntVoltage_mV();   // Measure shunt voltage in millivolts
      busvoltage = ina219.getBusVoltage_V();        // Measure bus voltage
      current_A = ina219.getCurrent_mA() / 1000.0;  // Calculate current in amperes

      Serial.print(loop_counter);
      Serial.print("\t");
      Serial.print(step_counter);
      Serial.print("\t");
      Serial.print("signal: ");
      Serial.print(motor_signal);
      Serial.print(" ms \t");
      Serial.print("Bus: ");
      Serial.print(busvoltage);
      Serial.print(" V \t");
      Serial.print("Shunt: ");
      Serial.print(shuntvoltage);
      Serial.print(" mV \t");
      Serial.print("Cur:");
      Serial.print(current_A);
      Serial.print(" A \t");


      /*
      Serial.print(loop_counter);
      Serial.print(",");
      Serial.print(step_counter);
      Serial.print(",");
      Serial.print(busvoltage);
      Serial.print(",");
      Serial.print(shuntvoltage);
      Serial.print(",");
      Serial.print(current_A);
      Serial.print(",");
      Serial.print(motor_signal);
      Serial.print(",");
*/

      if (newDataReady) {
        motor_trusth_grams = LoadCell.getData();
        Serial.println(motor_trusth_grams);
        newDataReady = 0;
      }
      else{
        motor_trusth_grams = 0; // even when no data is there add it to prevent form ble glithes
        Serial.println();
      }
      motor.writeMicroseconds(motor_signal);

      save_to_memory(loop_counter, motor_signal, busvoltage, shuntvoltage, current_A, motor_trusth_grams);

      ten_sec_counter++;
      loop_counter++;
      if (ten_sec_counter >= 100) {
        ten_sec_counter = 0;
        motor_signal += 100;
        step_counter++;
      }
      if (step_counter >= 10) {
        system_status = LOW;
        motor.writeMicroseconds(minPulseWidth);
        Serial.println("test done! motor turned off!");
        delay(2500);
        // dump_memory_Serial();
        // dump_memory_BLE();
        dump_memory = true;
      }
      last_time = time;
    }
  }

  // put your main code here, to run repeatedly:
}

void hz_looptime(int loop_freqenty) {
  loop_time = 1000 / loop_freqenty;
}

void killswitch() {
  unsigned long interrupt_time = millis();

  // Debounce: Ignore interrupt if triggered within the last 200ms
  if (interrupt_time - last_interrupt_time > 200) {
    // Toggle the system state
    system_status = !system_status;

    // Motor control based on the new state
    if (system_status == LOW) {
      motor.writeMicroseconds(minPulseWidth);
      ten_sec_counter = 0;
      step_counter = 0;
      motor_signal = 1000;
      loop_counter = 0;
    }
    if (system_status == HIGH) {
      motor_signal = 1100;
    }
    // Update the last interrupt time
    last_interrupt_time = interrupt_time;
  }
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

void init_loadcell() {
  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue;     // calibration value (see example file "Calibration.ino")
  calibrationValue = 388.82;  // uncomment this if you want to set the calibration value in the sketch

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
    Serial.println("Startup is complete");
  }
}

void dump_memory_Serial() {
  stop_for_copying = HIGH;
  //dump all the memory one by one, so first print the loop numbers to the serialmoinitor then print the motor_signal, then the busvoltage, then the shunt voltage , then the ampere.
  // also make a flag to keep track if there even is some thing saved, if there is 1 or more test saved print them one by one. also print the names of wath you are printeing to the serial monitor
  // so print motor signal and then dump the 1000 values, then print busvolt, you get it i think.
  Serial.println("memory dump: ");
  Serial.println(".............................................................");

  delay(2000);

  Serial.println("the loop_number");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println(" ");
  Serial.println(" ");
  for (int i = 0; i < 1000; i++) {
    Serial.println(loop_numbers[i]);
  }
  Serial.println(" ");
  Serial.println(" ");

  Serial.println("the trottle value");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  for (int i = 0; i < 1000; i++) {
    Serial.println(trottle_values[i]);
  }
  Serial.println(" ");
  Serial.println(" ");

  Serial.println("the busvoltage");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println(" ");
  Serial.println(" ");
  for (int i = 0; i < 1000; i++) {
    Serial.println(busvoltages[i]);
  }
  Serial.println(" ");
  Serial.println(" ");

  Serial.println("the  shunt volt");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println(" ");
  Serial.println(" ");
  for (int i = 0; i < 1000; i++) {
    Serial.println(shuntvoltages[i]);
  }
  Serial.println(" ");
  Serial.println(" ");

  Serial.println("the ampere");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println(" ");
  Serial.println(" ");
  for (int i = 0; i < 1000; i++) {
    Serial.println(amps[i]);
  }
  Serial.println(" ");
  Serial.println(" ");

  Serial.println("trusth Values");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.println(" ");
  Serial.println(" ");
  for (int i = 0; i < 1000; i++) {
    Serial.println(trusth_values[i]);
  }
}

void dump_memory_BLE() {


  Serial.println("memory dump: ");
  delay(2000);
  if (Keyboard.isConnected()) {
    stop_for_copying = HIGH;
    delay(10);
    Keyboard.print("Loop Numbers");
    delay(500);
    Keyboard.write(KEY_TAB);
    delay(500);
    Keyboard.print("Trottle Values");
    delay(500);
    Keyboard.write(KEY_TAB);
    delay(500);

    Keyboard.print("Bus voltage");
    delay(500);
    Keyboard.write(KEY_TAB);
    delay(500);

    Keyboard.print("Shunt voltage");
    delay(500);
    Keyboard.write(KEY_TAB);
    delay(500);

    Keyboard.print("Amps");
    delay(500);
    Keyboard.write(KEY_TAB);

    Keyboard.print("Trust Values");
    delay(500);
    Keyboard.write(KEY_TAB);
    delay(500);

    Keyboard.write(KEY_RETURN);
    Keyboard.write(KEY_RETURN);

    delay(1000);

    for (int i = 0; i < 1000; i++) {
      Keyboard.print(loop_numbers[i]);
      Keyboard.write(KEY_TAB);

      Keyboard.print(trottle_values[i]);
      Keyboard.write(KEY_TAB);

      Keyboard.print(busvoltages[i]);
      Keyboard.write(KEY_TAB);

      Keyboard.print(shuntvoltages[i]);
      Keyboard.write(KEY_TAB);

      Keyboard.print(amps[i]);
      Keyboard.write(KEY_TAB);

      Keyboard.print(trusth_values[i]);
      Keyboard.write(KEY_TAB);

      Keyboard.write(KEY_RETURN);
      Keyboard.write(KEY_RETURN);

      delay(1000);
      if (i == 999) {
        dump_memory = false;
        Serial.println("memory dump done!!!");
      }
    }
  } else {

    Serial.println("waiting to connect via ble (turn it on?!)");
  }
  delay(1000);
}




void save_to_memory(int loop_number, int trottle_value, float busvolt, float shuntvolt, float ampere, float trusth_grams) {
  // save all the values to the "offline memory of the esp32 so even when powerd of the memory persists", save every value to there own data block,
  //make a value that trackes if memory is saved
  loop_numbers[loop_number] = loop_number;
  trottle_values[loop_number] = trottle_value;
  busvoltages[loop_number] = busvolt;
  shuntvoltages[loop_number] = shuntvolt;
  amps[loop_number] = ampere;
  trusth_values[loop_number] = trusth_grams;
}
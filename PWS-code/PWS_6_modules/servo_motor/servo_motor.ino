#include <ESP32Servo.h>


#define ENCA 3      // Define the pin YELLOW encoder wire
#define ENCB 4      //Define the pin WHITE encoder
#define motor1 5    // Define the pin for the first motor output signal
#define motor2 6    // Define the pin for the second motor output signal
#define PWM_PIN 20  // Define the pin for the PWM signal
#define ENDSTOP 8   // Define the pin for the endstop signal



void IRAM_ATTR readEncoder();
void IRAM_ATTR readEndstop();

HardwareSerial main_controller(0);  //Create a new HardwareSerial class.

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//pid related vars
float kp = 10;
float kd = 0.4;
float ki = 0.2;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

volatile bool encoder_trigger = LOW;
volatile bool endstop_pressed = false;
volatile int posi = 0;
int pos = 0;
unsigned long previousMillis = 0;  // Stores the last time the loop ran
unsigned long prevMillis = 0;
int endstop_pos = 0;

float degtopos = 7.540271;

float target = 0;
int angle_pos = 0;
float curr_angle = 0;

int angleOUT_pin = 21;
Servo angle_out;

int pos_offset = 4;
int subzero_offset;
bool calibarted = false;

float prev_target = 0;
void setup() {
  Serial.begin(115200);
  //assing all the pins as in or outputs
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(ENDSTOP, INPUT_PULLUP);
  pinMode(PWM_PIN, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);

  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);

  ESP32PWM::allocateTimer(0);  // set timers for the pwm signals
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  angle_out.setPeriodHertz(50);  // standard 50 hz esc signal
  angle_out.attach(angleOUT_pin, 500, 2500);
  //attach the interupts;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP), readEndstop, FALLING);

  find_endpoint();
}

void loop() {
  if (Serial.available()) {
    target = Serial.parseFloat();
    Serial.println(target);
    clearSerialBuffer();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 20) {  // 50hz loop rate
    previousMillis = currentMillis;
    target = read_target_from_pwm();

    if(target < -0.5){
      find_endpoint();
    }
    // get target position
    PID_motor(target);

    send_angle_pwm(curr_angle);
  }

//test code --->
  /*
  target = 0;
  while (abs(target - curr_angle) >= 0.5) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 25) {
      previousMillis = currentMillis;
      //  target = read_target_from_pwm();
      // get target position
      PID_motor(target);

      send_angle_pwm(curr_angle);
    }
  }
  for (float i = 0; i <= 45; i += 0.2) {
    target = i;
    prevMillis = millis();
    while (millis() - prevMillis <= 200) {

      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 25) {
        previousMillis = currentMillis;
        //  target = read_target_from_pwm();
        // get target position
        PID_motor(target);

        send_angle_pwm(curr_angle);
      }
    }
  }
*/
}
//nothing


void setMotor(int dir, float pwmVal, int out1, int out2) {
  if (dir == 1) {
    analogWrite(out1, pwmVal);
    analogWrite(out2, 0);
  } else if (dir == -1) {
    analogWrite(out1, 0);
    analogWrite(out2, pwmVal);
  } else {
    digitalWrite(out1, LOW);
    digitalWrite(out2, LOW);
  }
}

float read_target_from_pwm() {
  // Read the pulse width in microseconds
  unsigned long pulseWidth = pulseIn(20, HIGH);
  // pulseWidth = constrain(pulseWidth, 1000, 2000);
  // Map the pulse width to the target angle
  float target = map(pulseWidth, 500, 2500, -100, 4500);
  target /= 100;
 
  Serial.print("target recieved:");
  Serial.print(target);
  Serial.print("\t");
  Serial.println(pulseWidth);
  target = constrain(target, -1, 45);
  // prev_target = target;
  return target;
}

void send_angle_pwm(float actual_angle) {
  actual_angle *= 100;
  unsigned long pulsewidth = map(actual_angle, 0, 4500, 500, 2500);
  Serial.print("send angle: ");
  Serial.print(actual_angle);
  Serial.print("\t");
  Serial.println(pulsewidth);
  angle_out.writeMicroseconds(pulsewidth);
}

void PID_motor(float target_angle) {

  target_angle = constrain(target_angle, 0, 70);
  target_angle *= degtopos;  // convert target for deg to rotations.

  angle_pos = target_angle - pos_offset;
  // time difference
  long currT = micros();                              // curent time
  float deltaT = ((float)(currT - prevT)) / (1.0e6);  //difference in time
  prevT = currT;

  //get the saved posi(from the ram), and make it an int for furter calculations.
  int pos = 0;
  portENTER_CRITICAL(&mux);
  pos = posi;
  portEXIT_CRITICAL(&mux);

  //calculate the error between the target
  int e = pos - angle_pos;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;


  float u = kp * e + kd * dedt + ki * eintegral;
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  if (angle_pos / degtopos < 20 && dir == -1) {
    kp = 7;
  } else {
    kp = 10;
  }
  float pwr = fabs(u) + 65;

  if (pwr > 255) {
    pwr = 255;
  }
  if (abs(e) <= 1) {
    pwr = 0;
  }
  //if (pwr <= 65) {
  //  pwr = 0;
  // }

  setMotor(dir, pwr, motor1, motor2);  //set the signal to the motor

  eprev = e;
  curr_angle = (pos +4) / degtopos;

  Serial.print("Target: ");
  Serial.print(angle_pos);
  Serial.print("\t");

  Serial.print("pos: ");
  Serial.print(pos);
  Serial.print("\t");

  Serial.print("speed: ");
  Serial.print(pwr);
  Serial.print("\t");

  Serial.print("u: ");
  Serial.print(u);
  Serial.print("\t");

  Serial.print("cur_angle: ");
  Serial.print(curr_angle);
  Serial.print("\t");

  Serial.print("target_angle: ");
  Serial.print((angle_pos+ 4) / degtopos);
  Serial.print("\t");

  Serial.print("direction: ");
  Serial.print(dir);
  Serial.println();

  /*
  Serial.print(target);
  Serial.print(",");

  // Serial.print("pos: ");
  Serial.print(pos);
  Serial.print("\n");
  */
}

void find_endpoint() {
  prevMillis = 0;
  while (!encoder_trigger) {
    setMotor(1, 200, motor1, motor2);
    Serial.println("direction: 1");
    delay(10);
    if (millis() - prevMillis >= 2000) {
      prevMillis = millis();
      Serial.println("move other way -->");
      Serial.println("direction: -1");
      setMotor(-1, 200, motor1, motor2);
      delay(700);
    }
  }
  if (endstop_pressed) {
    setMotor(0, 0, motor1, motor2);
// happens in the interupt code
    endstop_pressed = false;
  }
}

void IRAM_ATTR readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

void IRAM_ATTR readEndstop() {
  encoder_trigger = HIGH;
  endstop_pressed = true;
  if (!calibarted) {
    portENTER_CRITICAL(&mux);
    posi = 0;  // Reset encoder position
    portEXIT_CRITICAL(&mux);
    calibarted = true;
  }

  Serial.println("endstop pressed!!!!!!!!!!");
}
void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();  // Read and discard all characters in the buffer
  }
}

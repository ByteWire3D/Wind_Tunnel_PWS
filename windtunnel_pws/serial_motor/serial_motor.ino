#define ENCA 3     // Define the pin YELLOW encoder wire
#define ENCB 4     //Define the pin WHITE encoder
#define motor1 5   // Define the pin for the first motor output signal
#define motor2 6   // Define the pin for the second motor output signal
#define PWM_PIN 20  // Define the pin for the PWM signal
#define ENDSTOP 8  // Define the pin for the endstop signal



void IRAM_ATTR readEncoder();
void IRAM_ATTR readEndstop();

HardwareSerial main_controller(0);  //Create a new HardwareSerial class.

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//pid related vars
float kp = 3;
float kd = 0.1;
float ki = 0.00;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

volatile bool encoder_trigger = LOW;
volatile bool endstop_pressed = false;
volatile int posi = 0;
unsigned long previousMillis = 0;  // Stores the last time the loop ran
int endstop_pos = 0;

float target = 0;

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
  //attach the interupts;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP), readEndstop, FALLING);


  // while (!main_controller) { delay(10); }
  while (!Serial) { delay(10); }
  //delay(2000);
  find_endpoint();
}

void loop() {
  if (Serial.available()) {
    target = Serial.parseFloat();
    Serial.println(target);
    clearSerialBuffer();
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 25) {
    previousMillis = currentMillis;
    target = get_target_from_pwm();
    // get target position
    PID_motor(target);
  }
}



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

float get_target_from_pwm() {
  // Read the pulse width in microseconds
  unsigned long pulseWidth = pulseIn(PWM_PIN, HIGH);
  constrain(pulseWidth, 1000, 2000);
  // Map the pulse width to the target angle
  float target = map(pulseWidth, 1000, 2000, -45, 45);
  return target;
}

void PID_motor(float target) {
  float degtopos = -7.540271;
  target *= degtopos;  // convert target for deg to rotations.

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
  int e = pos - target;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  float u = kp * e + kd * dedt + ki * eintegral;
  float pwr = fabs(u) + 65;

  if (pwr > 255) {
    pwr = 255;
  }
  if (pwr <= 65) {
    pwr = 0;
  }
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  setMotor(dir, pwr, motor1, motor2);  //set the signal to the motor

  eprev = e;

  Serial.print("Target: ");
  Serial.print(target);
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
  while (!encoder_trigger) {
    setMotor(1, 200, motor1, motor2);
    Serial.println("direction: 1");
    delay(10);
  }
  if (endstop_pressed) {
    setMotor(0, 0, motor1, motor2);
    portENTER_CRITICAL(&mux);
    posi = 0;  // Reset encoder position
    portEXIT_CRITICAL(&mux);
    //delay(2000);
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
  Serial.println("endstop pressed!!!!!!!!!!");
}
void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();  // Read and discard all characters in the buffer
  }
}

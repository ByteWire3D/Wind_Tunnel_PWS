void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  for (float i = -1.0; i < 0; i--) {
    float j = better_contrain(i, 0, 180);
    j -= j;
    Serial.print(i);
    Serial.print("\t");
    Serial.println(j);
    delay(500);
    if(i <= -10){
      break;
    }
  }
  for (float i = 181; i > 180; i++) {
    float j = better_contrain(i, 0, 180);
    j -= j;
    Serial.print(i);
    Serial.print("\t");
    Serial.println(j);
    delay(500);
    if(i >= 190){
      break;
    }
  }
  float value = 181.1;
  value = better_contrain(value, 0, 180);
  value -= value;
  Serial.println(value);
}

void loop() {
  // put your main code here, to run repeatedly:
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
    return value;
  }
}
#include <BleCombo.h>
int value = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting work!");
  Keyboard.begin();
  delay(1000);
}

void loop() {
  if (Keyboard.isConnected()) {

    Keyboard.print(value);
    Keyboard.write(KEY_TAB);
   

    Keyboard.print(value);
    Keyboard.write(KEY_TAB);
  

    Keyboard.print(value);
    Keyboard.write(KEY_TAB);
   

    Keyboard.print(value);
    Keyboard.write(KEY_TAB);
   

    Keyboard.print(value);
    Keyboard.write(KEY_TAB);
  

    Keyboard.write(KEY_RETURN);
    Keyboard.write(KEY_RETURN);

    delay(1000);
    value++;
  }
}

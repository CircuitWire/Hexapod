#include <Servo.h>

int keyInput = 0;

int posX = 0;
int posY = 0;

Servo servo1; // Up + Down
Servo servo2; // Left + Right

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino code starting");
  servo1.attach(3);
  servo2.attach(5);
}

void loop() {
  if (Serial.available()) {
    keyInput = Serial.read();

    Serial.print("Incoming Key: ");
    Serial.println(keyInput, DEC);

    // W | Up
    if (keyInput == 119) {
      posY += 15;
      servo1.write(posY);
      delay(3000);
      keyInput = 0;
    }
    // A | Left
    if (keyInput == 97) {
      posX -= 1;
      servo2.write(posX);
      delay(3000);
      keyInput = 0;
    }
    // S | Down
    if (keyInput == 115) {
      posY -= 15;
      servo1.write(posY);
      delay(3000);
      keyInput = 0;
    }
    // D | Right
    if (keyInput == 100) {
      posX += 1;
      servo2.write(posX);
      delay(3000);
      keyInput = 0;
    }
    // L | Shut Down Servos
    if (keyInput == 108){
      servo1.detach();
      servo2.detach();
    }
    // M | Connect To Servos
    if (keyInput == 108){
      servo1.attach(3);
      servo2.attach(5);
    }
  }
}

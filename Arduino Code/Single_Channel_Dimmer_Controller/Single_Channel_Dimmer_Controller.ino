 /*

  By Spa Electronics 09/06/2018

*/
#include <TimerOne.h>
// FOUND THAT USING TRIAL AND ERROR GIVES ME THE BEST VALUES..
#define DIM_MAX_MICROS 6800
#define BRIGHT_MAX_MICROS 5128//3400 //5128
#define FULL_ON_MARGIN_MICROS 50
#define INTERRUPT 0
#define AC_PIN 10
int offTime = DIM_MAX_MICROS;
String inputString = "";
boolean stringComplete = false;

void serialEvent() {
  //Excpecting 0-100 from the input to end with new line
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void zero_cross_detect() {
  Timer1.restart(); // sync with zero cross detect
  Timer1.attachInterrupt(nowIsTheTime, offTime);
}

void nowIsTheTime() {
  if (offTime < DIM_MAX_MICROS) // FULL OFF?
    digitalWrite(AC_PIN, HIGH);
  if (offTime > (BRIGHT_MAX_MICROS + FULL_ON_MARGIN_MICROS)) // FULL ON?
    digitalWrite(AC_PIN, LOW);
}

void setup() {
  inputString.reserve(100);
  Serial.begin(9600);
  pinMode(AC_PIN, OUTPUT);    // Set the Triac pin as output
  attachInterrupt(INTERRUPT, zero_cross_detect, RISING);
  Timer1.initialize(DIM_MAX_MICROS);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10);
}

void loop() {    // Non time sensitive tasks - ie. read the serial port
  if (stringComplete) {
    Serial.print("dimming to:");
    Serial.println(inputString);
    offTime = map(inputString.toInt(), 0, 100, DIM_MAX_MICROS, BRIGHT_MAX_MICROS);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

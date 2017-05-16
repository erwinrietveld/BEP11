#include <Servo.h>

int radioPin = 7;
int signal;

void setup() {
  Serial.begin(9600);
  pinMode(radioPin, INPUT);
  delay(500);
}

void loop() {
  signal = pulseIn(radioPin,HIGH,25000);
  signal = map(signal, 1049, 1949, 68, 113);
  
  Serial.print("Signal is ");
  Serial.print(signal);
  Serial.println();
  delay(500);

}

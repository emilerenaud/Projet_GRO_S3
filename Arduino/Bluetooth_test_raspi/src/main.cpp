#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i<50; i++)
  {
    Serial1.print("test ");
    Serial1.println(i);
    delay(1000);
  }

}
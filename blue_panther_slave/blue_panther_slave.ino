#include <FreqMeasure.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  FreqMeasure.begin();
  
  mySerial.begin(9600);
}

double sum=0;
int count=0;

int freq = 0;
int minAlarmFreq = 3500;
int maxAlarmFreq = 4000;

void loop() {
  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 100) {
      float frequency = FreqMeasure.countToFrequency(sum / count);
//      Serial.println(frequency);
      
      if (minAlarmFreq < frequency && frequency < maxAlarmFreq) {
        mySerial.write(0x1);
        Serial.println(frequency);
        delay(50);
      }
      sum = 0;
      count = 0;
    }
  }
}

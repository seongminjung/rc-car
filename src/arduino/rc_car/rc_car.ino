#include "ObstacleAvoidance.h"
// analogWrite(PinNum, desired us / 2 -> (Ex: To get 900us -> 1800))
// Analog Read 10 bit (based 5V), so result value 1023 is 5V

// THR: 2000(Reverse) ~ 3000(Stay) ~ 4094(Full)
// SER: 2240(R) ~ 3003(Center) ~ 3858(L)

ObstacleAvoidance obstacle_avoidance;

unsigned long start = millis();
void setup() {
  // RTM_TimerCalc 1.40, RuntimeMicro.com
  // Timer-3 16-bit, Mode-14 Fast, Top=ICR
  // 50 Hz Frequency, Clock is 16 MHz

  TCCR3B = 0x18;  // 0001 1000, Disable Timer
  TCCR3A = 0xA2;  // 1010 0010

  ICR3 = 40000 - 1;
  OCR3A = (int)(ICR3 * 0.25);
  OCR3B = (int)(ICR3 * 0.50);
  TCNT3 = 0x0;

  pinMode(2, OUTPUT);  // OC3b
  pinMode(5, OUTPUT);  // OC3a

  TCCR3B |= 2;  // Prescale=8, Enable Timer

  // For Serial print
  Serial.begin(115200);

  for (int i = 0; i < 20000; i++) {
    analogWrite(2, 3000);
  }
  
}

void loop() { obstacle_avoidance.ir_callback(); 
  Serial.print(millis() - start);
  Serial.println();
  start = millis();
}

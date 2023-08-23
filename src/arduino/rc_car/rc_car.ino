#define THROTTLE_PIN 2
#define SERVO_PIN 5

#define IR_MAX 100000.0
#define IR_MIN 0.0
#define IR_THRESHOLD 40.0
#define THROTTLE_FORWARD 3200
#define THROTTLE_IDLE 3000
#define SERVO_LEFT 2240
#define SERVO_CENTER 3003
#define SERVO_RIGHT 3858

float adc_result[11], distance[11];
int state = 0;

// analogWrite(PinNum, desired us / 2 -> (Ex: To get 900us -> 1800))
// Analog Read 10 bit (based 5V), so result value 1023 is 5V

// THR: 2000(Reverse) ~ 3000(Stay) ~ 4094(Full)
// SER: 2240(R) ~ 3003(Center) ~ 3858(L)

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
  Serial.begin(1200);
}

void loop() {
  get_result();
  update_state();
  do_action();
}

void control_once(int throttle, int servo) {
  analogWrite(THROTTLE_PIN, throttle);
  analogWrite(SERVO_PIN, servo);
}

int killSpikes(){
  int loopcount = 10; 
  int sum=0,x=0,avg=0,high=0,low=0;

  high=low=distance[10];
  for(int i=0;i<loopcount;i++){
    x=distance[i];
    sum +=x;
    if (x > high) high = x;
    if (x < low) low = x;
  }

  return avg = (sum-high-low) / (loopcount-2);
}

void get_result() {
  adc_result[0] = analogRead(A0);
  adc_result[1] = map(analogRead(A1), 0, 1023, 0, 5000);
  adc_result[2] = map(analogRead(A2), 0, 1023, 0, 5000);
  adc_result[3] = analogRead(A3);
  adc_result[4] = map(analogRead(A4), 0, 1023, 0, 5000);
  adc_result[5] = analogRead(A5);
  adc_result[6] = map(analogRead(A6), 0, 1023, 0, 5000);
  adc_result[7] = map(analogRead(A7), 0, 1023, 0, 5000);
  adc_result[8] = map(analogRead(A8), 0, 1023, 0, 5000);
  adc_result[9] = map(analogRead(A9), 0, 1023, 0, 5000);
  adc_result[10] = map(analogRead(A10), 0, 1023, 0, 5000);

  for (int i = 0; i < 11; i++) {
    if (i != 0 || i != 3 || i != 5) {
      float filtered_data =
          distance[i] * 0.8 + (27.61 / (adc_result[i] - 0.1696)) * 100;
      distance[i] = filtered_data;
//      max(min(filtered_data, IR_MAX), IR_MIN);
    }
  }
  distance[0] =
      max(min(distance[0] * 0.8 +
                  float(10650.08 * pow(adc_result[0], -0.935) - 3.937) * 0.2,
              IR_MAX),
          IR_MIN);
  distance[3] =
      max(min(distance[3] * 0.8 +
                  float(10650.08 * pow(adc_result[3], -0.935) - 3.937) * 0.2,
              IR_MAX),
          IR_MIN);
  distance[5] =
      max(min(distance[5] * 0.8 +
                  float(10650.08 * pow(adc_result[5], -0.935) - 3.937) * 0.2,
              IR_MAX),
          IR_MIN);
  Serial.print(killSpikes());
  Serial.println();
}

void update_state() {
  if (distance[1] > IR_THRESHOLD && distance[11] > IR_THRESHOLD)
    state = 0;  // front open
  else
    state = 1;  // front blocked
}

void do_action() {
  switch (state) {
    case 0: {
      // go straight
      float diff = distance[0] - distance[3];
      int corrected_servo = int(SERVO_CENTER + diff * 25.0);
      control_once(THROTTLE_FORWARD, corrected_servo);
      break;
    }
    case 1: {
      if (distance[0] == IR_MAX && distance[3] < IR_MAX)  // left side open
      {
        for (int cnt = 0; cnt < 10000; cnt++) {
          control_once(THROTTLE_FORWARD, SERVO_RIGHT);
        }
      } else if (distance[0] < IR_MAX &&
                 distance[3] == IR_MAX)  // right side open
      {
        for (int cnt = 0; cnt < 10000; cnt++) {
          control_once(THROTTLE_FORWARD, SERVO_LEFT);
        }
      } else if (distance[0] == IR_MAX &&
                 distance[3] == IR_MAX)  // both side open
      {
        float diff = distance[1] - distance[2];
        if (abs(diff) > 3)  // if one front IR is bigger than other by 3cm
        {
          int corrected_servo = int(SERVO_CENTER + (1 / diff) * 2000.0);
          control_once(THROTTLE_FORWARD, corrected_servo);
        } else  // two front IRs are similar
        {
          control_once(THROTTLE_IDLE, SERVO_CENTER);
        }
      } else  // both side closed
      {
        control_once(THROTTLE_IDLE, SERVO_CENTER);
      }
      break;
    }
  }
}

#include "obstacle_avoidance.h"

#define THROTTLE_PIN 2
#define SERVO_PIN 5

#define IR_MAX 60.0
#define IR_MIN 20.0
#define IR_THRESHOLD 40.0
#define THROTTLE_FORWARD 3200
#define THROTTLE_IDLE 3000
#define SERVO_LEFT 2240
#define SERVO_CENTER 3003
#define SERVO_RIGHT 3858

float adc_result[11], distance[11];
int state = 0;

void control_once(int throttle, int servo)
{
  analogWrite(THROTTLE_PIN, throttle);
  analogWrite(SERVO_PIN, servo);
}

void get_result()
{
  adc_result[0] = analogRead(A0); // back left
  adc_result[1] = analogRead(A1); // front left
  adc_result[2] = analogRead(A2); // front right
  adc_result[3] = analogRead(A3); // back right
  adc_result[4] = analogRead(A4);
  adc_result[5] = analogRead(A5);
  adc_result[6] = analogRead(A6);
  adc_result[7] = analogRead(A7);
  adc_result[8] = analogRead(A8);
  adc_result[9] = analogRead(A9);
  adc_result[10] = analogRead(A10);

  for (int i = 0; i < 11; i++)
  {
    float filtered_data = distance[i] * 0.8 + float(10650.08 * pow(adc_result[i], -0.935) - 3.937) * 0.2;
    distance[i] = max(min(filtered_data, IR_MAX), IR_MIN);
  }
}

void update_state()
{
  if (distance[1] > IR_THRESHOLD && distance[2] > IR_THRESHOLD)
    state = 0; // front open
  else
    state = 1; // front blocked
}

void do_action()
{
  switch (state)
  {
  case 0:
  {
    // go straight
    float diff = distance[0] - distance[3];
    int corrected_servo = int(SERVO_CENTER + diff * 25.0);
    control_once(THROTTLE_FORWARD, corrected_servo);
    break;
  }
  case 1:
  {
    if (distance[0] == IR_MAX && distance[3] < IR_MAX) // left side open
    {
      for (int cnt = 0; cnt < 10000; cnt++)
      {
        control_once(THROTTLE_FORWARD, SERVO_RIGHT);
      }
    }
    else if (distance[0] < IR_MAX && distance[3] == IR_MAX) // right side open
    {
      for (int cnt = 0; cnt < 10000; cnt++)
      {
        control_once(THROTTLE_FORWARD, SERVO_LEFT);
      }
    }
    else if (distance[0] == IR_MAX && distance[3] == IR_MAX) // both side open
    {
      float diff = distance[1] - distance[2];
      if (abs(diff) > 3) // if one front IR is bigger than other by 3cm
      {
        int corrected_servo = int(SERVO_CENTER + (1 / diff) * 2000.0);
        control_once(THROTTLE_FORWARD, corrected_servo);
      }
      else // two front IRs are similar
      {
        control_once(THROTTLE_IDLE, SERVO_CENTER);
      }
    }
    else // both side closed
    {
      control_once(THROTTLE_IDLE, SERVO_CENTER);
    }
    break;
  }
  }
}
